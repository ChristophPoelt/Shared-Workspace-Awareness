import ctypes
import os
import queue
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configuration
SAMPLE_RATE = 16000        # Hz (Whisper native rate)
CHUNK_DURATION = 2         # seconds per recording chunk
LANGUAGE = "en"           # force English
MODEL_SIZE = "base"       # tiny / base

LOG_PROB_THRESHOLD = -2.0  # secondary filter for low-confidence transcriptions
VAD_THRESHOLD = 0.5        # silero VAD sensitivity (0-1, higher = stricter)
SUPPRESS_STDERR = False    # set True on hardware that emits noisy backend warnings
VOICE_COMMAND_TOPIC_DEFAULT = "/voice_commands"


def _suppress_c_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    os.dup2(devnull, 2)
    return devnull, old_stderr


def _restore_c_stderr(devnull, old_stderr):
    os.dup2(old_stderr, 2)
    os.close(devnull)
    os.close(old_stderr)


def _load_voice_dependencies():
    # Silence noisy native-library stderr during import/model backend probing.
    devnull, old_stderr = _suppress_c_stderr()
    try:
        import torch  # noqa: PLC0415
        from silero_vad import get_speech_timestamps, load_silero_vad  # noqa: PLC0415
        import whisper  # noqa: PLC0415
        import sounddevice as sd  # noqa: PLC0415
    except Exception as exc:
        raise RuntimeError(
            "Failed to import transcriber voice dependencies. Activate the Python environment "
            "you will use for 'colcon build', install Torch (platform-specific), then run "
            "`pip install -r requirements-voice.txt`. "
            f"Original error: {exc}"
        ) from exc
    finally:
        _restore_c_stderr(devnull, old_stderr)

    # Preserve previous behavior that preloads libgcc to avoid some PortAudio exit crashes.
    try:
        ctypes.CDLL("libgcc_s.so.1")
    except OSError:
        pass

    return {
        "torch": torch,
        "whisper": whisper,
        "sd": sd,
        "load_silero_vad": load_silero_vad,
        "get_speech_timestamps": get_speech_timestamps,
    }


class TranscriberNode(Node):
    def __init__(self):
        super().__init__("transcriber")
        self.declare_parameter("voice_command_topic", VOICE_COMMAND_TOPIC_DEFAULT)
        self.voice_command_topic = str(
            self.get_parameter("voice_command_topic").value or VOICE_COMMAND_TOPIC_DEFAULT
        )
        self.voice_pub = self.create_publisher(String, self.voice_command_topic, 10)

        self.audio_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.status_lock = threading.Lock()
        self.current_status = "Listening..."
        self._shutdown_lock = threading.Lock()
        self._shutdown_requested = False

        self._deps = None
        self._whisper_model = None
        self._vad_model = None
        self._t_transcribe = None
        self._t_status = None

    def set_status(self, msg: str):
        with self.status_lock:
            self.current_status = msg

    def _get_status(self) -> str:
        with self.status_lock:
            return self.current_status

    def request_stop(self):
        with self._shutdown_lock:
            if self._shutdown_requested:
                return
            self._shutdown_requested = True
        self.stop_event.set()
        self.audio_queue.put(None)

    def shutdown_workers(self):
        self.request_stop()
        if self._t_transcribe is not None:
            self._t_transcribe.join(timeout=5)
        if self._t_status is not None:
            self._t_status.join(timeout=2)

    def status_worker(self):
        spinners = ["|", "/", "-", "\\"]
        i = 0
        while not self.stop_event.is_set():
            msg = self._get_status()
            try:
                sys.stdout.write(f"\r{spinners[i % len(spinners)]} {msg}   ")
                sys.stdout.flush()
            except Exception:
                return
            i += 1
            time.sleep(0.1)
        try:
            sys.stdout.write("\r")
            sys.stdout.flush()
        except Exception:
            pass

    def audio_callback(self, indata, frames, time_info, status):
        del frames, time_info
        if status:
            print(f"\n[Audio warning] {status}")
        if not self.stop_event.is_set():
            self.audio_queue.put(indata.copy())

    def contains_speech(self, audio_chunk: np.ndarray) -> bool:
        torch = self._deps["torch"]
        audio_tensor = torch.from_numpy(audio_chunk)
        timestamps = self._deps["get_speech_timestamps"](
            audio_tensor,
            self._vad_model,
            sampling_rate=SAMPLE_RATE,
            threshold=VAD_THRESHOLD,
        )
        return len(timestamps) > 0

    def handle_transcription(self, text: str, elapsed: float):
        sys.stdout.write(f"\r[Transcription | {elapsed:.1f}s] {text}\n")
        sys.stdout.flush()

        if not rclpy.ok():
            return

        msg = String()
        msg.data = text
        try:
            self.voice_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"publish failed: {exc}")

    def transcription_worker(self):
        buffer = []
        chunk_samples = int(SAMPLE_RATE * CHUNK_DURATION)
        chunk_count = 0

        try:
            while not self.stop_event.is_set():
                try:
                    data = self.audio_queue.get(timeout=1)
                except queue.Empty:
                    continue

                if data is None:
                    break

                buffer.append(data)
                total_samples = sum(len(d) for d in buffer)
                if total_samples < chunk_samples:
                    continue

                chunk_count += 1
                audio_chunk = np.concatenate(buffer, axis=0).flatten().astype(np.float32)
                buffer = []

                self.set_status(f"Checking chunk {chunk_count} for speech...")

                devnull = old_stderr = None
                if SUPPRESS_STDERR:
                    devnull, old_stderr = _suppress_c_stderr()

                try:
                    speech = self.contains_speech(audio_chunk)
                finally:
                    if SUPPRESS_STDERR and devnull is not None and old_stderr is not None:
                        _restore_c_stderr(devnull, old_stderr)

                if not speech:
                    self.set_status(f"Listening... (chunk {chunk_count}: silence)")
                    continue

                self.set_status(f"Transcribing chunk {chunk_count}...")
                start = time.time()
                result = self._whisper_model.transcribe(
                    audio_chunk,
                    language=LANGUAGE,
                    fp16=False,
                    no_speech_threshold=1.0,
                    condition_on_previous_text=False,
                    logprob_threshold=None,
                )
                elapsed = time.time() - start
                self.set_status(f"Listening... (last chunk took {elapsed:.1f}s)")

                segments = result.get("segments", [])
                if not segments:
                    continue

                avg_logprob = np.mean([segment.get("avg_logprob", 0.0) for segment in segments])
                if avg_logprob < LOG_PROB_THRESHOLD:
                    continue

                text = result.get("text", "").strip()
                if text:
                    self.handle_transcription(text, elapsed)
        except Exception as exc:
            self.get_logger().error(f"Transcription worker crashed: {exc}")
            self.request_stop()

    def _load_models(self):
        self._deps = _load_voice_dependencies()

        self.get_logger().info(f"Loading Whisper model '{MODEL_SIZE}'...")
        self._whisper_model = self._deps["whisper"].load_model(MODEL_SIZE)

        self.get_logger().info("Loading Silero VAD model...")
        self._vad_model = self._deps["load_silero_vad"]()

    def run(self) -> int:
        try:
            self._load_models()
        except Exception as exc:
            self.get_logger().error(str(exc))
            return 1

        self.get_logger().info(
            f"Ready. Listening... (Ctrl+C to stop) Publishing transcriptions to {self.voice_command_topic}"
        )

        self._t_transcribe = threading.Thread(target=self.transcription_worker, daemon=True)
        self._t_status = threading.Thread(target=self.status_worker, daemon=True)
        self._t_transcribe.start()
        self._t_status.start()

        sd = self._deps["sd"]
        try:
            with sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype="float32",
                callback=self.audio_callback,
            ):
                while rclpy.ok() and not self.stop_event.is_set():
                    rclpy.spin_once(self, timeout_sec=0.0)
                    sd.sleep(100)
        except KeyboardInterrupt:
            self.get_logger().info("Stopping transcriber (KeyboardInterrupt)")
        except Exception as exc:
            self.get_logger().error(f"Audio input/transcriber runtime error: {exc}")
            return_code = 1
            self.request_stop()
            self.shutdown_workers()
            return return_code

        self.request_stop()
        self.shutdown_workers()
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = TranscriberNode()

    previous_handlers = {}

    def _handle_signal(signum, _frame):
        try:
            signame = signal.Signals(signum).name
        except Exception:
            signame = str(signum)
        node.get_logger().info(f"Received {signame}, shutting down transcriber")
        node.request_stop()

    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, _handle_signal)
        except Exception:
            # Signal registration can fail in some embedded/non-main-thread contexts.
            continue

    try:
        return node.run()
    finally:
        node.shutdown_workers()
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

        for signum, handler in previous_handlers.items():
            try:
                signal.signal(signum, handler)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
