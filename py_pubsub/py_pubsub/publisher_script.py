import ctypes
import os
import queue
import threading
import time
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── stderr suppression helpers ────────────────────────────────────────────────

def _suppress_c_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    os.dup2(devnull, 2)
    return devnull, old_stderr

def _restore_c_stderr(devnull, old_stderr):
    os.dup2(old_stderr, 2)
    os.close(devnull)
    os.close(old_stderr)

# ── suppress during imports ───────────────────────────────────────────────────

_dn, _oe = _suppress_c_stderr()
import torch
import whisper
import sounddevice as sd
from silero_vad import load_silero_vad, get_speech_timestamps
_restore_c_stderr(_dn, _oe)

# suppress portaudio exit crash
try:
    ctypes.CDLL("libgcc_s.so.1")
except OSError:
    pass

# ── config ────────────────────────────────────────────────────────────────────

SAMPLE_RATE = 16000
CHUNK_DURATION = 2
LANGUAGE = "en"
MODEL_SIZE = "base"
LOG_PROB_THRESHOLD = -2.0
VAD_THRESHOLD = 0.5
SUPPRESS_STDERR = True  # set False if hardware supports NNPACK

# ── node ──────────────────────────────────────────────────────────────────────

class TranscriberPublisher(Node):
    def __init__(self):
        super().__init__('transcriber_publisher')
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)
        self.transcription_queue = queue.Queue()
        self.audio_queue = queue.Queue()
        self.stop_event = threading.Event()

        self.get_logger().info(f"Loading Whisper model '{MODEL_SIZE}'...")
        _dn, _oe = _suppress_c_stderr()
        self.whisper_model = whisper.load_model(MODEL_SIZE)
        _restore_c_stderr(_dn, _oe)

        self.get_logger().info("Loading Silero VAD model...")
        _dn, _oe = _suppress_c_stderr()
        self.vad_model = load_silero_vad()
        _restore_c_stderr(_dn, _oe)

        threading.Thread(target=self._transcription_worker, daemon=True).start()
        self.create_timer(0.1, self._timer_callback)
        self.get_logger().info("Ready, listening...")

    def _timer_callback(self):
        try:
            text = self.transcription_queue.get_nowait()
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{text}"')
        except queue.Empty:
            pass

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"Audio warning: {status}")
        self.audio_queue.put(indata.copy())

    def _contains_speech(self, audio_chunk: np.ndarray) -> bool:
        if SUPPRESS_STDERR:
            _dn, _oe = _suppress_c_stderr()
        try:
            timestamps = get_speech_timestamps(
                torch.from_numpy(audio_chunk),
                self.vad_model,
                sampling_rate=SAMPLE_RATE,
                threshold=VAD_THRESHOLD,
            )
        finally:
            if SUPPRESS_STDERR:
                _restore_c_stderr(_dn, _oe)
        return len(timestamps) > 0

    def _transcribe(self, audio_chunk: np.ndarray):
        if SUPPRESS_STDERR:
            _dn, _oe = _suppress_c_stderr()
        try:
            result = self.whisper_model.transcribe(
                audio_chunk,
                language=LANGUAGE,
                fp16=False,
                no_speech_threshold=1.0,
                condition_on_previous_text=False,
                logprob_threshold=None,
            )
        finally:
            if SUPPRESS_STDERR:
                _restore_c_stderr(_dn, _oe)
        return result

    def _transcription_worker(self):
        buffer = []
        chunk_samples = int(SAMPLE_RATE * CHUNK_DURATION)
        chunk_count = 0

        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            callback=self._audio_callback,
        ):
            while not self.stop_event.is_set():
                try:
                    data = self.audio_queue.get(timeout=1)
                except queue.Empty:
                    continue

                buffer.append(data)
                if sum(len(d) for d in buffer) < chunk_samples:
                    continue

                chunk_count += 1
                audio_chunk = np.concatenate(buffer, axis=0).flatten().astype(np.float32)
                buffer = []

                if not self._contains_speech(audio_chunk):
                    continue

                start = time.time()
                result = self._transcribe(audio_chunk)
                elapsed = time.time() - start

                segments = result.get("segments", [])
                if not segments:
                    continue
                if np.mean([s["avg_logprob"] for s in segments]) < LOG_PROB_THRESHOLD:
                    continue

                text = result["text"].strip()
                if text:
                    self.get_logger().info(f"[{elapsed:.1f}s] {text}")
                    self.transcription_queue.put(text)

    def shutdown(self):
        self.stop_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = TranscriberPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()