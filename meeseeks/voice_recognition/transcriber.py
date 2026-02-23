import ctypes
import os

# Redirect C-level stderr during model loading
devnull = os.open(os.devnull, os.O_WRONLY)
old_stderr = os.dup(2)
os.dup2(devnull, 2)

import torch
from silero_vad import load_silero_vad, get_speech_timestamps

os.dup2(old_stderr, 2)  # restore stderr after imports
os.close(devnull)
os.close(old_stderr)

import whisper
import sounddevice as sd
import numpy as np
import queue
import threading
import ctypes
import time
import sys
import os
import torch
from silero_vad import load_silero_vad, get_speech_timestamps

# Suppress portaudio/sounddevice core dump on exit
libgcc = ctypes.CDLL("libgcc_s.so.1")

# ─── Configuration ────────────────────────────────────────────────────────────

SAMPLE_RATE = 16000        # Hz — Whisper's native rate
CHUNK_DURATION = 2         # seconds per recording chunk
LANGUAGE = "en"            # force English
MODEL_SIZE = "base"        # tiny / base 

LOG_PROB_THRESHOLD = -2.0  # secondary filter for low-confidence transcriptions
VAD_THRESHOLD = 0.5        # silero VAD sensitivity (0-1, higher = stricter)
SUPPRESS_STDERR = True     # set to False on hardware that supports NNPACK

# ─── Shared state ─────────────────────────────────────────────────────────────

audio_queue = queue.Queue()
stop_event = threading.Event()
status_lock = threading.Lock()
current_status = "Listening..."

# ─── Status display ───────────────────────────────────────────────────────────

def status_worker():
    spinners = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
    i = 0
    while not stop_event.is_set():
        with status_lock:
            msg = current_status
        sys.stdout.write(f"\r{spinners[i % len(spinners)]} {msg}   ")
        sys.stdout.flush()
        i += 1
        time.sleep(0.1)
    sys.stdout.write("\r")
    sys.stdout.flush()

def set_status(msg: str):
    global current_status
    with status_lock:
        current_status = msg

# ─── Callback: runs in audio thread, called when buffer full, never blocks ─────────────────────────────

def audio_callback(indata, frames, time_info, status):
    if status:
        print(f"\n[Audio warning] {status}")
    audio_queue.put(indata.copy())

# ─── VAD helper ───────────────────────────────────────────────────────────────

def contains_speech(vad_model, audio_chunk: np.ndarray) -> bool:
    audio_tensor = torch.from_numpy(audio_chunk)
    timestamps = get_speech_timestamps(
        audio_tensor,
        vad_model,
        sampling_rate=SAMPLE_RATE,
        threshold=VAD_THRESHOLD,
    )
    return len(timestamps) > 0

# ─── Transcription: runs in its own thread ────────────────────────────────────
# VM does not meet NNPACK requirements so we have to suppress a bunch of errors
def suppress_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    os.dup2(devnull, 2)
    return devnull, old_stderr

def restore_stderr(devnull, old_stderr):
    os.dup2(old_stderr, 2)
    os.close(devnull)
    os.close(old_stderr)

def transcription_worker(whisper_model, vad_model):
    buffer = []
    chunk_samples = int(SAMPLE_RATE * CHUNK_DURATION)
    chunk_count = 0

    while not stop_event.is_set():
        try:
            data = audio_queue.get(timeout=1)
        except queue.Empty:
            continue

        if data is None:
            break

        buffer.append(data)

        total_samples = sum(len(d) for d in buffer)
        if total_samples >= chunk_samples:
            chunk_count += 1
            audio_chunk = np.concatenate(buffer, axis=0).flatten().astype(np.float32)
            buffer = []

            set_status(f"Checking chunk {chunk_count} for speech...")

            if SUPPRESS_STDERR:
                devnull, old_stderr = suppress_stderr()

            speech = contains_speech(vad_model, audio_chunk)
            if not speech:
                if SUPPRESS_STDERR:
                    restore_stderr(devnull, old_stderr)
                set_status(f"Listening... (chunk {chunk_count}: silence)")
                continue

            set_status(f"Transcribing chunk {chunk_count}...")
            start = time.time()

            result = whisper_model.transcribe(
                audio_chunk,
                language=LANGUAGE,
                fp16=False,
                no_speech_threshold=1.0,
                condition_on_previous_text=False,
                logprob_threshold=None,
            )

            if SUPPRESS_STDERR:
                restore_stderr(devnull, old_stderr)
            elapsed = time.time() - start
            set_status(f"Listening... (last chunk took {elapsed:.1f}s)")
            # confidence filter
            segments = result.get("segments", [])
            if not segments:
                continue

            avg_logprob = np.mean([s["avg_logprob"] for s in segments])
            if avg_logprob < LOG_PROB_THRESHOLD:
                continue

            text = result["text"].strip()
            if text:
                handle_transcription(text, elapsed)

# ─── Handle transcription output ──────────────────────────────────────────────

def handle_transcription(text: str, elapsed: float):
    sys.stdout.write(f"\r[Transcription | {elapsed:.1f}s] {text}\n")
    sys.stdout.flush()

    # ─── Phrase detection ──────────────────────────────────────────────────────
    # non functional
    text_lower = text.lower()

    if "pause" in text_lower:
        print("pause command recognized\n")
    elif "continue" in text_lower:
        print("continue command recognized\n")
    elif "abort" in text_lower:
        print("abort command recognized\n")
    elif "whare are you going" in text_lower:
        print("target indication command recognized\n")

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print(f"Loading Whisper model '{MODEL_SIZE}'...")
    whisper_model = whisper.load_model(MODEL_SIZE)

    print("Loading Silero VAD model...")
    vad_model = load_silero_vad()

    print("Ready. Listening... (Ctrl+C to stop)\n")

    t_transcribe = threading.Thread(target=transcription_worker, args=(whisper_model, vad_model), daemon=True)
    t_status = threading.Thread(target=status_worker, daemon=True)

    t_transcribe.start()
    t_status.start()

    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            callback=audio_callback
        ):
            while True:
                sd.sleep(100)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop_event.set()
        audio_queue.put(None)
        t_transcribe.join(timeout=5)
        t_status.join(timeout=2)
        print("Stopped cleanly.")

if __name__ == "__main__":
    main()