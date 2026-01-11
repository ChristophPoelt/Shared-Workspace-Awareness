import sounddevice as sd
import numpy as np
import whisper
import queue
import threading

# Load the Whisper model
model = whisper.load_model("base")

# Audio parameters
SAMPLE_RATE = 16000
BUFFER_SIZE = 1024
audio_queue = queue.Queue()

def audio_callback(indata, frames, time, status):
    """Callback function to capture audio data."""
    if status:
        print(status)
    audio_queue.put(indata.copy())

def transcribe_audio():
    """Thread to transcribe audio in real time."""
    while True:
        audio_data = audio_queue.get()
        audio_data = np.concatenate(list(audio_queue.queue))  # Combine buffered audio
        audio_queue.queue.clear()

        # Transcribe the audio
        result = model.transcribe(audio_data.flatten(), language="en")
        print(f"Transcription: {result['text']}")

# Start the transcription thread
transcription_thread = threading.Thread(target=transcribe_audio, daemon=True)
transcription_thread.start()

# Start capturing audio from the microphone
with sd.InputStream(callback=audio_callback, channels=1, samplerate=SAMPLE_RATE, blocksize=BUFFER_SIZE):
    print("Listening... Press Ctrl+C to stop.")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nStopping...")
