import socket
import numpy as np
import whisper
import queue
import threading
import time
import gc

# Configuration
host_ip = '<WINDOWS_HOST_IP>'  # Replace with Windows host IP
port = 9633
CHUNK = 1024
SAMPLE_RATE = 16000  # Whisper expects 16kHz
audio_queue = queue.Queue()

# Load Whisper model (use "tiny" for speed, "base" for better accuracy)
model = whisper.load_model("tiny")
print("Whisper model loaded")

def udp_audio_receiver():
    """Receive UDP audio chunks and process like your reference code"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    
    # Initiate connection (send hello to Windows sender)
    sock.sendto(b'Hello', (host_ip, port))
    print(f"Listening for audio from {host_ip}:{port}")
    
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            
            # Convert raw bytes to normalized float32 exactly like your code
            audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            audio_chunk *= 10.0  # BOOST VOLUME
            audio_chunk = np.clip(audio_chunk, -1.0, 1.0)
            
            audio_queue.put(audio_chunk)
            
        except Exception as e:
            print(f"Receiver error: {e}")
            break

def transcribe_audio():
    """Streaming transcription with 3-second buffers, exactly like your code"""
    buffer = np.array([], dtype=np.float32)
    print("Streaming transcription ready")
    
    while True:
        try:
            if audio_queue.empty():
                time.sleep(0.1)
                continue
            
            chunk = audio_queue.get_nowait()
            buffer = np.append(buffer, chunk)
            
            if len(buffer) >= SAMPLE_RATE * 3.0:  # Process every 3 seconds
                audio = buffer[0:SAMPLE_RATE].copy()  # First second for Whisper
                print(f"Whisper processing {len(audio)/SAMPLE_RATE:.1f}s audio...")
                
                result = model.transcribe(audio, fp16=False, language='en')
                text = result["text"].strip()
                
                print(f"TRANSCRIPT: '{text}'")
                
                # Keep last 0.5s for overlap (exactly like your code)
                buffer = buffer[SAMPLE_RATE//2:]
                del audio
                gc.collect()
                
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Transcription error: {e}")
            import traceback
            traceback.print_exc()

# Start threads
threading.Thread(target=udp_audio_receiver, daemon=True).start()
threading.Thread(target=transcribe_audio, daemon=True).start()

print("Whisper receiver running! Run Windows sender.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped")
