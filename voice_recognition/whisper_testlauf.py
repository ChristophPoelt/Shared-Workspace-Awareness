import socket, numpy as np, whisper, queue, threading, time, gc

model = whisper.load_model("tiny")
SAMPLE_RATE = 16000
audio_queue = queue.Queue()

def udp_audio_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 12345))
    print("Listening UDP 0.0.0.0:12345")
    
    while True:
        data, addr = sock.recvfrom(4096)
        #print(f"UDP: {len(data)} bytes")
        
        audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        audio_chunk *= 10.0  # BOOST VOLUME
        audio_chunk = np.clip(audio_chunk, -1.0, 1.0)
        #print(f"Audio range: [{audio_chunk.min():.2f}, {audio_chunk.max():.2f}]")
        
        audio_queue.put(audio_chunk)

def transcribe_audio():
    buffer = np.array([], dtype=np.float32)
    print("Streaming transcription ready")
    
    while True:
        try:
            if audio_queue.empty():
                time.sleep(0.1)
                continue
            
            chunk = audio_queue.get_nowait()
            buffer = np.append(buffer, chunk)
            #print(f"Buffer: {len(buffer)/SAMPLE_RATE:.2f}s")
            
            if len(buffer) >= SAMPLE_RATE * 3.0:  
                audio = buffer[0:SAMPLE_RATE].copy()  
                print(f"Whisper processing audio...")
                
                result = model.transcribe(audio, fp16=False, language='en')
                text = result["text"].strip()
                
                print(f"TRANSCRIPT: '{text}'")
                
                # Keep last 0.5s for overlap
                buffer = buffer[SAMPLE_RATE//2:]
                del audio
                gc.collect()
                
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()

threading.Thread(target=udp_audio_receiver, daemon=True).start()
threading.Thread(target=transcribe_audio, daemon=True).start()

print("Run Windows sender!")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    print("Stopped")
