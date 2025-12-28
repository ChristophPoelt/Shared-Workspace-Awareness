# wsl can't access mics directly, so capture audio on Windows and send via UDP to WSL
# we will have to figure out how to configure in VM

import pyaudio
import socket
import time

# Match WSL receiver settings
SAMPLE_RATE = 16000
CHANNELS = 1
BUFFER_SIZE = 1024
UDP_IP = "172.30.253.170"  # WSL IP (or find with `hostname -I` in WSL)
UDP_PORT = 12345

p = pyaudio.PyAudio()
print(print(pyaudio.get_portaudio_version_text()))

info = p.get_default_input_device_info()
print(f"Using mic: {info['name']}")

stream = p.open(format=pyaudio.paInt16,
                channels=CHANNELS,
                rate=SAMPLE_RATE,
                input=True,
                frames_per_buffer=BUFFER_SIZE)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Sending mic audio to WSL... Press Ctrl+C to stop.")
try:
    while True:
        data = stream.read(BUFFER_SIZE, exception_on_overflow=False)
        sock.sendto(data, (UDP_IP, UDP_PORT))
except KeyboardInterrupt:
    print("Stopping...")
finally:
    stream.stop_stream()
    stream.close()
    p.terminate()