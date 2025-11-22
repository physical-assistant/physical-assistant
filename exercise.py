import serial
import pygame

SERIAL_PORT = "COM6"
BAUD_RATE = 9600

AUDIO_FILE = "gym-3-44581.mp3"

# --- Initialize pygame audio ---
pygame.mixer.init()
print("Audio system ready.")

# --- Load audio file ---
pygame.mixer.music.load(AUDIO_FILE)
print("Loaded:", AUDIO_FILE)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
print("Listening for BUTTON1...")

while True:
    raw = ser.readline().strip()
    if not raw:
        continue

    try:
        msg = raw.decode("utf-8")
    except UnicodeDecodeError:
        continue

    print("Arduino says:", msg)

    if "BUTTON1" in msg:
        print("▶ Playing audio...")
        pygame.mixer.music.play()
