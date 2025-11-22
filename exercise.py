import serial
import pygame

SERIAL_PORT = "COM6"
BAUD_RATE = 9600

# --- Map each button to a different sound file ---
BUTTON_SOUNDS = {
    "BUTTON1": "boing.mp3",
    "BUTTON2": "whistle.mp3",
    "BUTTON3": "train.mp3",
}

# --- Initialize pygame audio ---
pygame.mixer.init()
print("Audio system ready.")

# --- Pre-load all sounds for instant playback ---
sounds = {}
for button, file in BUTTON_SOUNDS.items():
    try:
        sounds[button] = pygame.mixer.Sound(file)
        print(f"Loaded: {file} for {button}")
    except pygame.error as e:
        print(f"Warning: Could not load {file} for {button}: {e}")

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
print(f"Listening for buttons: {', '.join(BUTTON_SOUNDS.keys())}...")

while True:
    raw = ser.readline().strip()
    if not raw:
        continue

    try:
        msg = raw.decode("utf-8")
    except UnicodeDecodeError:
        continue

    print("Arduino says:", msg)

    # Check each button
    for button, sound in sounds.items():
        if button in msg:
            print(f"▶ Playing {BUTTON_SOUNDS[button]}...")
            sound.play()
            break  # Only play one sound per message