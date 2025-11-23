import serial
import pygame
import asyncio
import websockets
import json

SERIAL_PORT = "COM6"
BAUD_RATE = 115200

BUTTON_SOUNDS = {
    "BUTTON1": "strengthmp.mp3",
    "BUTTON2": "balance.mp3",
    "BUTTON3": "Workout3.mp3",
}

# --- Initialize audio ---
pygame.mixer.init()
sounds = {}
for button, file in BUTTON_SOUNDS.items():
    try:
        sounds[button] = pygame.mixer.Sound(file)
        print(f"Loaded: {file}")
    except pygame.error:
        print(f"Could not load {file}")

# --- Shared data (matching React's expected format) ---
latest_data = {
    "HR": 0,
    "ACT": 0,
    "DYN": 0.0,
    "MAG": 0.0,
    "FALL": 0,
    "button_pressed": "",
}

# Track which button is currently playing
currently_playing = None

connected_clients = set()

async def broadcast(data):
    if connected_clients:
        message = json.dumps(data)
        await asyncio.gather(*[client.send(message) for client in connected_clients])

async def websocket_handler(websocket):
    connected_clients.add(websocket)
    print(f"Dashboard connected! Total clients: {len(connected_clients)}")
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.remove(websocket)
        print(f"Dashboard disconnected. Total clients: {len(connected_clients)}")

def parse_serial_message(msg):
    global currently_playing
    
    # Check for button presses
    for button, sound in sounds.items():
        if msg.strip() == button:
            latest_data["button_pressed"] = button
            
            # If this button is already playing, stop it
            if currently_playing == button:
                sound.stop()
                currently_playing = None
                print(f"⏹ Stopped sound for {button}")
            else:
                # Stop any currently playing sound
                if currently_playing:
                    sounds[currently_playing].stop()
                
                # Play the new sound
                sound.play()
                currently_playing = button
                print(f"▶ Playing sound for {button}")
            
            return True

    # Parse sensor data: HR:72,ACT:15,DYN:1.23,MAG:9.81,FALL:0
    if "HR:" in msg:
        try:
            parts = msg.split(",")
            for part in parts:
                if part.startswith("HR:"):
                    latest_data["HR"] = int(part.replace("HR:", ""))
                elif part.startswith("ACT:"):
                    latest_data["ACT"] = int(part.replace("ACT:", ""))
                elif part.startswith("DYN:"):
                    latest_data["DYN"] = float(part.replace("DYN:", ""))
                elif part.startswith("MAG:"):
                    latest_data["MAG"] = float(part.replace("MAG:", ""))
                elif part.startswith("FALL:"):
                    latest_data["FALL"] = int(part.replace("FALL:", ""))
            
            # Clear button after sending sensor data
            latest_data["button_pressed"] = ""
            return True
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e}")
            return False
    return False

async def serial_reader():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to Arduino on {SERIAL_PORT}")

    while True:
        raw = ser.readline().strip()
        if raw:
            try:
                msg = raw.decode("utf-8")
                print(f"Arduino: {msg}")
                if parse_serial_message(msg):
                    await broadcast(latest_data)
            except UnicodeDecodeError:
                pass
        await asyncio.sleep(0.01)

async def main():
    server = await websockets.serve(websocket_handler, "localhost", 8080)
    print("WebSocket server running on ws://localhost:8080")
    print("Waiting for dashboard to connect...")
    await serial_reader()

if __name__ == "__main__":
    asyncio.run(main())