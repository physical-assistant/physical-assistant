import serial

# Change COM4 to your port (e.g., "/dev/ttyACM0" on Linux)
arduino = serial.Serial('COM6', 9600)

print("Listening...")

while True:
    line = arduino.readline().decode().strip()
    print("Arduino says:", line)
