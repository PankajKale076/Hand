import speech_recognition as sr
import serial
import time

# Configure the serial connection (adjust the port as needed)
ser = serial.Serial(
    port='COM4',       # Replace 'COM3' with the appropriate port for your setup
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Function to calculate checksum
def calculate_checksum(data):
    return sum(data) & 0xFF

# Function to write to a register of the Dexterous Hand
def write_register(hand_id, address, values):
    packet_header = b'\xEB\x90'
    write_register_flag = b'\x12'
    length = len(values) + 3
    address_l = address & 0xFF
    address_h = (address >> 8) & 0xFF
    data = [hand_id, length, write_register_flag[0], address_l, address_h] + values
    checksum = calculate_checksum(data)
    command = packet_header + bytes(data) + bytes([checksum])
    ser.write(command)
    response = ser.read(9)
    return response

# Function to set angles
def set_angles(hand_id, angles):
    address = 0x05CE  # Start address for ANGLE_SET
    values = []
    for angle in angles:
        values.extend([angle & 0xFF, (angle >> 8) & 0xFF])
    return write_register(hand_id, address, values)

# Initialize the recognizer
r = sr.Recognizer()

# Define voice commands and corresponding angles (thumb always open at 1000)
voice_commands = {
    "open hand": [1000, 1000, 1000, 1000, 1000, 1000],
    "close hand": [0, 0, 0, 0, 0, 1000],
}

# Function to recognize the wake word
def recognize_wake_word():
    with sr.Microphone() as source:
        print("Say 'hey siri' to start...")
        audio = r.listen(source)

    try:
        command = r.recognize_google(audio).lower()
        print(f"You said: {command}")
        if command == "hey siri":
            return True
        else:
            print("Wake word not recognized.")
            return False
    except sr.UnknownValueError:
        print("Could not understand the audio.")
        return False
    except sr.RequestError as e:
        print(f"Could not request results; {e}")
        return False

# Function to recognize voice command and control the Dexterous Hand
def recognize_voice_command():
    with sr.Microphone() as source:
        print("Say 'open hand' or 'close hand'...")
        audio = r.listen(source)

    try:
        command = r.recognize_google(audio).lower()
        print(f"You said: {command}")
        if command in voice_commands:
            angles = voice_commands[command]
            response = set_angles(1, angles)
            print(f"Response: {response}")
        else:
            print("Command not recognized. Please say 'open hand' or 'close hand'.")
    except sr.UnknownValueError:
        print("Could not understand the audio. Please try again.")
    except sr.RequestError as e:
        print(f"Could not request results; {e}")

# Main loop to continuously listen for the wake word and then for voice commands
try:
    while True:
        if recognize_wake_word():
            print("Wake word detected. Listening for commands...")
            while True:
                recognize_voice_command()
except KeyboardInterrupt:
    print("Voice control stopped.")

# Close the serial connection
ser.close()
