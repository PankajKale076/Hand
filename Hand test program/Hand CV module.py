import cv2
import mediapipe as mp
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

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Open webcam
cap = cv2.VideoCapture(0)

hand_id = 1

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Flip the frame horizontally for a later selfie-view display
        frame = cv2.flip(frame, 1)

        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame to find hands
        result = hands.process(rgb_frame)

        # Draw the hand annotations on the frame
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Get landmark positions
                thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y
                middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y
                ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y
                pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y

                # Map the landmark positions to angles (0-1000)
                thumb_angle = int((1 - thumb_tip) * 1000)
                index_angle = int((1 - index_tip) * 1000)
                middle_angle = int((1 - middle_tip) * 1000)
                ring_angle = int((1 - ring_tip) * 1000)
                pinky_angle = int((1 - pinky_tip) * 1000)

                # Send the angles to the Dexterous Hand
                angles = [pinky_angle, ring_angle, middle_angle, index_angle, thumb_angle, 1000]  # Include thumb
                response = set_angles(hand_id, angles)
                print("Response:", response)

        # Display the frame
        cv2.imshow('Hand Tracking', frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Hand mimic stopped.")

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()
ser.close()
