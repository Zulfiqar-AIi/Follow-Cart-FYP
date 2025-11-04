import cv2
import numpy as np
import time
import serial
import math
import sys

# --- Bluetooth Serial Setup ---
esp32 = None
com_port = sys.argv[1] if len(sys.argv) > 1 else "COM5"  # Default to COM3 if not provided
try:
    esp32 = serial.Serial(com_port, 115200, timeout=1)
    time.sleep(2)
    print(f"Connected to ESP32 on {com_port}")
except:
    print(f"Failed to connect to Bluetooth on {com_port}")

# --- Configuration ---
SAFE_ZONE_RADIUS = 120  # pixels - adjust this value based on your needs (increased size)
STOP_ZONE_COLOR = (0, 255, 255)  # Yellow color for the safe zone circle

# --- Helper Functions ---
def send_command(cmd):
    if esp32 and esp32.is_open:
        try:
            esp32.write(cmd.encode())
            print(f"[Bluetooth] Sent: {cmd}")
        except Exception as e:
            print(f"Error sending command: {e}")

def determine_speed(distance_cm):
    if distance_cm < 50:
        return '0'
    elif distance_cm < 100:
        return '2'
    elif distance_cm < 150:
        return '4'
    elif distance_cm < 200:
        return '6'
    elif distance_cm < 250:
        return '8'
    else:
        return 'q'

def calculate_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def angle_between_vectors(v1, v2):
    # Convert 2D vectors to 3D by adding zero z-component
    v1_3d = np.array([v1[0], v1[1], 0.0])
    v2_3d = np.array([v2[0], v2[1], 0.0])
    
    unit_v1 = v1_3d / (np.linalg.norm(v1_3d) + 1e-8)
    unit_v2 = v2_3d / (np.linalg.norm(v2_3d) + 1e-8)
    dot_product = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
    angle = math.degrees(np.arccos(dot_product))
    cross = np.cross(unit_v1, unit_v2)
    
    # cross[2] is the z-component (since x and y are zero)
    if cross[2] < 0:
        angle = -angle
    return angle


# --- Camera Setup ---
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# --- Marker Tracking ---
last_seen = {}
timeout = 1.0  # seconds

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    current_time = time.time()

    if ids is not None:
        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            pts = corners[i][0].astype(int)
            center = np.mean(pts, axis=0).astype(int)
            cv2.polylines(frame, [pts], True, (255, 255, 255), 2)
            cv2.putText(frame, str(marker_id), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            last_seen[int(marker_id)] = {'pos': center, 'time': current_time}

    def get_position(marker_id):
        if marker_id in last_seen:
            if current_time - last_seen[marker_id]['time'] < timeout:
                return last_seen[marker_id]['pos']
        return None

    # Marker IDs for your cart and subject
    subject_pos = get_position(999)          # subject
    follower_main_pos = get_position(997)    # back part of follower (main)
    follower_front_pos = get_position(994)   # front part of follower

    # Draw safe zone around subject if visible
    if subject_pos is not None:
        cv2.circle(frame, tuple(subject_pos), SAFE_ZONE_RADIUS, STOP_ZONE_COLOR, 2)
        cv2.putText(frame, f"Safe Zone (R={SAFE_ZONE_RADIUS})", 
                   (subject_pos[0] - 60, subject_pos[1] - SAFE_ZONE_RADIUS - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, STOP_ZONE_COLOR, 1)

    if follower_main_pos is not None and follower_front_pos is not None:
        cv2.arrowedLine(frame, tuple(follower_main_pos), tuple(follower_front_pos), (0, 0, 255), 2)
        cv2.putText(frame, "Front", tuple(follower_front_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    if subject_pos is not None and follower_main_pos is not None and follower_front_pos is not None:
        cv2.arrowedLine(frame, tuple(follower_main_pos), tuple(subject_pos), (0, 255, 0), 3)
        cv2.putText(frame, "To Subject", tuple(subject_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Calculate orientation vector and vector to subject
        follower_vector = np.array(follower_front_pos) - np.array(follower_main_pos)
        subject_vector = np.array(subject_pos) - np.array(follower_main_pos)

        angle_diff = angle_between_vectors(follower_vector, subject_vector)
        distance_pixels = calculate_distance(subject_pos, follower_main_pos)
        
        # Display distance information
        cv2.putText(frame, f"Distance: {distance_pixels:.1f}px", 
                   (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        angle_threshold = 10  # degrees tolerance for "facing"

        # Check if cart is within the safe zone
        if distance_pixels <= SAFE_ZONE_RADIUS:
            # Within safe zone - stop the cart with double command for safety
            send_command('D')  # Stop All
            time.sleep(0.02)   # Small delay
            send_command('S')  # Second trigger command
            cv2.putText(frame, "IN SAFE ZONE - STOPPED", (30, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # Outside safe zone - normal following behavior
            if abs(angle_diff) <= angle_threshold:
                # Facing subject, move forward
                speed = determine_speed(distance_pixels)
                send_command('F')
                time.sleep(0.05)
                send_command(speed)
                cv2.putText(frame, f"MOVING FORWARD - Speed: {speed}", (30, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                # Rotate to face the subject
                if angle_diff > 0:
                    send_command('R')
                    cv2.putText(frame, "TURNING RIGHT", (30, 110), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    send_command('L')
                    cv2.putText(frame, "TURNING LEFT", (30, 110), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                time.sleep(0.05)
                send_command('0')  # Stop forward movement while rotating

    else:
        cv2.putText(frame, "Subject or Markers Lost", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        send_command('D')  # Stop All if subject or markers lost

    # Display configuration info
    cv2.putText(frame, f"Safe Zone Radius: {SAFE_ZONE_RADIUS}px", (30, frame.shape[0] - 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("ArUco Follower System", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
if esp32 and esp32.is_open:
    esp32.close()