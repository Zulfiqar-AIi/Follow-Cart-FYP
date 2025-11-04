import sys
import cv2
import numpy as np
import time
import serial
import math

# --- Bluetooth Serial Setup ---
esp32 = None
com_port = sys.argv[1] if len(sys.argv) > 1 else "COM8"  # Default to COM3 if not provided
try:
    esp32 = serial.Serial(com_port, 115200, timeout=1)
    time.sleep(2)
    print(f"Connected to ESP32 on {com_port}")
except:
    print(f"Failed to connect to Bluetooth on {com_port}")

# --- Helper Functions ---
def send_command(cmd):
    if esp32 and esp32.is_open:
        try:
            esp32.write(cmd.encode())
            print(f"[Bluetooth] Sent: {cmd}")
        except Exception as e:
            print(f"Error sending command: {e}")

def determine_speed(distance_cm):
    if distance_cm < 50: return '0'
    elif distance_cm < 100: return '2'
    elif distance_cm < 150: return '4'
    elif distance_cm < 200: return '6'
    elif distance_cm < 250: return '8'
    else: return 'q'

def calculate_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2)) * 0.1  # Convert to cm

def angle_between_vectors(v1, v2):
    v1_3d = np.array([v1[0], v1[1], 0.0])
    v2_3d = np.array([v2[0], v2[1], 0.0])
    unit_v1 = v1_3d / (np.linalg.norm(v1_3d) + 1e-8)
    unit_v2 = v2_3d / (np.linalg.norm(v2_3d) + 1e-8)
    dot_product = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
    angle = math.degrees(np.arccos(dot_product))
    cross = np.cross(unit_v1, unit_v2)
    if cross[2] < 0: angle = -angle
    return angle

# --- Camera Setup ---
cap1 = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(1, cv2.CAP_DSHOW)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

# --- Tracking State ---
class TrackingState:
    def __init__(self):
        self.active_camera = 1  # Start with camera 1
        self.last_seen = {}  # Stores marker data from both cameras
        self.last_switch_time = time.time()
        self.switch_cooldown = 2.0  # Minimum time between camera switches
        self.timeout = 1.0  # Marker timeout in seconds

state = TrackingState()

def process_frame(cap, camera_num):
    ret, frame = cap.read()
    if not ret:
        print(f"Error reading from Camera {camera_num}")
        return None
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            pts = corners[i][0].astype(int)
            center = np.mean(pts, axis=0).astype(int)
            
            # Draw marker info on the frame
            cv2.polylines(frame, [pts], True, (255, 255, 255), 2)
            cv2.putText(frame, str(marker_id), tuple(center), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Store marker data
            state.last_seen[int(marker_id)] = {
                'pos': center,
                'time': time.time(),
                'camera': camera_num
            }
    
    return frame

def get_position(marker_id, camera=None):
    if marker_id in state.last_seen:
        marker_data = state.last_seen[marker_id]
        if time.time() - marker_data['time'] < state.timeout:
            if camera is None or marker_data['camera'] == camera:
                return marker_data['pos']
    return None

# --- Main Loop ---
while True:
    current_time = time.time()
    
    # Process both camera frames
    frame1 = process_frame(cap1, 1)
    frame2 = process_frame(cap2, 2)
    
    # Determine which markers are visible in each camera
    subject_in_cam1 = get_position(999, 1) is not None
    subject_in_cam2 = get_position(999, 2) is not None
    follower_in_cam1 = (get_position(997, 1) is not None and 
                       get_position(994, 1) is not None)
    follower_in_cam2 = (get_position(997, 2) is not None and 
                       get_position(994, 2) is not None)
    
    # Camera switching logic
    if current_time - state.last_switch_time > state.switch_cooldown:
        if state.active_camera == 1 and not subject_in_cam1 and subject_in_cam2:
            state.active_camera = 2
            state.last_switch_time = current_time
            print("Switched to Camera 2 - Subject detected")
        elif state.active_camera == 2 and not subject_in_cam2 and subject_in_cam1:
            state.active_camera = 1
            state.last_switch_time = current_time
            print("Switched to Camera 1 - Subject detected")
    
    # Get positions from active camera
    active_frame = frame1 if state.active_camera == 1 else frame2
    subject_pos = get_position(999, state.active_camera)
    follower_main_pos = get_position(997, state.active_camera)
    follower_front_pos = get_position(994, state.active_camera)
    
    # Tracking and movement logic
    if subject_pos is not None and follower_main_pos is not None and follower_front_pos is not None:
        # Draw orientation and target vectors
        cv2.arrowedLine(active_frame, tuple(follower_main_pos), tuple(follower_front_pos), 
                       (0, 0, 255), 2)
        cv2.putText(active_frame, "Front", tuple(follower_front_pos), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        cv2.arrowedLine(active_frame, tuple(follower_main_pos), tuple(subject_pos), 
                       (0, 255, 0), 3)
        cv2.putText(active_frame, "To Subject", tuple(subject_pos), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Calculate movement vectors
        follower_vector = np.array(follower_front_pos) - np.array(follower_main_pos)
        subject_vector = np.array(subject_pos) - np.array(follower_main_pos)
        
        angle_diff = angle_between_vectors(follower_vector, subject_vector)
        distance_cm = calculate_distance(subject_pos, follower_main_pos)
        speed = determine_speed(distance_cm)
        
        angle_threshold = 10  # degrees tolerance for "facing"
        
        if abs(angle_diff) <= angle_threshold:
            # Facing subject, move forward
            send_command('F')
            time.sleep(0.05)
            send_command(speed)
        else:
            # Rotate to face the subject
            if angle_diff > 0:
                send_command('R')
            else:
                send_command('L')
            time.sleep(0.05)
            send_command('0')  # Stop forward movement while rotating
    else:
        cv2.putText(active_frame, "Subject or Markers Lost", (30, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        send_command('D')  # Stop All if subject or markers lost
    
    # Display active camera feed
    if active_frame is not None:
        cv2.putText(active_frame, f"Active: Camera {state.active_camera}", (30, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.imshow("ArUco Follower System", active_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap1.release()
cap2.release()
cv2.destroyAllWindows()
if esp32 and esp32.is_open:
    esp32.close()