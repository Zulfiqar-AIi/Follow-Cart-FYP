import sys
import cv2
import numpy as np
import time
import serial
from ultralytics import YOLO

# --- Bluetooth Serial Setup ---
esp32 = None
com_port = sys.argv[1] if len(sys.argv) > 1 else "COM8"  # Default to COM3 if not provided
try:
    esp32 = serial.Serial(com_port, 115200, timeout=1)
    time.sleep(2)
    print(f"Connected to ESP32 on {com_port}")
except:
    print(f"Failed to connect to Bluetooth on {com_port}")

def send_command(cmd):
    if esp32 and esp32.is_open:
        esp32.write(cmd.encode())
        print(f"[Bluetooth] Sent: {cmd}")

def determine_direction(dx, dy, threshold=30):
    if abs(dx) < threshold and dy < -threshold:
        return 'F'  # Forward
    elif dx < -threshold and abs(dy) < threshold:
        return 'L'  # Left
    elif dx > threshold and abs(dy) < threshold:
        return 'R'  # Right
    elif abs(dx) < threshold and dy > threshold:
        return 'B'  # Backward
    return 'S'  # Stop

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

# --- Load YOLOv8 Model ---
model = YOLO("yolov8n.pt")

# --- ArUco Detection Setup ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# --- Open Two Cameras ---
cam1 = cv2.VideoCapture(0)
cam2 = cv2.VideoCapture(1)

# --- Obstacle Rerouting State ---
rerouting = False
rerouting_start_time = 0
rerouting_direction = None
rerouting_duration = 1.5  # seconds

while True:
    for cam_index, cap in enumerate([cam1, cam2]):
        ret, frame = cap.read()
        if not ret:
            continue

        current_cam = cam_index + 1
        frame_height, frame_width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ArUco detection
        corners, ids, _ = detector.detectMarkers(gray)
        subject_pos = front_pos = back_pos = None

        if ids is not None:
            ids = ids.flatten()
            for i, marker_id in enumerate(ids):
                pts = corners[i][0].astype(int)
                center = np.mean(pts, axis=0).astype(int)
                if marker_id == 999:
                    subject_pos = center
                elif marker_id == 994:
                    front_pos = center
                elif marker_id == 997:
                    back_pos = center
                cv2.polylines(frame, [pts], True, (255, 255, 255), 2)
                cv2.putText(frame, f"ID {marker_id}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if subject_pos is not None and front_pos is not None and back_pos is not None:
            break  # Valid tracking points found

    if not ret:
        continue

    current_time = time.time()

    # --- Rerouting logic for obstacle (no rotate here as per request) ---
    if rerouting:
        if current_time - rerouting_start_time < rerouting_duration:
            send_command(rerouting_direction)
            send_command('4')
            cv2.putText(frame, f"Rerouting: {rerouting_direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow(f"Camera {current_cam}", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue
        else:
            rerouting = False

    if subject_pos is not None and front_pos is not None and back_pos is not None:
        # Orientation logic: Rotate if back is closer than front
        dist_front = calculate_distance(subject_pos, front_pos)
        dist_back = calculate_distance(subject_pos, back_pos)

        if dist_back < dist_front - 20:
            dx = subject_pos[0] - ((front_pos[0] + back_pos[0]) // 2)
            rerouting_direction = 'L' if dx > 0 else 'R'
            send_command(rerouting_direction)
            send_command('4')
            time.sleep(1.0)  # small rotation duration
            send_command('S')
            continue  # Skip current frame commands

        follower_pos = ((front_pos[0] + back_pos[0]) // 2, (front_pos[1] + back_pos[1]) // 2)
        cv2.line(frame, follower_pos, tuple(subject_pos), (0, 255, 0), 2)

        # YOLOv8 Obstacle Detection
        results = model(frame, verbose=False)[0]
        obstacle_on_path = False

        for box in results.boxes:
            cls = int(box.cls[0])
            label = model.names[cls]
            if label in ['person', 'chair', 'car', 'bottle', 'cup']:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                path_vec = np.array(subject_pos) - np.array(follower_pos)
                obj_vec = np.array([cx, cy]) - np.array(follower_pos)
                proj_len = np.dot(path_vec, obj_vec) / np.linalg.norm(path_vec)

                if 0 < proj_len < np.linalg.norm(path_vec):
                    obstacle_on_path = True
                    rerouting_direction = 'B'  # Instead of rotating, reverse
                    rerouting = True
                    rerouting_start_time = current_time
                    send_command(rerouting_direction)
                    send_command('4')
                    break

        if not obstacle_on_path and not rerouting:
            dx = subject_pos[0] - follower_pos[0]
            dy = subject_pos[1] - follower_pos[1]
            direction = determine_direction(dx, dy)
            distance = calculate_distance(subject_pos, follower_pos)
            speed = determine_speed(distance)
            send_command(direction)
            time.sleep(0.05)
            send_command(speed)

    cv2.putText(frame, f"Camera {current_cam} Active", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.imshow(f"Camera {current_cam}", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam1.release()
cam2.release()
cv2.destroyAllWindows()
if esp32 and esp32.is_open:
    esp32.close()
