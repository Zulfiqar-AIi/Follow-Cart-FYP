import cv2
import numpy as np
import time
import serial
import math
import sys

# --- Bluetooth Serial Setup ---
esp32 = None
com_port = sys.argv[1] if len(sys.argv) > 1 else "COM1"  # Default to COM3 if not provided
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

def point_line_distance(point, line_start, line_end):
    """Calculate perpendicular distance from point to line segment"""
    A = np.array(point)
    B = np.array(line_start)
    C = np.array(line_end)
    
    # Vector from B to C
    BC = C - B
    # Vector from B to A
    BA = A - B
    
    # Project BA onto BC
    if np.dot(BC, BC) == 0:  # B and C are the same point
        return np.linalg.norm(BA)
    
    t = np.dot(BA, BC) / np.dot(BC, BC)
    t = max(0, min(1, t))  # Clamp t to [0, 1] for line segment
    
    # Find closest point on line segment
    closest_point = B + t * BC
    return np.linalg.norm(A - closest_point)

def find_obstacles_in_path(follower_pos, subject_pos, all_markers, obstacle_radius=60):
    """Find all obstacles blocking the direct path to subject"""
    obstacles = []
    
    # Exclude robot markers and subject from being obstacles
    excluded_ids = {997, 994, 999}  # robot main, robot front, subject
    
    for marker_id, marker_data in all_markers.items():
        if marker_id not in excluded_ids:
            obstacle_pos = marker_data['pos']
            distance_to_path = point_line_distance(obstacle_pos, follower_pos, subject_pos)
            if distance_to_path < obstacle_radius:
                obstacles.append({
                    'id': marker_id,
                    'pos': obstacle_pos,
                    'distance_to_robot': calculate_distance(follower_pos, obstacle_pos)
                })
    
    # Sort obstacles by distance to robot (closest first)
    obstacles.sort(key=lambda x: x['distance_to_robot'])
    return obstacles

def calculate_waypoint(follower_pos, subject_pos, obstacle_pos, offset_distance=80):
    """Calculate waypoint to go around obstacle"""
    if obstacle_pos is None:
        return subject_pos
    
    # Vector from obstacle to subject
    obs_to_subject = np.array(subject_pos) - np.array(obstacle_pos)
    obs_to_follower = np.array(follower_pos) - np.array(obstacle_pos)
    
    # Normalize the perpendicular vector
    if np.linalg.norm(obs_to_subject) == 0:
        return subject_pos
    
    # Create perpendicular vector (rotate 90 degrees)
    perp_vector = np.array([-obs_to_subject[1], obs_to_subject[0]])
    perp_vector = perp_vector / (np.linalg.norm(perp_vector) + 1e-8)
    
    # Determine which side to go around (choose side closer to follower)
    side1 = np.array(obstacle_pos) + perp_vector * offset_distance
    side2 = np.array(obstacle_pos) - perp_vector * offset_distance
    
    dist1 = calculate_distance(follower_pos, side1)
    dist2 = calculate_distance(follower_pos, side2)
    
    # Choose the closer side
    waypoint = side1 if dist1 < dist2 else side2
    return waypoint.astype(int)

def draw_path_plan(frame, follower_pos, subject_pos, obstacles, waypoint, avoiding_obstacle):
    """Draw the planned path on the frame"""
    if avoiding_obstacle and waypoint is not None:
        # Draw path: follower -> waypoint -> subject
        cv2.arrowedLine(frame, tuple(follower_pos), tuple(waypoint), (255, 0, 255), 3)
        cv2.arrowedLine(frame, tuple(waypoint), tuple(subject_pos), (255, 0, 255), 3)
        cv2.circle(frame, tuple(waypoint), 8, (255, 0, 255), -1)
        cv2.putText(frame, "Waypoint", tuple(waypoint + np.array([10, -10])), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Draw obstacle warning zones for all obstacles
        for obstacle in obstacles:
            obstacle_pos = obstacle['pos']
            obstacle_id = obstacle['id']
            cv2.circle(frame, tuple(obstacle_pos), 60, (0, 0, 255), 2)
            cv2.putText(frame, f"OBSTACLE {obstacle_id}", tuple(obstacle_pos + np.array([10, 30])), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
    else:
        # Draw direct path
        cv2.arrowedLine(frame, tuple(follower_pos), tuple(subject_pos), (0, 255, 0), 3)

# --- Camera Setup ---
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# --- Marker Tracking ---
last_seen = {}
timeout = 1.0  # seconds

# --- Obstacle Avoidance State ---
current_target = None  # Current navigation target (subject or waypoint)
waypoint_reached_threshold = 30  # pixels
avoiding_obstacle = False

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
            
            # Different colors for different markers
            if marker_id == 999:  # Subject
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                cv2.putText(frame, f"SUBJECT {marker_id}", tuple(center), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            elif marker_id in [997, 994]:  # Robot markers
                cv2.polylines(frame, [pts], True, (255, 255, 255), 2)
                cv2.putText(frame, f"ROBOT {marker_id}", tuple(center), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:  # All other markers are potential obstacles
                cv2.polylines(frame, [pts], True, (0, 0, 255), 2)
                cv2.putText(frame, f"OBS {marker_id}", tuple(center), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            last_seen[int(marker_id)] = {'pos': center, 'time': current_time}

    def get_position(marker_id):
        if marker_id in last_seen:
            if current_time - last_seen[marker_id]['time'] < timeout:
                return last_seen[marker_id]['pos']
        return None

    # Marker IDs
    subject_pos = get_position(999)          # subject
    follower_main_pos = get_position(997)    # back part of follower (main)
    follower_front_pos = get_position(994)   # front part of follower

    # Draw robot orientation
    if follower_main_pos is not None and follower_front_pos is not None:
        cv2.arrowedLine(frame, tuple(follower_main_pos), tuple(follower_front_pos), (0, 0, 255), 2)
        cv2.putText(frame, "Front", tuple(follower_front_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    if subject_pos is not None and follower_main_pos is not None and follower_front_pos is not None:
        # Get all current markers for obstacle detection
        current_markers = {}
        for marker_id, marker_data in last_seen.items():
            if current_time - marker_data['time'] < timeout:
                current_markers[marker_id] = marker_data
        
        # Find obstacles blocking the path
        obstacles = find_obstacles_in_path(follower_main_pos, subject_pos, current_markers)
        
        # Calculate waypoint if obstacles are blocking
        waypoint = None
        if obstacles:
            # Use closest obstacle for waypoint calculation
            main_obstacle = obstacles[0]
            waypoint = calculate_waypoint(follower_main_pos, subject_pos, main_obstacle['pos'])
            avoiding_obstacle = True
            current_target = waypoint
        else:
            # Check if we were avoiding obstacle and have reached waypoint
            if avoiding_obstacle and current_target is not None:
                distance_to_waypoint = calculate_distance(follower_main_pos, current_target)
                if distance_to_waypoint < waypoint_reached_threshold:
                    # Waypoint reached, now go to subject
                    avoiding_obstacle = False
                    current_target = subject_pos
            else:
                avoiding_obstacle = False
                current_target = subject_pos

        # Draw path planning visualization
        draw_path_plan(frame, follower_main_pos, subject_pos, obstacles, waypoint, avoiding_obstacle)

        # Navigation logic
        if current_target is None:
            current_target = subject_pos

        # Calculate orientation vector and vector to current target
        follower_vector = np.array(follower_front_pos) - np.array(follower_main_pos)
        target_vector = np.array(current_target) - np.array(follower_main_pos)

        angle_diff = angle_between_vectors(follower_vector, target_vector)
        distance_cm = calculate_distance(current_target, follower_main_pos)
        speed = determine_speed(distance_cm)

        angle_threshold = 10  # degrees tolerance for "facing"

        # Display navigation info
        nav_info = "AVOIDING OBSTACLE" if avoiding_obstacle else "DIRECT PATH"
        cv2.putText(frame, nav_info, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                   (255, 0, 255) if avoiding_obstacle else (0, 255, 0), 2)
        
        cv2.putText(frame, f"Distance: {distance_cm:.1f}px", (30, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Angle: {angle_diff:.1f}°", (30, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if abs(angle_diff) <= angle_threshold:
            # Facing target, move forward
            send_command('F')
            time.sleep(0.05)
            send_command(speed)
        else:
            # Rotate to face the target
            if angle_diff > 0:
                send_command('R')
            else:
                send_command('L')
            time.sleep(0.05)
            send_command('0')  # Stop forward movement while rotating

    else:
        cv2.putText(frame, "Subject or Robot Markers Lost", (30, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        send_command('D')  # Stop All if subject or markers lost

    # Display obstacle status
    obstacle_count = len([marker_id for marker_id in last_seen.keys() 
                         if marker_id not in {997, 994, 999} and 
                         current_time - last_seen[marker_id]['time'] < timeout])
    
    if obstacle_count > 0:
        cv2.putText(frame, f"OBSTACLES DETECTED: {obstacle_count}", (30, frame.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # List detected obstacle IDs
        obstacle_ids = [str(marker_id) for marker_id in last_seen.keys() 
                       if marker_id not in {997, 994, 999} and 
                       current_time - last_seen[marker_id]['time'] < timeout]
        cv2.putText(frame, f"IDs: {', '.join(obstacle_ids)}", (30, frame.shape[0] - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        cv2.putText(frame, "No Obstacles", (30, frame.shape[0] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("ArUco Follower with Obstacle Avoidance", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap.release()
cv2.destroyAllWindows()
if esp32 and esp32.is_open:
    esp32.close()