import cv2
import numpy as np
import time
import serial
import math
import sys
import threading
from collections import deque
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List
import queue

# --- System States ---
class SystemState(Enum):
    IDLE = "IDLE"
    FOLLOWING = "FOLLOWING"
    AVOIDING = "AVOIDING"
    EMERGENCY_STOP = "EMERGENCY"
    RECOVERING = "RECOVERING"
    PROXIMITY_TRIGGERED = "PROXIMITY_TRIGGERED"  # New state for subject proximity

# --- Configuration ---
@dataclass
class SystemConfig:
    # ArUco markers
    ARUCO_SUBJECT: int = 999
    ARUCO_CART_BACK: int = 997
    ARUCO_CART_FRONT: int = 994
    
    # Ultrasonic thresholds (cm)
    CRITICAL_DISTANCE: float = 15.0  # Emergency stop
    WARNING_DISTANCE: float = 40.0   # Start avoidance
    SAFE_DISTANCE: float = 60.0      # Safe to proceed
    MAX_SENSOR_RANGE: float = 400.0  # Maximum reliable range
    
    # Navigation parameters
    ANGLE_TOLERANCE: float = 15.0    # degrees
    TRACKING_TIMEOUT: float = 1.0     # seconds
    
    # Subject proximity settings
    SUBJECT_PROXIMITY_RADIUS: float = 100.0  # pixels - circular radius around subject
    PROXIMITY_D_DURATION: float = 2.0        # seconds to send 'D' command
    PROXIMITY_S_INTERVAL: float = 0.5        # seconds between 'S' commands
    
    # Speed settings (0-9 scale)
    MAX_SPEED: int = 9
    NORMAL_SPEED: int = 6
    SLOW_SPEED: int = 3
    TURN_SPEED: int = 5
    
    # Sensor validation
    SENSOR_HISTORY_SIZE: int = 5
    OUTLIER_THRESHOLD: float = 2.0   # Standard deviations
    
    # Communication settings
    COMMAND_TIMEOUT: float = 0.1
    MAX_COMMAND_QUEUE: int = 5
    FRAME_SKIP: int = 1

config = SystemConfig()

# --- Non-blocking ESP32 Controller ---
class ESP32Controller:
    def __init__(self, port):
        self.port = port
        self.baud = 115200
        self.ser = None
        self.running = False
        self.ultrasonic_data = [config.MAX_SENSOR_RANGE, config.MAX_SENSOR_RANGE, config.MAX_SENSOR_RANGE]
        self.sensor_history = [deque(maxlen=config.SENSOR_HISTORY_SIZE) for _ in range(3)]
        self.last_command = ('S', 0)
        self.command_queue = queue.Queue(maxsize=config.MAX_COMMAND_QUEUE)
        self.data_lock = threading.Lock()
        self.last_command_time = 0
        self.connection_stable = False
        
    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baud, 
                timeout=0.05,
                write_timeout=0.05
            )
            self.running = True
            
            threading.Thread(target=self._read_serial, daemon=True).start()
            threading.Thread(target=self._write_serial, daemon=True).start()
            
            time.sleep(1)
            self.connection_stable = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def _read_serial(self):
        """Non-blocking serial reader"""
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith("US:"):
                            self._process_ultrasonic(line)
                
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"Serial read error: {e}")
                time.sleep(0.1)
    
    def _write_serial(self):
        """Non-blocking serial writer"""
        while self.running:
            try:
                cmd = self.command_queue.get(timeout=0.1)
                
                if self.ser and self.ser.is_open:
                    self.ser.write((cmd + '\n').encode())
                    self.ser.flush()
                    
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    print(f"Serial write error: {e}")
                time.sleep(0.1)
    
    def _process_ultrasonic(self, line):
        try:
            distances = [float(x) for x in line.split(':')[1].split(',')]
            if len(distances) == 3:
                with self.data_lock:
                    for i, dist in enumerate(distances):
                        if dist < 0 or dist > config.MAX_SENSOR_RANGE:
                            if len(self.sensor_history[i]) > 0:
                                distances[i] = self.sensor_history[i][-1]
                            else:
                                distances[i] = config.MAX_SENSOR_RANGE
                        
                        self.sensor_history[i].append(distances[i])
                    
                    self.ultrasonic_data = distances
        except (ValueError, IndexError):
            pass
    
    def get_filtered_distances(self):
        """Thread-safe filtered distances"""
        with self.data_lock:
            filtered = []
            for i in range(3):
                if len(self.sensor_history[i]) < 2:
                    filtered.append(self.ultrasonic_data[i])
                else:
                    values = list(self.sensor_history[i])[-3:]
                    median_val = np.median(values)
                    filtered.append(median_val)
            return filtered
    
    def send_movement(self, direction: str, speed: int):
        """Non-blocking movement command"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < 0.05:
            return True
        
        # Duplicate command check
        if (direction, speed) == self.last_command:
            return True
        
        try:
            if not self.command_queue.full():
                self.command_queue.put_nowait(direction)
                if not self.command_queue.full():
                    self.command_queue.put_nowait(str(speed))
                    self.last_command = (direction, speed)
                    self.last_command_time = current_time
                    return True
            else:
                self._clear_command_queue()
                self.command_queue.put_nowait(direction)
                self.command_queue.put_nowait(str(speed))
                return True
        except queue.Full:
            return False
    
    def _clear_command_queue(self):
        """Clear command queue"""
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break
    
    def emergency_stop(self):
        """Immediate stop with queue priority"""
        self._clear_command_queue()
        try:
            self.command_queue.put_nowait('S')
            self.command_queue.put_nowait('0')
            self.last_command = ('S', 0)
        except queue.Full:
            pass

# --- Optimized Vision System ---
class ArucoTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 15
        params.adaptiveThreshWinSizeStep = 4
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
        
        self.last_seen = {}
        self.position_history = {}
        self.frame_count = 0
        
    def get_marker_positions(self):
        self.frame_count += 1
        
        # Frame skip logic
        if self.frame_count % config.FRAME_SKIP != 0:
            ret, frame = self.cap.read()
            if ret:
                return frame, *self._get_last_positions()
            return None, None, None, None
        
        ret, frame = self.cap.read()
        if not ret:
            return None, None, None, None
        
        # Frame processing
        if frame.shape[0] > 480:
            frame = cv2.resize(frame, (640, 480))
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        current_time = time.time()
        
        if ids is not None:
            ids = ids.flatten()
            
            for i, marker_id in enumerate(ids):
                center = np.mean(corners[i][0], axis=0).astype(int)
                marker_id = int(marker_id)
                
                # Position smoothing
                if marker_id not in self.position_history:
                    self.position_history[marker_id] = deque(maxlen=3)
                
                self.position_history[marker_id].append(center)
                
                if len(self.position_history[marker_id]) > 1:
                    smoothed = np.mean(list(self.position_history[marker_id]), axis=0).astype(int)
                else:
                    smoothed = center
                    
                self.last_seen[marker_id] = {'pos': smoothed, 'time': current_time}
        
        # Get target markers
        subject = self._get_valid_position(config.ARUCO_SUBJECT)
        cart_back = self._get_valid_position(config.ARUCO_CART_BACK)
        cart_front = self._get_valid_position(config.ARUCO_CART_FRONT)
        
        return frame, subject, cart_back, cart_front
    
    def _get_last_positions(self):
        """Get last known positions without processing new frame"""
        subject = self._get_valid_position(config.ARUCO_SUBJECT)
        cart_back = self._get_valid_position(config.ARUCO_CART_BACK)
        cart_front = self._get_valid_position(config.ARUCO_CART_FRONT)
        return subject, cart_back, cart_front
    
    def _get_valid_position(self, marker_id):
        if marker_id in self.last_seen:
            age = time.time() - self.last_seen[marker_id]['time']
            if age < config.TRACKING_TIMEOUT:
                return self.last_seen[marker_id]['pos']
        return None

# --- Navigation Controller with Proximity Detection ---
class NavigationController:
    def __init__(self):
        self.state = SystemState.IDLE
        self.avoidance_direction = None
        self.avoidance_start_time = None
        self.avoidance_phase = "turn"
        self.stuck_counter = 0
        self.last_distances = [config.MAX_SENSOR_RANGE] * 3
        self.last_subject_angle = 0
        
        # Proximity detection state
        self.proximity_triggered_time = None
        self.d_command_sent = False
        self.last_s_command_time = 0
        
    def check_proximity(self, cart_back, subject_pos):
        """Check if cart is within proximity radius of subject"""
        if cart_back is None or subject_pos is None:
            return False
        
        distance = np.linalg.norm(np.array(cart_back) - np.array(subject_pos))
        return distance <= config.SUBJECT_PROXIMITY_RADIUS
    
    def calculate_movement(self, cart_back, cart_front, subject_pos, ultrasonic_data):
        """Main navigation decision logic with proximity detection"""
        
        # Check proximity first
        if self.check_proximity(cart_back, subject_pos):
            if self.state != SystemState.PROXIMITY_TRIGGERED:
                # First time entering proximity
                self.state = SystemState.PROXIMITY_TRIGGERED
                self.proximity_triggered_time = time.time()
                self.d_command_sent = False
                self.last_s_command_time = 0
                print(f"🎯 PROXIMITY TRIGGERED - Cart within {config.SUBJECT_PROXIMITY_RADIUS}px of subject")
            
            return self._handle_proximity_mode()
        else:
            # Reset proximity state if we exit the radius
            if self.state == SystemState.PROXIMITY_TRIGGERED:
                print("🎯 EXITED PROXIMITY - Resuming normal operation")
                self.state = SystemState.FOLLOWING
                self.proximity_triggered_time = None
                self.d_command_sent = False
        
        # Normal navigation logic
        cart_forward_vector = np.array(cart_front) - np.array(cart_back)
        subject_vector = np.array(subject_pos) - np.array(cart_back)
        
        angle = self._angle_between_vectors_fixed(cart_forward_vector, subject_vector)
        self.last_subject_angle = angle
        
        distance_to_subject = np.linalg.norm(subject_vector)
        
        left_dist, center_dist, right_dist = ultrasonic_data
        min_distance = min(ultrasonic_data)
        
        self.last_distances = ultrasonic_data.copy()
        
        # STATE MACHINE DECISION LOGIC
        if min_distance < config.CRITICAL_DISTANCE:
            return self._handle_emergency_stop(ultrasonic_data, angle)
            
        elif self.state == SystemState.AVOIDING:
            return self._handle_intelligent_avoidance(ultrasonic_data, angle, distance_to_subject)
            
        elif min_distance < config.WARNING_DISTANCE:
            return self._initiate_intelligent_avoidance(ultrasonic_data, angle)
            
        else:
            # Normal following behavior
            self.state = SystemState.FOLLOWING
            return self._corrected_following(angle, distance_to_subject, ultrasonic_data)
    
    def _handle_proximity_mode(self):
        """Handle proximity triggered mode - send D then repeated S commands"""
        current_time = time.time()
        elapsed_time = current_time - self.proximity_triggered_time
        
        # Phase 1: Send D command for specified duration
        if not self.d_command_sent and elapsed_time < config.PROXIMITY_D_DURATION:
            print(f"🎯 PROXIMITY: Sending D command ({elapsed_time:.1f}s/{config.PROXIMITY_D_DURATION}s)")
            return ('D', config.NORMAL_SPEED)
        
        # Mark D command phase as complete
        if not self.d_command_sent and elapsed_time >= config.PROXIMITY_D_DURATION:
            self.d_command_sent = True
            self.last_s_command_time = current_time
            print("🎯 PROXIMITY: D command phase complete, starting S commands")
        
        # Phase 2: Send repeated S commands
        if self.d_command_sent:
            if current_time - self.last_s_command_time >= config.PROXIMITY_S_INTERVAL:
                self.last_s_command_time = current_time
                print(f"🎯 PROXIMITY: Sending S command (interval: {config.PROXIMITY_S_INTERVAL}s)")
                return ('S', 0)
            else:
                # Continue with last S command to maintain stop
                return ('S', 0)
        
        # Fallback
        return ('S', 0)
    
    def _angle_between_vectors_fixed(self, v1, v2):
        """Calculate signed angle between vectors with correct OpenCV coordinate system"""
        # Normalize vectors
        v1_magnitude = np.linalg.norm(v1) + 1e-8
        v2_magnitude = np.linalg.norm(v2) + 1e-8
        v1_norm = v1 / v1_magnitude
        v2_norm = v2 / v2_magnitude
        
        # Calculate angle using dot product
        dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle = np.degrees(np.arccos(dot_product))
        
        # Correct cross product for OpenCV coordinate system (Y-axis inverted)
        cross_product = v1_norm[0] * v2_norm[1] - v1_norm[1] * v2_norm[0]
        
        # Invert the sign because OpenCV Y-axis is flipped
        signed_angle = -angle if cross_product >= 0 else angle
        
        return signed_angle
    
    def _corrected_following(self, angle, distance, ultrasonic_data):
        """Corrected following behavior with proper angle interpretation"""
        speed = self._calculate_dynamic_speed(distance, ultrasonic_data)
        
        if abs(angle) <= config.ANGLE_TOLERANCE:
            # Subject is straight ahead - move forward
            print("→ Moving FORWARD (subject ahead)")
            command = ('F', speed)
        elif angle > config.ANGLE_TOLERANCE:
            # Subject is to the LEFT - turn LEFT  
            print(f"→ Turning LEFT (subject {angle:.1f}° left)")
            command = ('L', config.TURN_SPEED)
        else:
            # Subject is to the RIGHT - turn RIGHT
            print(f"→ Turning RIGHT (subject {abs(angle):.1f}° right)")
            command = ('R', config.TURN_SPEED)
        
        return command
    
    def _handle_emergency_stop(self, distances, target_angle):
        """Emergency stop logic"""
        self.state = SystemState.EMERGENCY_STOP
        
        left, center, right = distances
        
        # Find the direction with most space
        escape_options = {
            'left': left,
            'right': right,
            'center': center
        }
        
        best_direction = max(escape_options, key=escape_options.get)
        best_distance = escape_options[best_direction]
        
        if best_direction == 'left' and left > config.SAFE_DISTANCE:
            command = ('L', config.TURN_SPEED)
        elif best_direction == 'right' and right > config.SAFE_DISTANCE:
            command = ('R', config.TURN_SPEED)
        elif center > config.CRITICAL_DISTANCE + 5:
            command = ('B', config.SLOW_SPEED)
        else:
            # Target-based escape
            if abs(target_angle) > 90:  # Subject is behind us
                direction = 'L' if target_angle > 0 else 'R'
                command = (direction, config.TURN_SPEED)
            else:  # Subject is in front, turn towards the side with more space
                direction = 'L' if left > right else 'R'
                command = (direction, config.TURN_SPEED)
        
        if command is None:
            command = ('S', 0)
        
        return command
    
    def _initiate_intelligent_avoidance(self, distances, target_angle):
        """Smart avoidance initiation"""
        self.state = SystemState.AVOIDING
        self.avoidance_start_time = time.time()
        self.avoidance_phase = "turn"
        
        left, center, right = distances
        
        # Decision logic for avoidance direction
        if center < config.WARNING_DISTANCE and max(left, right) > config.SAFE_DISTANCE:
            space_difference = abs(left - right)
            
            if space_difference > 20:  # Significant difference in space
                self.avoidance_direction = 'L' if left > right else 'R'
            else:  # Similar space on both sides, consider target angle
                if abs(target_angle) < 30:  # Target is roughly ahead
                    self.avoidance_direction = 'L' if target_angle > 0 else 'R'
                else:
                    self.avoidance_direction = 'L' if left > right else 'R'
        
        elif left < config.WARNING_DISTANCE and right > config.SAFE_DISTANCE:
            self.avoidance_direction = 'R'
        
        elif right < config.WARNING_DISTANCE and left > config.SAFE_DISTANCE:
            self.avoidance_direction = 'L'
        
        else:
            # Both sides problematic, choose based on target
            if target_angle > 0:
                self.avoidance_direction = 'L'
            else:
                self.avoidance_direction = 'R'
        
        return (self.avoidance_direction, config.TURN_SPEED)
    
    def _handle_intelligent_avoidance(self, distances, target_angle, target_distance):
        """Multi-phase avoidance logic"""
        left, center, right = distances
        elapsed_time = time.time() - self.avoidance_start_time
        
        # Anti-stuck mechanism
        if elapsed_time > 8.0:
            self.stuck_counter += 1
            
            if self.stuck_counter > 2:
                # Try backing up if really stuck
                self.avoidance_direction = 'B'
                self.stuck_counter = 0
            else:
                # Switch avoidance direction
                self.avoidance_direction = 'R' if self.avoidance_direction == 'L' else 'L'
            
            self.avoidance_start_time = time.time()
            self.avoidance_phase = "turn"
            return (self.avoidance_direction, config.TURN_SPEED)
        
        # PHASE 1: Turn away from obstacle
        if self.avoidance_phase == "turn":
            # Check if we've turned enough and path is clear
            if self.avoidance_direction == 'L':
                path_clear = left > config.SAFE_DISTANCE and center > config.WARNING_DISTANCE
            else:
                path_clear = right > config.SAFE_DISTANCE and center > config.WARNING_DISTANCE
            
            if path_clear and elapsed_time > 1.0:  # Turned for at least 1 second
                self.avoidance_phase = "forward"
                return ('F', config.SLOW_SPEED)
            else:
                return (self.avoidance_direction, config.TURN_SPEED)
        
        # PHASE 2: Move forward around obstacle
        elif self.avoidance_phase == "forward":
            # Keep moving forward while path is clear
            path_clear = center > config.WARNING_DISTANCE and min(distances) > config.CRITICAL_DISTANCE
            
            if path_clear:
                # Check if we can start returning to target
                if elapsed_time > 3.0:  # Moved forward for sufficient time
                    self.avoidance_phase = "return"
                
                return ('F', config.NORMAL_SPEED)
            else:
                # Path blocked again, continue turning
                return (self.avoidance_direction, config.TURN_SPEED)
        
        # PHASE 3: Return towards target
        elif self.avoidance_phase == "return":
            # Turn back towards the subject
            opposite_direction = 'R' if self.avoidance_direction == 'L' else 'L'
            
            # If we can see a clear path towards target, end avoidance
            clear_path = (center > config.SAFE_DISTANCE and 
                         min(distances) > config.WARNING_DISTANCE)
            
            if clear_path:
                self.state = SystemState.FOLLOWING
                self.stuck_counter = 0  # Reset stuck counter
                return self._corrected_following(target_angle, target_distance, distances)
            else:
                return (opposite_direction, config.TURN_SPEED)
        
        # Default fallback
        return (self.avoidance_direction, config.TURN_SPEED)
    
    def _calculate_dynamic_speed(self, distance_to_subject, ultrasonic_data):
        """Dynamic speed calculation"""
        min_obstacle = min(ultrasonic_data)
        
        # Base speed on obstacle proximity
        if min_obstacle < config.WARNING_DISTANCE:
            base_speed = config.SLOW_SPEED
        elif min_obstacle < config.SAFE_DISTANCE:
            base_speed = config.NORMAL_SPEED
        else:
            base_speed = config.MAX_SPEED
        
        # Adjust based on distance to subject
        if distance_to_subject < 100:  # Very close
            base_speed = min(base_speed, config.SLOW_SPEED)
        elif distance_to_subject > 300:  # Far away
            base_speed = min(base_speed + 1, config.MAX_SPEED)
        
        return base_speed

# --- Enhanced Visualization ---
def create_hud_overlay(frame, state, ultrasonic_data, cmd, speed, fps=0, connection_status=True, proximity_info=None):
    """Create modern HUD overlay with proximity visualization"""
    h, w = frame.shape[:2]
    
    overlay = frame.copy()
    
    # Modern state indicator
    state_colors = {
        SystemState.IDLE: ((100, 100, 100), (150, 150, 150)),
        SystemState.FOLLOWING: ((0, 200, 0), (0, 255, 0)),
        SystemState.AVOIDING: ((0, 100, 255), (0, 150, 255)),
        SystemState.EMERGENCY_STOP: ((0, 0, 200), (0, 0, 255)),
        SystemState.RECOVERING: ((150, 150, 0), (255, 255, 0)),
        SystemState.PROXIMITY_TRIGGERED: ((255, 0, 255), (255, 100, 255))  # New state color
    }
    
    color1, color2 = state_colors[state]
    
    # State panel
    cv2.rectangle(overlay, (20, 20), (280, 80), color2, -1)
    cv2.rectangle(overlay, (20, 20), (280, 80), color1, 3)
    cv2.putText(overlay, f"STATE: {state.value}", (30, 50), 
                cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
    
    # Connection indicator
    conn_color = (0, 255, 0) if connection_status else (0, 0, 255)
    cv2.circle(overlay, (250, 35), 8, conn_color, -1)
    cv2.putText(overlay, "CONN", (200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # Command display
    cmd_panel_y = 100
    cv2.rectangle(overlay, (20, cmd_panel_y), (280, cmd_panel_y + 40), (50, 50, 50), -1)
    cv2.rectangle(overlay, (20, cmd_panel_y), (280, cmd_panel_y + 40), (100, 100, 100), 2)
    
    arrow_center = (60, cmd_panel_y + 20)
    draw_direction_arrow(overlay, arrow_center, cmd)
    
    cv2.putText(overlay, f"CMD: {cmd} | SPD: {speed}", (90, cmd_panel_y + 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Proximity info display
    if proximity_info:
        prox_y = cmd_panel_y + 50
        cv2.rectangle(overlay, (20, prox_y), (380, prox_y + 30), (30, 30, 50), -1)
        cv2.putText(overlay, f"Proximity: {proximity_info['distance']:.0f}px | Radius: {config.SUBJECT_PROXIMITY_RADIUS:.0f}px", 
                    (25, prox_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    
    # Sensor display
    sensor_y_start = 200
    sensor_labels = ['LEFT', 'CENTER', 'RIGHT']
    sensor_positions = [(30, sensor_y_start), (30, sensor_y_start + 40), (30, sensor_y_start + 80)]
    
    for i, (label, pos, dist) in enumerate(zip(sensor_labels, sensor_positions, ultrasonic_data)):
        if dist < config.CRITICAL_DISTANCE:
            color = (0, 0, 255)
            status = "CRITICAL"
        elif dist < config.WARNING_DISTANCE:
            color = (0, 165, 255)
            status = "WARNING"
        elif dist < config.SAFE_DISTANCE:
            color = (0, 255, 255)
            status = "CAUTION"
        else:
            color = (0, 255, 0)
            status = "SAFE"
        
        bar_x, bar_y = pos[0] + 80, pos[1] - 8
        cv2.rectangle(overlay, (bar_x, bar_y), (bar_x + 150, bar_y + 20), (30, 30, 30), -1)
        
        fill_width = int(150 * min(dist / config.MAX_SENSOR_RANGE, 1.0))
        cv2.rectangle(overlay, (bar_x, bar_y), (bar_x + fill_width, bar_y + 20), color, -1)
        
        text = f"{label}: {dist:.0f}cm"
        cv2.putText(overlay, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(overlay, status, (bar_x + 10, bar_y + 14), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # FPS counter
    if fps > 0:
        cv2.putText(overlay, f"FPS: {fps:.1f}", (w - 120, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # Radar display
    radar_x = w - 180
    draw_modern_radar(overlay, ultrasonic_data, (radar_x, h - 180))
    
    # Blend overlay
    alpha = 0.8
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
    
    return frame

def draw_direction_arrow(frame, center, direction):
    """Draw directional arrow"""
    cx, cy = center
    size = 15
    
    if direction == 'F':
        points = np.array([[cx, cy-size], [cx-size//2, cy+size//2], [cx+size//2, cy+size//2]], np.int32)
        cv2.fillPoly(frame, [points], (0, 255, 0))  # Green for forward
    elif direction == 'B':
        points = np.array([[cx, cy+size], [cx-size//2, cy-size//2], [cx+size//2, cy-size//2]], np.int32)
        cv2.fillPoly(frame, [points], (255, 0, 0))  # Blue for backward
    elif direction == 'L':
        points = np.array([[cx-size, cy], [cx+size//2, cy-size//2], [cx+size//2, cy+size//2]], np.int32)
        cv2.fillPoly(frame, [points], (0, 255, 255))  # Yellow for left
    elif direction == 'R':
        points = np.array([[cx+size, cy], [cx-size//2, cy-size//2], [cx-size//2, cy+size//2]], np.int32)
        cv2.fillPoly(frame, [points], (255, 0, 255))  # Magenta for right
    elif direction == 'D':
        # Special indicator for D command
        cv2.circle(frame, (cx, cy), size, (255, 0, 255), -1)
        cv2.putText(frame, "D", (cx-4, cy+4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    else:  # Stop
        cv2.rectangle(frame, (cx-size//2, cy-size//2), (cx+size//2, cy+size//2), (0, 0, 255), -1)

def draw_modern_radar(frame, distances, center):
    """Draw modern radar display"""
    cx, cy = center
    max_radius = 80
    
    cv2.circle(frame, (cx, cy), max_radius, (40, 40, 40), -1)
    cv2.circle(frame, (cx, cy), max_radius, (100, 100, 100), 2)
    
    for i in range(1, 4):
        radius = int(max_radius * i / 3)
        cv2.circle(frame, (cx, cy), radius, (60, 60, 60), 1)
    
    angles = [-45, 0, 45]
    colors = [(255, 100, 100), (100, 255, 100), (100, 100, 255)]
    
    for i, (angle, dist, color) in enumerate(zip(angles, distances, colors)):
        # Normalize distance
        normalized = min(dist / config.MAX_SENSOR_RANGE, 1.0)
        beam_length = int(max_radius * normalized)
        
        # Calculate beam end
        rad = math.radians(angle - 90)
        end_x = int(cx + beam_length * math.cos(rad))
        end_y = int(cy + beam_length * math.sin(rad))
        
        # Draw beam
        cv2.line(frame, (cx, cy), (end_x, end_y), color, 3)
        
        # Draw obstacle indicator
        if dist < config.WARNING_DISTANCE:
            cv2.circle(frame, (end_x, end_y), 5, (0, 0, 255), -1)
    
    # Labels
    cv2.putText(frame, "RADAR", (cx - 25, cy + max_radius + 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

def draw_proximity_radius(frame, subject_pos, cart_back, is_triggered=False):
    """Draw proximity detection radius around subject marker"""
    if subject_pos is None:
        return
    
    center = tuple(subject_pos.astype(int))
    radius = int(config.SUBJECT_PROXIMITY_RADIUS)
    
    # Color changes based on proximity state
    if is_triggered:
        circle_color = (255, 0, 255)  # Magenta when triggered
        text_color = (255, 255, 255)
        thickness = 3
    else:
        circle_color = (0, 255, 255)  # Yellow when not triggered
        text_color = (0, 255, 255)
        thickness = 2
    
    # Draw proximity circle
    cv2.circle(frame, center, radius, circle_color, thickness)
    
    # Draw subject marker
    cv2.circle(frame, center, 8, (0, 255, 0), -1)
    cv2.putText(frame, "SUBJECT", (center[0] + 15, center[1] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Show proximity radius info
    cv2.putText(frame, f"Proximity: {radius}px", (center[0] - 40, center[1] + radius + 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)
    
    # If cart is detected, show distance to subject
    if cart_back is not None:
        distance = np.linalg.norm(np.array(cart_back) - np.array(subject_pos))
        status_text = "TRIGGERED!" if is_triggered else f"Dist: {distance:.0f}px"
        status_color = (255, 0, 255) if is_triggered else (255, 255, 255)
        
        cv2.putText(frame, status_text, (center[0] - 30, center[1] + radius + 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)

# --- Main Program ---
def main():
    # Parse command line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else "COM1"
    
    print("=" * 60)
    print("🚀 CART FOLLOWING SYSTEM WITH PROXIMITY DETECTION")
    print("=" * 60)
    # print("🎯 NEW FEATURES:")
    # print(f"   ✅ Subject proximity radius: {config.SUBJECT_PROXIMITY_RADIUS}px")

    
    # Initialize systems
    esp32 = ESP32Controller(port)
    if not esp32.connect():
        print("❌ Failed to connect to ESP32. Exiting...")
        return
    
    tracker = ArucoTracker()
    navigator = NavigationController()
    
    print("\n✅ System initialized successfully!")
    print("🎮 Controls:")
    print("   'q' - Quit")
    print("   'p' - Pause/Resume")
    print("   'r' - Reset navigation state")
    print("🎯 PROXIMITY DETECTION:")
    print(f"   When cart enters {config.SUBJECT_PROXIMITY_RADIUS}px radius of subject:")
    print(f"   1. Send 'D' command for {config.PROXIMITY_D_DURATION}s")
    print(f"   2. Then send 'S' commands every {config.PROXIMITY_S_INTERVAL}s")
    print("-" * 60)
    
    paused = False
    fps_counter = deque(maxlen=30)
    last_fps_time = time.time()
    
    try:
        while True:
            frame_start = time.time()
            
            # Get sensor data
            frame, subject, cart_back, cart_front = tracker.get_marker_positions()
            
            if frame is None:
                break
            
            ultrasonic = esp32.get_filtered_distances()
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                paused = not paused
                if paused:
                    esp32.emergency_stop()
                    navigator.state = SystemState.IDLE
                print(f"{'⏸️  PAUSED' if paused else '▶️  RESUMED'}")
            elif key == ord('r'):
                navigator.state = SystemState.IDLE
                navigator.stuck_counter = 0
                navigator.avoidance_direction = None
                # Reset proximity state
                navigator.proximity_triggered_time = None
                navigator.d_command_sent = False
                navigator.last_s_command_time = 0
                print("🔄 Navigation state reset")
            
            # Calculate FPS
            current_time = time.time()
            fps_counter.append(1.0 / (current_time - last_fps_time))
            last_fps_time = current_time
            avg_fps = np.mean(fps_counter) if fps_counter else 0
            
            # Proximity info for display
            proximity_info = None
            if cart_back is not None and subject is not None:
                distance = np.linalg.norm(np.array(cart_back) - np.array(subject))
                proximity_info = {'distance': distance}
            
            # Main logic
            if paused:
                create_hud_overlay(frame, SystemState.IDLE, ultrasonic, 'S', 0, avg_fps, 
                                 esp32.connection_stable, proximity_info)
                cv2.putText(frame, "⏸️ SYSTEM PAUSED", (frame.shape[1]//2 - 100, frame.shape[0]//2),
                           cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 3)
            
            elif all(pos is not None for pos in [subject, cart_back, cart_front]):
                # Tracking mode with proximity detection
                
                # Calculate movement command
                cmd, speed = navigator.calculate_movement(
                    cart_back, cart_front, subject, ultrasonic
                )
                
                # Send command to ESP32
                esp32.send_movement(cmd, speed)
                
                # Draw proximity radius around subject
                is_proximity_triggered = navigator.state == SystemState.PROXIMITY_TRIGGERED
                draw_proximity_radius(frame, subject, cart_back, is_proximity_triggered)
                
                # Draw cart markers
                cv2.circle(frame, tuple(cart_back), 8, (0, 0, 255), -1)  # Red for cart back
                cv2.circle(frame, tuple(cart_front), 8, (255, 0, 0), -1)  # Blue for cart front
                
                # Draw cart's forward direction
                cart_forward_vector = np.array(cart_front) - np.array(cart_back)
                cart_center = np.array(cart_back) + cart_forward_vector * 0.5
                cart_tip = cart_center + cart_forward_vector * 0.3
                cv2.arrowedLine(frame, tuple(cart_center.astype(int)), tuple(cart_tip.astype(int)), 
                               (0, 0, 255), 4, tipLength=0.3)
                
                # Draw direction to subject
                subject_vector = np.array(subject) - np.array(cart_back)
                subject_tip = np.array(cart_back) + subject_vector * 0.6
                cv2.arrowedLine(frame, tuple(cart_back), tuple(subject_tip.astype(int)), 
                               (0, 255, 0), 3, tipLength=0.3)
                
                # Labels
                cv2.putText(frame, "CART FRONT", tuple(cart_front + np.array([10, -10])), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame, "CART BACK", tuple(cart_back - np.array([20, 20])), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Show proximity status
                if is_proximity_triggered:
                    elapsed_time = time.time() - navigator.proximity_triggered_time
                    if not navigator.d_command_sent:
                        phase_text = f"🎯 PROXIMITY: D Command ({elapsed_time:.1f}s/{config.PROXIMITY_D_DURATION}s)"
                    else:
                        phase_text = "🎯 PROXIMITY: S Commands Active"
                    
                    cv2.putText(frame, phase_text, (50, frame.shape[0] - 50),
                               cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 0, 255), 2)
                
                create_hud_overlay(frame, navigator.state, ultrasonic, cmd, speed, 
                                 avg_fps, esp32.connection_stable, proximity_info)
                
            else:
                # Lost tracking mode
                esp32.emergency_stop()
                navigator.state = SystemState.IDLE
                
                # Reset proximity state when tracking is lost
                navigator.proximity_triggered_time = None
                navigator.d_command_sent = False
                navigator.last_s_command_time = 0
                
                create_hud_overlay(frame, navigator.state, ultrasonic, 'S', 0, avg_fps, 
                                 esp32.connection_stable, proximity_info)
                cv2.putText(frame, "🔍 SEARCHING FOR MARKERS...", (50, frame.shape[0]//2),
                           cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 2)
                
                # Show marker status
                marker_status = []
                for marker_id, name in [(config.ARUCO_SUBJECT, "SUBJECT"), 
                                       (config.ARUCO_CART_BACK, "CART_BACK"), 
                                       (config.ARUCO_CART_FRONT, "CART_FRONT")]:
                    if marker_id in tracker.last_seen:
                        age = time.time() - tracker.last_seen[marker_id]['time']
                        if age < config.TRACKING_TIMEOUT:
                            marker_status.append(f"✅ {name}")
                        else:
                            marker_status.append(f"⏰ {name} ({age:.1f}s)")
                    else:
                        marker_status.append(f"❌ {name}")
                
                for i, status in enumerate(marker_status):
                    cv2.putText(frame, status, (50, frame.shape[0]//2 + 50 + i*30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display frame
            window_title = "🎯 Cart Tracker with Proximity Detection"
            cv2.imshow(window_title, frame)
            
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        esp32.emergency_stop()
    
    finally:
        print("🔄 Shutting down systems...")
        esp32.emergency_stop()
        esp32.running = False
        
        time.sleep(0.5)
        
        if esp32.ser and esp32.ser.is_open:
            esp32.ser.close()
        if tracker.cap and tracker.cap.isOpened():
            tracker.cap.release()
        cv2.destroyAllWindows()
        
        print("✅ System shutdown complete.")
        print("🎯 Proximity detection features:")
        print(f"   • Radius: {config.SUBJECT_PROXIMITY_RADIUS}px")
        print(f"   • D command duration: {config.PROXIMITY_D_DURATION}s")
        print(f"   • S command interval: {config.PROXIMITY_S_INTERVAL}s")

if __name__ == "__main__":
    main()