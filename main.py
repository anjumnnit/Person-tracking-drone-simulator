from dronekit import connect, VehicleMode
import time
import cv2
import numpy as np
import argparse
from pymavlink import mavutil
from ultralytics import YOLO  # For YOLOv8 model

# ----------- Drone Connection -----------
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='udp:127.0.0.1:14551')
args = parser.parse_args()

print(f"Connecting to vehicle on: {args.connect}")
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# ----------- YOLOv8 Person Detection Setup -----------
model = YOLO('./train/weights/best.pt')  # Path to your trained model

# ----------- Takeoff Function -----------
def arm_and_takeoff(target_altitude):
    print("Arming motors")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# ----------- MAVLink Velocity Command Function -----------
def send_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ----------- Camera Setup -----------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    vehicle.close()
    exit()

# ----------- Takeoff -----------
arm_and_takeoff(3)
time.sleep(2)

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2
FRAME_CENTER_Y = FRAME_HEIGHT // 2
TOLERANCE = 30  # Pixels tolerance for staying still

print("Tracking person using YOLOv8 model...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame_resized = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        results = model.predict(source=frame_resized, classes=[0], conf=0.5, verbose=False)  # class 0 = person

        # Extract the first person bounding box
        boxes = results[0].boxes
        if boxes and len(boxes) > 0:
            box = boxes[0].xyxy[0].cpu().numpy()  # x1, y1, x2, y2
            x1, y1, x2, y2 = box
            mid_x = int((x1 + x2) / 2)
            mid_y = int((y1 + y2) / 2)

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (mid_x, mid_y), 5, (0, 0, 255), -1)

            dx = mid_x - FRAME_CENTER_X
            dy = mid_y - FRAME_CENTER_Y

            # Convert to velocity commands
            vx, vy = 0, 0
            if abs(dx) > TOLERANCE:
                vy = 0.03 * dx
            if abs(dy) > TOLERANCE:
                vx = -0.03 * dy

            # Clamp max speed
            max_speed = 1.5
            vx = max(min(vx, max_speed), -max_speed)
            vy = max(min(vy, max_speed), -max_speed)

            print(f"Person detected. Moving drone: vx={vx:.2f}, vy={vy:.2f}")
        else:
            print("No person detected. Hovering.")
            vx = vy = 0

        # Apply velocity for next 10 seconds
        start_time = time.time()
        while time.time() - start_time < 5:
            send_ned_velocity(vx, vy, 0)

            cv2.imshow("YOLOv8 Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Landing...")
                break
            time.sleep(0.3)

except KeyboardInterrupt:
    print("Keyboard Interrupt. Landing...")

# ----------- Cleanup -----------
cap.release()
cv2.destroyAllWindows()
vehicle.mode = VehicleMode("LAND")
vehicle.close()
