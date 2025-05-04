from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import cv2
import numpy as np
import argparse
from pymavlink import mavutil

# ---------- Drone Setup ----------
# Handle command-line argument for drone connection
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='udp:127.0.0.1:14551')
args = parser.parse_args()

# Connect to the Vehicle
print(f'Connecting to vehicle on: {args.connect}')
vehicle = connect(args.connect, baud=921600, wait_ready=True)

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

# ---------- Camera Setup ----------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    vehicle.close()
    exit()

# ---------- Takeoff ----------
arm_and_takeoff(3)
time.sleep(2)

# ---------- Control Loop ----------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2
FRAME_CENTER_Y = FRAME_HEIGHT // 2
TOLERANCE = 30  # Pixels

def send_ned_velocity(vx, vy, vz):
    """
    vx: forward, vy: right, vz: down
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

print("Tracking circle. Show a circle to control the drone...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        # Detect circles
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100,
                                   param1=50, param2=30, minRadius=10, maxRadius=100)

        vx, vy = 0, 0  # Default no movement

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0, :1]:  # only one circle (the biggest)
                x, y, r = c
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

                dx = x - FRAME_CENTER_X
                dy = y - FRAME_CENTER_Y


                if abs(dx) > TOLERANCE:
                    vy = -0.5 if dx > 0 else 0.5  # move right or left

                if abs(dy) > TOLERANCE:
                    vx = -0.5 if dy > 0 else 0.5  # move forward or backward
                
                # Adjust scale for visible motion
                scale = 0.03  # 10x the previous speed
                vx = -dy * scale
                vy = dx * scale

                # Clamp speed for safety (optional)
                max_speed = 1.5  # m/s
                vx = max(min(vx, max_speed), -max_speed)
                vy = max(min(vy, max_speed), -max_speed)

                print(f"Moving drone: vx={vx:.2f}, vy={vy:.2f}")

        else:
            print("No circle detected")
            vx = vy = 0

        # Apply velocity for next 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10:
            send_ned_velocity(vx, vy, 0)

            # Show the SAME frame where circle was drawn
            cv2.imshow("Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

            time.sleep(0.5)

        # # Send movement command
        # send_ned_velocity(vx, vy, 0)

        # cv2.imshow("Circle Tracker", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     print("Landing...")
        #     break

except KeyboardInterrupt:
    print("Interrupted. Landing...")

# ---------- Cleanup ----------
cap.release()
cv2.destroyAllWindows()
vehicle.mode = VehicleMode("LAND")
vehicle.close()
