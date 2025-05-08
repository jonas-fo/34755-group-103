import cv2
import numpy as np
import os
import math
import time

# Load calibration data
calib_data = np.load("/path/to/calibration_data.npz")  # Update path accordingly
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["distortion_coeffs"]

# Save path for processed images
save_path = "./ball_detect_img"
if not os.path.exists(save_path):
    os.makedirs(save_path)

# Ball diameter (43 mm) and hole diameter (52 mm)
object_d = 0.043  # Golf ball diameter in meters (43 mm)

# HSV thresholds for orange (golf ball)
lower_orange = np.array([5, 100, 120])
upper_orange = np.array([15, 255, 255])

# Camera transformation matrix parameters
theta = 11 * np.pi / 180
cxr, cyr, czr = 0.016, 0, 0.188  # camera lean and height on robot

# Helper function: convert pixel coordinates to world coordinates
def pixel_to_world(x, y, camera_matrix, radius, object_d):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    Z = (object_d * fy) / (2 * radius)  # Estimate depth using the ball radius
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy

    # Camera to robot transformation matrix
    T = np.array([
        [np.cos(theta), 0, np.sin(theta), -cxr],
        [0, 1, 0, cyr],
        [-np.sin(theta), 0, np.cos(theta), -czr],
        [0, 0, 0, 1]
    ])

    # Frame change matrix (robot-specific adjustments)
    F = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])

    # Camera coordinates in [m]
    C = np.array([[X], [Y], [Z], [1]])

    # Compute robot coordinates
    R = np.dot(np.dot(T, F), C)
    R = R.flatten()

    return X, Y, Z, R

# Function to simulate the robot's movement
def move_robot_to_target(Z, X, stop_distance=0.30, followup=True):
    forward_speed = 0.15  # m/s
    turn_speed = 0.5  # rad/s

    distance = max(0, math.sqrt(Z**2 + X**2) - stop_distance)
    angle = math.atan2(X, Z)
    turn_time = abs(angle / turn_speed)

    # Simulate turning
    if angle > 0:
        print(f"Simulating turn right for {turn_time:.2f} seconds...")
    elif angle < 0:
        print(f"Simulating turn left for {turn_time:.2f} seconds...")
    else:
        print("No turning needed.")

    # Simulate moving forward
    if followup:
        move_time = distance / forward_speed
        print(f"Simulating move forward for {move_time:.2f} seconds...")
        time.sleep(move_time)
        print("Simulated arrival at target.")

# Function to process each frame and detect the ball and hole
def process_frame(camera_matrix, dist_coeffs, object_d):
    mask = None
    # Capture frame from camera (adjust to your frame capture method)
    # For now, assume frame is provided by the camera
    frame = cv2.imread('test_frame.jpg')  # Replace with actual frame capture

    # Undistort the frame
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix)
    height, width = undistorted_frame.shape[:2]
    bottom_half = undistorted_frame[height//2:, :]  # Bottom half of the frame

    # Ball detection using HSV threshold
    hsv = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        X_ball, Y_ball, Z_ball, R_ball = pixel_to_world(x, y, camera_matrix, radius, object_d)

        # Hole detection using Hough Circles (assuming black holes)
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 100, param1=50, param2=30, minRadius=20, maxRadius=40)

        if circles is not None:
            x_hole, y_hole, r_hole = circles[0][0]
            X_hole, Y_hole, Z_hole, R_hole = pixel_to_world(x_hole, y_hole, camera_matrix, r_hole, 0.052)

            # Plot the ball's trajectory to the hole
            print(f"Ball coordinates: {X_ball}, {Y_ball}, {Z_ball}")
            print(f"Hole coordinates: {X_hole}, {Y_hole}, {Z_hole}")

            # Simulate movement (draw path)
            plot_trajectory(X_ball, Y_ball, Z_ball, X_hole, Y_hole, Z_hole)
            move_robot_to_target(Z_ball, X_ball)
            time.sleep(1)
            move_robot_to_target(Z_hole, X_hole)

            return undistorted_frame, (X_ball, Y_ball, Z_ball), (X_hole, Y_hole, Z_hole)

    return undistorted_frame, None, None

# Function to plot the trajectory from ball to hole
def plot_trajectory(X_ball, Y_ball, Z_ball, X_hole, Y_hole, Z_hole):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot([X_ball, X_hole], [Y_ball, Y_hole], [Z_ball, Z_hole], 'b-o')
    ax.scatter(X_ball, Y_ball, Z_ball, color='g', s=100)
    ax.scatter(X_hole, Y_hole, Z_hole, color='r', s=100)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend(['Path', 'Ball', 'Hole'])
    plt.show()

# Main loop
def test_loop():
    while True:
        user_input = input("Press Enter to move to object, or type 'exit' to quit: ")
        if user_input.lower() == 'exit':
            break
        elif user_input.lower() == '1':
            processed_frame, ball_position, hole_position = process_frame(camera_matrix, dist_coeffs, object_d)
            
            if ball_position and hole_position:
                print(f"Moving ball from {ball_position} to hole at {hole_position}")
                move_robot_to_target(ball_position[2], ball_position[0])
                time.sleep(1)
                move_robot_to_target(hole_position[2], hole_position[0])
                print("Ball placed in hole.")
            else:
                print("Ball or hole not detected.")
