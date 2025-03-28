import cv2
import numpy as np
import glob

# Chessboard parameters
CHESSBOARD_SIZE = (9, 6)  # Change this based on your chessboard
SQUARE_SIZE = 0.025  # Real-world size of a square in meters (adjust accordingly)

# Prepare object points like (0,0,0), (1,0,0), (2,0,0) ..., (8,5,0)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # Scale to real-world size

objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Load images
images = glob.glob("/home/local/svn/robobot/mqtt_python/group_103/captured_images/*.jpg")  # Put calibration images in this folder

if len(images) == 0:
    print("No images found! Check the folder path.")
else:
    print(f"Found {len(images)} images for calibration.")

for fname in images:
    img = cv2.imread(fname)
    if img is None:  # Ensure the image is loaded correctly
        print(f"Failed to load image {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners, ret)
        #cv2.imshow("Calibration", img)
        #cv2.waitKey(500)
        print(f"Found corners in {fname}")
    else:
        print(f"No corners found in {fname}")

cv2.destroyAllWindows()

# Check if enough valid images were found
if len(objpoints) < 10:  # Minimum number of images for calibration
    print("Not enough valid images with detected corners.")
else:
    # Perform camera calibration
    ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save calibration data
    np.savez("calibration_data.npz", camera_matrix=camera_matrix, distortion_coeffs=distortion_coeffs)
    print("Calibration complete! Data saved to calibration_data.npz.")
