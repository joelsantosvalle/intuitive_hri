import cv2
import numpy as np

# Define the chessboard dimensions
chessboard_size = (7, 6)  # Change to your chessboard size
square_size = 0.025  # Size of a square in meters (adjust accordingly)

# Prepare object points (3D points in real-world space)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Capture images for calibration
cap = cv2.VideoCapture(0)
print("Press 'c' to capture an image, and 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)
        cv2.imshow("Calibration", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c') and ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        print("Captured image for calibration.")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)
