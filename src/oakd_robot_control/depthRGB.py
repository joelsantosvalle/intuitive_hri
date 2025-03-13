"""
PoseTracking.py

@date 07.12.2024
@author Joel Santos

This script uses MediaPipe's Holistic model to track human pose landmarks in real-time
from a webcam feed. It detects and visualizes key body landmarks while printing the 
depth (z-coordinate) of the nose.

Press 'q' to exit the application.
"""

import cv2
import mediapipe as mp

# Initialize MediaPipe Holistic model and drawing utilities
mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils

# Open webcam feed
cap = cv2.VideoCapture(0)

# Configure MediaPipe Holistic model with detection and tracking confidence
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to RGB format (required for MediaPipe)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame to detect pose landmarks
        results = holistic.process(frame_rgb)

        # If landmarks are detected, print the nose depth (z-axis)
        if results.pose_landmarks:
            nose_landmark = results.pose_landmarks.landmark[mp_holistic.PoseLandmark.NOSE]
            print("Nose depth:", nose_landmark.z)

        # Draw pose landmarks on the frame
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)

        # Display the annotated frame
        cv2.imshow('MediaPipe Holistic', frame)

        # Exit if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
