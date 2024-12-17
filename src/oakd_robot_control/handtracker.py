import cv2
import mediapipe as mp
import numpy as np

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, 
                       max_num_hands=1, 
                       min_detection_confidence=0.5, 
                       min_tracking_confidence=0.5)

# Initialize OpenCV for video capture
cap = cv2.VideoCapture(0) 

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Flip the frame for a mirrored view (optional)
    frame = cv2.flip(frame, 1)

    # Convert the frame to RGB for MediaPipe processing
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Extract landmark coordinates
            height, width, _ = frame.shape
            points_2d = [(int(landmark.x * width), int(landmark.y * height)) 
                         for landmark in hand_landmarks.landmark]
            points_3d = [(landmark.x, landmark.y, landmark.z) 
                         for landmark in hand_landmarks.landmark]

            # Calculate the center of the palm in 3D space
            palm_indices = [0, 1, 5, 9, 13, 17]  # Selected key palm landmarks
            palm_points_3d = [points_3d[i] for i in palm_indices]

            # Compute the mean 3D coordinates for the palm center
            center_x = np.mean([point[0] for point in palm_points_3d])
            center_y = np.mean([point[1] for point in palm_points_3d])
            center_z = np.mean([point[2] for point in palm_points_3d])
            center_3d = (center_x, center_y, center_z)

            # Print the 3D coordinates of the palm center
            print(f"Palm center in 3D: x={center_3d[0]:.4f}, y={center_3d[1]:.4f}, z={center_3d[2]:.4f}")

            # Draw the landmarks and center
            for point in points_2d:
                cv2.circle(frame, point, 5, (0, 255, 0), -1)  
            cv2.circle(frame, points_2d[0], 9, (0, 0, 255), -1)  
            cv2.putText(frame, "Center", points_2d[0], cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 0, 0), 2)

            # Draw the hand landmarks
            mp.solutions.drawing_utils.draw_landmarks(
                frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Display the frame
    cv2.imshow('Hand Center Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
hands.close()
