#!/usr/bin/env python
import cv2
import face_recognition
import os
import numpy as np

# Function to load known faces from dataset
def load_known_faces(dataset_path):
    known_face_encodings = []
    known_face_names = []
   
    for person_name in os.listdir(dataset_path):
        person_dir = os.path.join(dataset_path, person_name)
        if os.path.isdir(person_dir):
            for image_file in os.listdir(person_dir):
                image_path = os.path.join(person_dir, image_file)
                image = face_recognition.load_image_file(image_path)
                face_encodings = face_recognition.face_encodings(image)
               
                for face_encoding in face_encodings:
                    known_face_encodings.append(face_encoding)
                    known_face_names.append(person_name)
                   
    return known_face_encodings, known_face_names

# Load known faces
dataset_path = "/home/user/catkin_ws/src/detection/dataset"
known_face_encodings, known_face_names = load_known_faces(dataset_path)

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

# Start video capture
video_capture = cv2.VideoCapture(1)

while True:
    ret, frame = video_capture.read()
    if not ret:
        print("Failed to grab frame")
        continue

    # Resize frame to 1/4 size for faster processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR to RGB
    rgb_small_frame = small_frame[:, :, ::1]

    # Only process every other frame of video to save time
    if process_this_frame:
        # Detect face locations and face encodings
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # Compare to known faces
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=0.4)
            name = "Unknown"

            # Use the known face with the smallest distance
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]

            face_names.append(name)

    process_this_frame = not process_this_frame

    # Display results
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        # Draw a rectangle around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with the name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting frame
    cv2.imshow('Video', frame)

    # Exit the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture
video_capture.release()
cv2.destroyAllWindows()

