import cv2
from apriltag import apriltag

def detect_apriltags(camera_index=0):
    # Create a VideoCapture object for the camera (change the index if needed)
    cap = cv2.VideoCapture(camera_index)

    # Create an AprilTag detector
    options = apriltag.DetectorOptions(families='tag36h11')
    detector = apriltag.Detector(options)

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()

        # Convert the frame to grayscale for AprilTag detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the frame
        result = detector.detect(gray)

        # Draw bounding boxes and tag ID on the frame
        for detection in result:
            rect = detection['lb-rb-rt-lt'].reshape((-1, 2)).astype(int)
            cv2.polylines(frame, [rect], True, (0, 255, 0), 2)
            cv2.putText(frame, str(detection['id']), (rect[0, 0], rect[0, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame with AprilTag detection
        cv2.imshow('AprilTag Detection', frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Call the function to detect AprilTags using the default camera (index 0)
    detect_apriltags()

