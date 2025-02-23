import cv2

cap = cv2.VideoCapture(0)  # Open default webcam

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    cv2.imshow("Webcam Feed", frame)  # Display webcam feed

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cap.release()
cv2.destroyAllWindows()