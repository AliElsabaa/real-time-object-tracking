import cv2
from ultralytics import YOLO

model = YOLO("yolov12n.pt")  # Load YOLOv12 model

cap = cv2.VideoCapture(0)  # Open webcam
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

selected_box = None        # Selected object bounding box
tracker = None             # OpenCV tracker object
tracked_object_name = None # Name of tracked object
object_lost = False        # Flag for lost tracking

# Ensure the window is created before setting the mouse callback
cv2.namedWindow("Select Object")

def select_object(event, x, y, flags, param):
    global selected_box, tracker, tracked_object_name

    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button clicked
        print(f"Mouse clicked at: {x}, {y}")

        results = model(frame)  # Perform object detection on current frame
        boxes = []  # List of bounding boxes containing mouse click

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                if x1 <= x <= x2 and y1 <= y <= y2:  # Check if mouse click is inside bounding box
                    conf = float(box.conf[0])  # Confidence score
                    cls = int(box.cls[0])  # Class index
                    object_name = model.names[cls]  # Class name
                    area = (x2 - x1) * (y2 - y1)  # Area of bounding box
                    boxes.append(((x1, y1, x2, y2, object_name, conf), area))

        if not boxes:
            print("No object selected.")
            return
        else:
            boxes.sort(key=lambda b: b[1])  # Sort by area (smallest first)
            selected_box, tracked_object_name = boxes[0][0][:4], boxes[0][0][4]  # Pick smallest box

        # Initialize tracker
        if selected_box:
            tracker = cv2.TrackerCSRT_create()
            tracker.init(frame, (selected_box[0], selected_box[1], selected_box[2] - selected_box[0], selected_box[3] - selected_box[1])) 
            cv2.destroyWindow("Select Object")

cv2.setMouseCallback("Select Object", select_object)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    results = model(frame)
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = f"{model.names[cls]} {conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Select Object", frame)

    if selected_box:
        break
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

cv2.namedWindow("Tracking")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    success, new_box = tracker.update(frame)

    if success:
        x, y, w, h = map(int, new_box)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, tracked_object_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        object_lost = False

    else:
        if not object_lost:
            print("Object lost. Searching...")
            object_lost = True

        results = model(frame)
        max_conf = 0
        found_box = None

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                object_name = model.names[cls]

                if object_name == tracked_object_name and conf >= 0.7:
                    if found_box is None or conf > max_conf:
                        found_box = (x1, y1, x2, y2)
                        max_conf = conf

        if found_box:
            print(f"Object {tracked_object_name} found with confidence {max_conf:.2f}! Restarting tracking...")
            x1, y1, x2, y2 = found_box
            tracker = cv2.TrackerCSRT_create()
            tracker.init(frame, (x1, y1, x2 - x1, y2 - y1))
            object_lost = False

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()