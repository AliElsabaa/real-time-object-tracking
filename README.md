# 📌 Real-Time Object Tracking with YOLOv12 and OpenCV

A real-time object tracking system that combines **YOLOv12** detection with 
**OpenCV's CSRT tracker**. Point your webcam at any scene, click on an object, 
and the system tracks it continuously — automatically re-detecting it if tracking 
is lost.

---

## 🎯 How It Works

1. **Detection phase** — YOLOv12 runs on the live webcam feed and draws bounding 
   boxes around all detected objects in real time.
2. **Selection** — Click on any detected object with your mouse. If multiple 
   bounding boxes overlap at that point, the system picks the smallest (most 
   specific) one.
3. **Tracking phase** — OpenCV's CSRT tracker takes over, tracking the selected 
   object frame by frame without running the full detection model on every frame.
4. **Re-detection** — If the tracker loses the object (e.g., it leaves the frame 
   or is occluded), YOLOv12 re-runs automatically and resumes tracking as soon as 
   the object reappears with confidence ≥ 0.7.

---

## ✅ Features

- **Real-time detection** with YOLOv12 (nano variant — fast and lightweight)
- **Click-to-track** — select any object from the live feed with a single mouse click
- **CSRT tracking** — accurate, robust tracker for smooth object following
- **Automatic re-detection** — recovers tracking after object loss without 
  user intervention
- **Smallest-box selection** — intelligently selects the most specific bounding 
  box when objects overlap

---

## 🛠️ Tech Stack

| Tool | Role |
|------|------|
| YOLOv12 (Ultralytics) | Real-time object detection |
| OpenCV CSRT Tracker | Frame-by-frame object tracking |
| Python 3.8+ | Core language |

---

## 📁 Project Structure

real-time-object-tracking/

│

├── object_tracker.py    # Main script — detection, selection, and tracking

├── test_webcam.py       # Simple webcam test to verify camera access

├── yolov12n.pt          # YOLOv12 nano pretrained model weights

└── README.md
---

## ⚙️ Setup

1. Install dependencies:
```bash
pip install ultralytics opencv-python
```

2. Verify your webcam works:
```bash
python test_webcam.py
```

3. Run the tracker:
```bash
python object_tracker.py
```

---

## 🖱️ Controls

| Action | Key / Input |
|--------|------------|
| Select object to track | Left mouse click on detected object |
| Quit | Press `q` |

---

## 📝 Notes

- The model used is `yolov12n.pt` (nano) — optimized for speed on CPU.
- CSRT is chosen over simpler trackers (KCF, MOSSE) for its higher accuracy 
  on fast-moving or partially occluded objects.
- Re-detection only triggers if the CSRT tracker explicitly fails — keeping 
  CPU usage low during stable tracking.
