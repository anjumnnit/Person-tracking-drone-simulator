Person-Tracking Drone Simulator

🚀 Introduction
This project simulates a drone capable of detecting and tracking people in real time using a YOLOv8 model and OpenCV, integrated with DroneKit and ArduPilot SITL. The system uses computer vision to calculate a person's position and adjusts the drone's flight path accordingly. It's ideal for experimenting with AI-powered autonomous drone applications.

🎯 Objectives
Train a custom YOLOv8 model on the COCO dataset for person detection.

Simulate a drone using ArduPilot SITL and control it via DroneKit.

Use OpenCV and YOLO to detect a person through a webcam feed.

Command the drone to follow the person based on bounding box midpoints.

🔧 Technologies Used
DroneKit-Python

ArduPilot SITL (Software-In-The-Loop)

MAVProxy

Mission Planner (for optional visualization)

YOLOv8 (ultralytics)

OpenCV

Python 3.8+

📁 Project Structure
plaintext
Copy
├── eda.py                  # Filter COCO dataset for 'person' class
├── train.py                # Train YOLOv8 on filtered dataset
├── track.py                # Track people in video using trained model
├── tracking.py             # Real-time person tracking + Drone movement
├── yolov5-coco-datasets/   # Exported YOLO dataset
├── training/               # Trained model weights
├── requirements.txt
└── README.md
✅ Features
Real-time webcam input for detection and tracking

Circle or person-based visual tracking via OpenCV

Integration with simulated drone via DroneKit

Precision commands sent based on detection location

Preprocessing of COCO dataset for faster training

Model support: YOLOv8n (custom-trained)

📦 Installation
Clone the repository:

bash
Copy
git clone https://github.com/anjumnnit/Person-tracking-drone-simulator.git
cd Person-tracking-drone-simulator
Set up Python environment:

bash
Copy
python -m venv myenv
source myenv/bin/activate  # On Windows: myenv\Scripts\activate
pip install -r requirements.txt
Install and run DroneKit-SITL:

bash
Copy
pip install dronekit-sitl
dronekit-sitl copter
(Optional) Launch MAVProxy:

bash
Copy
mavproxy.py --master tcp:127.0.0.1:5760 --console --map
Connect with Mission Planner (TCP port: 5760) for visual control if desired.

📊 Exploratory Data Analysis (EDA)
To filter the COCO 2017 dataset for the 'person' class and convert it to YOLO format:

bash
Copy
python eda.py
This will export only person-related annotations to yolov5-coco-datasets/.

🏋️‍♂️ Model Training
To train YOLOv8 on the custom dataset:

bash
Copy
python train.py
The trained model will be saved in the training/ directory (e.g., yolov8n.pt).

🎯 Real-Time Person Tracking with Drone Control
Use this to open webcam, detect person, and send MAVLink commands to the simulated drone:

bash
Copy
python tracking.py
The drone will take off and attempt to follow the largest detected person using bounding box midpoint navigation logic.

📽️ Output Example
Output video shows bounding boxes with tracking info.

Terminal displays drone attitude and command logs.

Person movement results in drone motion simulation.

📈 Model Performance
YOLOv8 trained on the filtered COCO dataset showed:

yolov8n - mAP: 0.61287

yolov8s - mAP: 0.56026

yolov8m - mAP: 0.59617

🛠️ Future Improvements
Add Kalman Filter or Deep SORT for better tracking.

Deploy on a real drone using Raspberry Pi + Pixhawk.

Add precision landing using circle detection (ArUco/AprilTag).

Improve drone navigation logic (e.g., PID controller).

Expand to multiple-person tracking.

📜 License
This project is licensed under the MIT License.

Would you like me to generate a .gitignore file now to clean up your repo before final push?
