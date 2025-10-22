# ROS2 Webcam Publisher and Subscriber

This repository contains a simple ROS2 package that demonstrates publishing and subscribing to webcam frames using OpenCV and `cv_bridge`.

## Package Structure

webcam_publisher/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│ └── webcam_publisher
├── webcam_publisher/
│ ├── init.py
│ ├── webcam_publisher_node.py
│ └── webcam_subscriber_node.py
└── test/
├── test_copyright.py
├── test_flake8.py
└── test_pep257.py

markdown
Copy code

- **webcam_publisher_node.py** – Publishes webcam frames on the ROS2 topic `webcam_publisher`.  
- **webcam_subscriber_node.py** – Subscribes to the topic `webcam_publisher` and displays the webcam frames.  

## Dependencies

- ROS2 Humble or later
- Python 3.10+
- OpenCV
- cv_bridge

You can install ROS2 dependencies with:

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv
Building the Package
From the root of your ROS2 workspace:

bash
Copy code
colcon build --packages-select webcam_publisher
source install/setup.bash
Running the Nodes
Start the Publisher
bash
Copy code
ros2 run webcam_publisher webcam_publisher_node
Start the Subscriber (in another terminal)
bash
Copy code
ros2 run webcam_publisher webcam_subscriber_node
You should see the webcam feed displayed by the subscriber node.

Notes
Make sure your webcam is connected and accessible.

If the subscriber shows blank frames, check the camera index in VideoCapture(0) inside webcam_publisher_node.py.

Resolution can be adjusted via:

python
Copy code
self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
License
MIT License
