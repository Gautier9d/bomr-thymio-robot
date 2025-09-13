# BOMR - Autonomous Thymio Robot Navigation

Autonomous mobile robot navigation system using computer vision, path planning, and state estimation for a Thymio robot.

## 🎯 Project Overview

This project implements an autonomous navigation system for a Thymio educational robot as part of the EPFL Basics of Mobile Robotics (BOMR) course. The system integrates computer vision for environment perception, A* algorithm for global path planning, local obstacle avoidance, and Extended Kalman Filter for robust state estimation.

### Key Features
- **Real-time Computer Vision**: Object detection and tracking using OpenCV
- **Global Navigation**: A* pathfinding with grid-based mapping
- **Local Navigation**: Dynamic obstacle avoidance with kidnapping detection
- **State Estimation**: Extended Kalman Filter for sensor fusion
- **Motion Control**: PID-based differential drive control
- **Finite State Machine**: Robust behavior management

## 📁 Project Structure

```
bomr-thymio-project/
├── CV_utils.py                    # Computer vision utilities and object detection
├── calib_hsv_image.py            # HSV color calibration tool
├── BOMR-PROJECT.ipynb            # Jupyter notebook with analysis and demos
├── docs/
│   └── BOMR_PROJECT_PRESENTATION.pdf  # Project presentation
├── README.md                      # This file
├── requirements.txt              # Python dependencies
└── .gitignore                    # Git ignore rules
```

## 🚀 Getting Started

### Prerequisites
- Python 3.8 or higher
- Webcam or camera for vision system
- Thymio robot (optional for simulation mode)

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/yourusername/bomr-thymio-robot.git
cd bomr-thymio-robot
```

2. **Create a virtual environment** (recommended)
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. **Install dependencies**
```bash
pip install -r requirements.txt
```

## 💻 Usage

### HSV Color Calibration
Calibrate color detection for your specific lighting conditions:

```bash
python calib_hsv_image.py
```
- Place a test image named `test.png` in the project directory
- Adjust the trackbars to find optimal HSV ranges
- Note down the values for use in `CV_utils.py`

### Computer Vision Module
```python
from CV_utils import Grid

# Initialize the grid system
grid = Grid()

# Process camera frames
grid.detect_black_rectangles(frame)  # For calibration
# ... see CV_utils.py for full API
```

### Run Jupyter Notebook
```bash
jupyter notebook BOMR-PROJECT.ipynb
```

## 🔧 System Components

### Computer Vision Pipeline
1. **Anchor Detection**: Identifies reference points for coordinate mapping
2. **Target Detection**: Locates colored circles using Hough Transform
3. **Obstacle Detection**: Identifies and tracks obstacles
4. **Robot Tracking**: Monitors robot position and orientation
5. **Grid Generation**: Converts real-world to grid coordinates

### Navigation System
- **Global Path Planning**: A* algorithm with Euclidean heuristic
- **Local Obstacle Avoidance**: Trial-and-error based system
- **Kidnapping Detection**: IMU-based detection (18g threshold)

### State Machine
- **Initialization**: Map detection, environment setup
- **Loop States**: Kalman update, navigation, target tracking
- **Transitions**: Event-driven state changes

## 📊 Key Algorithms

### Extended Kalman Filter
- Sensor fusion for improved state estimation
- Handles non-linear system dynamics
- Predict-Measure-Update cycle

### Motion Control
- Differential drive kinematics
- PID controller for angular and linear velocity
- Threshold-based movement decisions

## 📝 API Documentation

### CV_utils.Grid Class

#### Key Methods:
- `detect_black_rectangles(frame)`: Calibration rectangles detection
- `get_grid(name)`: Retrieve specific grid by name
- `set_grid(name, grid)`: Update grid data
- `update_stock_obstacles_positions()`: Track obstacle positions
- `draw_constant_polygons(frame)`: Visualize detected obstacles

### Color Detection Functions
- `detect_objects(frame, grid, anchors)`: Main object detection
- `find_robot(vertices)`: Calculate robot orientation
- `smooth_robot(positions)`: Filter robot position noise
- `found_target(robot_pos, targets)`: Check target reached

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📚 Documentation

For detailed information about the algorithms and system architecture, see:
- [Project Presentation](docs/BOMR_PROJECT_PRESENTATION.pdf)
- [Jupyter Notebook Analysis](BOMR-PROJECT.ipynb)

## 📄 License

This project is licensed under the MIT License - see below for details:

```
MIT License

Copyright (c) 2024 EPFL - École Polytechnique Fédérale de Lausanne
Basics of Mobile Robotics course Project

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction.
```

## 👥 Authors

- BOMR Project Team - EPFL Robotics Course

## 🙏 Acknowledgments

- EPFL Robotics Laboratory
- Thymio educational robot platform
- OpenCV community

---

*For questions or support, please open an issue on GitHub.*
