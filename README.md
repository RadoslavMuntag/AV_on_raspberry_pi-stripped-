# Self-Driving Vehicle on Raspberry Pi

## Project Overview

This repository is a stripped down version of a fork of the [Freenove Tank Robot Kit for Raspberry Pi](https://github.com/Freenove/Freenove_Tank_Robot_Kit_for_Raspberry_Pi) (main branch). It serves as the foundation for a bachelor's thesis project focused on developing autonomous vehicle capabilities on the Raspberry Pi platform.

## Purpose

The primary objective of this project is to explore and implement self-driving vehicle technologies using a Raspberry Pi-based tank robot. This includes integrating computer vision, sensor fusion, and control algorithms to enable autonomous navigation, obstacle avoidance, and path planning in a controlled environment.

## Key Features

- **Hardware Integration**: Utilizes the Freenove Tank Robot Kit components including motors, servos, ultrasonic sensors and camera modules.
- **Computer Vision**: Implements image processing and object detection for environmental awareness.
- **Autonomous Control**: Develops algorithms for self-driving functionality, including lane following, obstacle detection, and decision-making.
- **Real-time Processing**: Leverages Raspberry Pi's capabilities for on-board processing and control.

## Project Structure

## Repository Structure
AV_on_raspberry_pi-stripped-/
│
├── backend/                    # Application backend
│   ├── config/                 # Configuration
│   ├── misc/                   # Additional utilities
│   │   ├── dualsense/          # DualSense controller support
│   │   └── snapshots/          # Image snapshots
│   ├── pipeline/               # Modular autonomous pipeline
│   ├── services/
│   │   ├── hardware.py         # HAL bridging model to backend
│   │   ├── runtime.py          # Runtime control loop
│   │   └── state.py            # Application state management
│   ├── static/                 # Frontend (HTML, JS, CSS)
│   ├── app.py                  # Backend and API endpoints
│   └── contracts.py            # Frame and sensor dataclasses
│
├── model/                      # Freenove hardware interface
│
├── systemd-service/            # systemd service template
├── requirements.txt            # Python dependencies
└── run-backend.py              # Uvicorn entry point
```



## Acknowledgments

This project is based on the open-source Freenove Tank Robot Kit. Special thanks to Freenove for providing the hardware platform and initial codebase.

## IMPORTANT

If using a virtual enviroment for running the server and have - ModuleNotFoundError: No module named 'libcamera' - error

run:
```
/usr/bin/python3 -m venv --system-site-packages .venv

(.venv is the name of your virtual enviroment folder)
```


## License

This project inherits the license from the original Freenove repository: [Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License](http://creativecommons.org/licenses/by-nc-sa/3.0/).
