# AG4NBA — luggage validation and HMI

Three services I built during my internship at Fraunhofer IEM for AG4NBA, an automated luggage handling system. The system is a set of Python microservices around a conveyor belt with two camera zones and a LiDAR, exchanging data through Redis Streams. This repository holds my part of it.

| Service | What it does |
|---|---|
| [per07 — Dynamic Validation Service](services/perception/per07_dynamic_validation_service/README.md) | Confirms that the luggage seen at Zone 1 (camera and LiDAR) is the same physical piece that reaches Zone 2 (second camera) 60–90 s later. It scores every Zone 2 detection on SIFT features verified with RANSAC, class and colour, and raises an alert after three failed attempts. |
| [hmi01 — Configuration](services/human_machine_interface/hmi01_configuration/README.md) | Reads the output of every service in the system once a second, strictly read-only, and publishes one snapshot for the HMI. |
| [hmi02 — Dashboard](services/human_machine_interface/hmi02_dashboard/README.md) | Serves the Luggage Handling System HMI: live pipeline health, a digital twin of the belt, validation results and alerts. |

**Built with:** Python, Redis Streams, OpenCV (SIFT, RANSAC, colour histograms), Ultralytics YOLO, Open3D and NumPy. The HMI is served by Python's standard `http.server` and rendered with React in the browser.

## Not included

- **The rest of the system.** The services these three read from (sensor streaming, detection, tracking and others) belong to other team members. Without them, these services start but have nothing to validate or show.
- **Model weights and camera recordings.**
- **Internal network addresses.** They are replaced with documentation addresses (`192.0.2.x`).
