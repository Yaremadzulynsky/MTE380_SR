
# Purpose
State machine that ingests information and provides system state outputs for control system.

# Functions
- State machine that ingests information and provides system state outputs for control system.
![alt text](image.png)

# Technologies
- Python

# Ingress

# Egress
REST API calls to control communication system to get PID parameters and system state.
REST API calls to computer vision system to get object detection and distance data.
Send data and logs to insights aggregator via IPC pipe.

