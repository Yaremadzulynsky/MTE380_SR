
# Purpose
Facilitate communication between the control system and other components via a RESTful API.

# Functions
- Provide a communication interface for the control system to interact with other system components.
- Provide endpoints for setting and getting control parameters such as Proportional, Integral, and Derivative values.

# Technologies
Python
SPI
Flask
RESTful API


# Ingress
- set Proportional returns set P value and returns 200 OK
- set Integral returns set I value and returns 200 OK
- set Derivative returns set D value and returns 200 OK
- get Proportional returns P
- get Integral returns I
- get Derivative returns D

# Egress
- SPI communication to the Arduino
  - Send updated PID parameters to the Arduino for control adjustments.
  - Send direction and speed commands to the Arduino for motor control.

- Send data and logs to insights aggregator via IPC pipe.