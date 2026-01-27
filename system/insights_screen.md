
# Purpose
Provide a visual interface to graphically view system insights and performance metrics as well as commands being sent to the control system.

# Functions
- Visualize control commands being sent to the control system.
- Provide historical data analysis and visualization.

# Technologies
- Grafana Dashboard
- Querying with Prometheus
- Querying with Grafana Alloy




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