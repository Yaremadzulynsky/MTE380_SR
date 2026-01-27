
# Purpose
Allow users to tune the Proportional, Integral, and Derivative (PID) parameters of the control system through a web interface.

# Functions
- Display current PID parameters (P, I, D) on the screen.
- Allow users to input new values for P, I, and D.
- Validate user inputs to ensure they are within acceptable ranges.
- Send updated PID parameters to the control communication endpoint.
- Provide feedback to the user on successful updates or errors.

# Technologies
- HTML
- CSS
- JavaScript
- Express.js (Node.js framework)
- RESTful API

# Ingress
- User inputs via the control tuning screen.
# Egress
Will send HTTP GET/POST requests to the control communication endpoint.
- set Proportional returns set P value and returns 200 OK
- set Integral returns set I value and returns 200 OK
- set Derivative returns set D value and returns 200 OK
- get Proportional returns P
- get Integral returns I
- get Derivative returns D

- Send data and logs to insights aggregator via IPC pipe.