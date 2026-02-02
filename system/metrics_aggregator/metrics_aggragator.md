
# Purpose
Opens an IPC pipe to receive logs and metrics data from other system components and provide it in a format for prometheus to scrape and stdout for grafana alloy to ingest.

# Functions
- Provide a queryable database for logs.

# Technologies
- Grafana Alloy
- Prometheus?

# Ingress
Tailing docker logs from docker containers
IPC pipe from prometheus metrics.

# Egress
to alloy and prometheus for evaluation.