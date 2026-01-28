
# Purpose
Opens an IPC pipe to receive logs and metrics data from other system components and provide it in a format for prometheus to scrape and stdout for grafana alloy to ingest.

# Functions
- Provide a queryable database for logs.

# Technologies
- Grafana Alloy

# Ingress
Tailing docker logs from docker containers

# Egress
to grafana dashboard evaluation.