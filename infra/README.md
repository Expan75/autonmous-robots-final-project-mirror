# Infrastructure documentation

Welcome to the infra part of the repository! Here you'll find all the supporting services for both cloud and robot. Mainly this concerns telemetry and dashboards.

## Local development

To recreate both the robot and cloud environmnents locally using docker, all you need to do is:

```bash
cd infra

# bring up the cloud
ADMIN_USER='team11' ADMIN_PASSWORD='admin' ADMIN_PASSWORD_HASH='$2a$14$1l.IozJx7xQRVmlkEQ32OeEEfP5mRxTpbDTCTcXRqn19gXD8YK1pO' docker-compose -f cloud.yml up

# bring up the robot (MacOS, cadvisor not supported)
docker compose -f robot up nodeexporter

# bring up the robot (linux)
docker compose -f robot up
```

NOTE: For ansible development, see ansible/README.md!
