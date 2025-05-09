# ROS 1 Silvus Radio Driver

## Docker instructions
0. `cd docker`  
1. Build image: `docker compose -f compose.yml build`  
2. Run container: `./run.sh`  
3. Get shell: `docker exec -it silvus-driver bash`  

## Nodes
- `scripts/rssi_publisher.py` -- Reads and publishes RSSI data from radio. Requires RSSI reporting configuration.  
- `scripts/tof_publisher.py` -- Requests, receives, and publishes ToF data from radio.  
