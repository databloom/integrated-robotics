# Isaac-Nexus Software Configuration Guide

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [ROS 2 Setup](#2-ros-2-setup)
3. [NVIDIA Isaac GR00T Integration](#3-nvidia-isaac-gr00t-integration)
4. [NFS 4.2 Configuration](#4-nfs-42-configuration)
5. [Hammerspace Setup](#5-hammerspace-setup)
6. [MCP Implementation](#6-mcp-implementation)
7. [SCADA Integration](#7-scada-integration)
8. [Video Processing Setup](#8-video-processing-setup)
9. [System Integration](#9-system-integration)
10. [Troubleshooting](#10-troubleshooting)

## 1. System Requirements

### 1.1 Hardware Requirements

**Minimum Requirements:**
- CPU: 4-core x86_64 processor
- RAM: 8GB
- Storage: 100GB SSD
- Network: Gigabit Ethernet
- GPU: NVIDIA GTX 1060 or better (for AI processing)

**Recommended Requirements:**
- CPU: 8-core x86_64 processor
- RAM: 32GB
- Storage: 500GB NVMe SSD
- Network: 10 Gigabit Ethernet
- GPU: NVIDIA RTX 3080 or better

### 1.2 Software Requirements

**Operating System:**
- Ubuntu 22.04 LTS (recommended)
- Ubuntu 20.04 LTS (supported)
- CentOS 8+ (supported)

**Dependencies:**
- Docker Engine 20.10+
- Docker Compose 2.0+
- Python 3.8+
- Node.js 16+
- Git 2.30+

## 2. ROS 2 Setup

### 2.1 Installation

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.2 Workspace Setup

```bash
# Create workspace
mkdir -p ~/isaac_nexus_ws/src
cd ~/isaac_nexus_ws

# Clone Isaac-Nexus packages
cd src
git clone https://github.com/isaac-nexus/isaac_nexus_ros2.git
git clone https://github.com/isaac-nexus/robot_control.git
git clone https://github.com/isaac-nexus/sensor_drivers.git
git clone https://github.com/isaac-nexus/ai_processing.git

# Install dependencies
cd ~/isaac_nexus_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install
```

### 2.3 Configuration Files

**Robot Configuration (`config/robot_config.yaml`):**
```yaml
robot:
  type: "aerial_drone"  # aerial_drone, ground_crawler, submarine_drone
  id: "drone_001"
  namespace: "/isaac_nexus/drone_001"
  
sensors:
  lidar:
    enabled: true
    topic: "/sensors/lidar"
    frame_id: "lidar_link"
  camera:
    enabled: true
    topic: "/sensors/camera"
    frame_id: "camera_link"
  imu:
    enabled: true
    topic: "/sensors/imu"
    frame_id: "imu_link"

actuators:
  motors:
    count: 4
    topic: "/actuators/motors"
  servos:
    count: 6
    topic: "/actuators/servos"

communication:
  mqtt:
    broker: "mqtt://192.168.1.100:1883"
    topics:
      commands: "/isaac_nexus/commands"
      telemetry: "/isaac_nexus/telemetry"
  nfs:
    server: "192.168.1.100"
    mount_point: "/mnt/isaac_nexus"
```

## 3. NVIDIA Isaac GR00T Integration

### 3.1 Installation

```bash
# Install NVIDIA drivers
sudo apt install nvidia-driver-525 -y
sudo reboot

# Install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.0.0/local_installers/cuda-repo-ubuntu2204-12-0-local_12.0.0-525.60.13-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-0-local_12.0.0-525.60.13-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-0-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda

# Install Isaac Sim
wget https://omniverse-content-production.s3-us-west-2.amazonaws.com/IsaacSim/2023.1.1/IsaacSim-linux.tar.gz
tar -xzf IsaacSim-linux.tar.gz
cd IsaacSim-linux
./isaac-sim.sh --install-path ~/isaac_sim
```

### 3.2 Isaac GR00T Setup

```bash
# Create Isaac GR00T environment
conda create -n isaac_groot python=3.8 -y
conda activate isaac_groot

# Install Isaac GR00T
pip install isaac-groot
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Download pre-trained models
isaac-groot download-models --model-path ~/isaac_groot_models
```

### 3.3 Configuration

**Isaac GR00T Configuration (`config/isaac_groot_config.yaml`):**
```yaml
isaac_groot:
  model_path: "~/isaac_groot_models"
  device: "cuda"  # cuda, cpu
  
  perception:
    object_detection:
      model: "yolov8n.pt"
      confidence_threshold: 0.5
      nms_threshold: 0.4
    depth_estimation:
      model: "midas_v21_small_256.pt"
      input_size: [256, 256]
    segmentation:
      model: "sam_vit_b_01ec64.pth"
      
  planning:
    path_planning:
      algorithm: "a_star"
      grid_resolution: 0.1
      safety_margin: 0.5
    obstacle_avoidance:
      enabled: true
      reaction_distance: 2.0
      
  control:
    pid_controller:
      kp: 1.0
      ki: 0.1
      kd: 0.05
    velocity_limits:
      linear: 1.0
      angular: 1.0
```

## 4. NFS 4.2 Configuration

### 4.1 Server Setup

```bash
# Install NFS server
sudo apt install nfs-kernel-server -y

# Create data directory
sudo mkdir -p /mnt/isaac_nexus_data
sudo chown nobody:nogroup /mnt/isaac_nexus_data
sudo chmod 755 /mnt/isaac_nexus_data

# Configure NFS exports
sudo tee /etc/exports << EOF
/mnt/isaac_nexus_data *(rw,sync,no_subtree_check,fsid=0,no_root_squash)
EOF

# Enable NFS 4.2 features
sudo tee -a /etc/nfs.conf << EOF
[nfsd]
vers4=y
vers4.0=y
vers4.1=y
vers4.2=y
EOF

# Start NFS services
sudo systemctl enable nfs-kernel-server
sudo systemctl start nfs-kernel-server
sudo exportfs -a
```

### 4.2 Client Setup

```bash
# Install NFS client
sudo apt install nfs-common -y

# Create mount point
sudo mkdir -p /mnt/isaac_nexus

# Mount NFS share
sudo mount -t nfs4 -o vers=4.2 192.168.1.100:/mnt/isaac_nexus_data /mnt/isaac_nexus

# Add to fstab for persistent mounting
echo "192.168.1.100:/mnt/isaac_nexus_data /mnt/isaac_nexus nfs4 vers=4.2,defaults 0 0" | sudo tee -a /etc/fstab
```

### 4.3 NFS 4.2 Features Configuration

**Tagging System Setup:**
```python
# nfs42_tagging.py
import os
import json
from pathlib import Path

class NFS42Tagging:
    def __init__(self, mount_point="/mnt/isaac_nexus"):
        self.mount_point = Path(mount_point)
        self.tag_file = self.mount_point / ".tags"
        
    def tag_file(self, file_path, tags):
        """Tag a file with metadata"""
        file_path = Path(file_path)
        if not file_path.is_absolute():
            file_path = self.mount_point / file_path
            
        # Create tag entry
        tag_entry = {
            "file_path": str(file_path),
            "tags": tags,
            "timestamp": datetime.utcnow().isoformat()
        }
        
        # Append to tag file
        with open(self.tag_file, "a") as f:
            f.write(json.dumps(tag_entry) + "\n")
            
    def query_files(self, tag_filters):
        """Query files by tag criteria"""
        results = []
        with open(self.tag_file, "r") as f:
            for line in f:
                tag_entry = json.loads(line)
                if self._matches_filters(tag_entry["tags"], tag_filters):
                    results.append(tag_entry)
        return results
        
    def _matches_filters(self, tags, filters):
        """Check if tags match filter criteria"""
        for key, value in filters.items():
            if key not in tags or tags[key] != value:
                return False
        return True
```

## 5. Hammerspace Setup

### 5.1 Anvil Server Installation

```bash
# Download Hammerspace Anvil
wget https://downloads.hammerspace.com/anvil/latest/hammerspace-anvil-latest.tar.gz
tar -xzf hammerspace-anvil-latest.tar.gz
cd hammerspace-anvil

# Install Anvil server
sudo ./install.sh

# Configure Anvil
sudo hammerspace-anvil configure --admin-password "admin123" --data-path "/mnt/hammerspace"
```

### 5.2 Data Mover Setup

```bash
# Install data mover on edge nodes
wget https://downloads.hammerspace.com/data-mover/latest/hammerspace-data-mover-latest.tar.gz
tar -xzf hammerspace-data-mover-latest.tar.gz
cd hammerspace-data-mover

# Install data mover
sudo ./install.sh

# Configure data mover
sudo hammerspace-data-mover configure --anvil-server "192.168.1.100" --local-cache "/mnt/local_cache"
```

### 5.3 Policy Configuration

**Data Movement Policies (`config/hammerspace_policies.yaml`):**
```yaml
policies:
  hot_data:
    name: "Hot Data Policy"
    conditions:
      - field: "access_time"
        operator: "within"
        value: "24h"
      - field: "access_count"
        operator: ">"
        value: 10
    actions:
      - type: "move"
        target: "local_nvme"
        priority: "high"
      - type: "replicate"
        target: "regional_cache"
        count: 2
        
  warm_data:
    name: "Warm Data Policy"
    conditions:
      - field: "access_time"
        operator: "within"
        value: "7d"
      - field: "access_count"
        operator: ">"
        value: 3
    actions:
      - type: "move"
        target: "regional_storage"
        priority: "medium"
        
  cold_data:
    name: "Cold Data Policy"
    conditions:
      - field: "access_time"
        operator: "older_than"
        value: "30d"
      - field: "access_count"
        operator: "<"
        value: 2
    actions:
      - type: "move"
        target: "central_archive"
        priority: "low"
      - type: "compress"
        algorithm: "gzip"
        level: 9
```

## 6. MCP Implementation

### 6.1 MCP Node Setup

```bash
# Install MCP dependencies
pip install p2p-network cryptography asyncio-mqtt

# Create MCP node configuration
mkdir -p ~/mcp_node
cd ~/mcp_node
```

**MCP Node Configuration (`config/mcp_node_config.yaml`):**
```yaml
mcp_node:
  node_id: "node_001"
  network:
    bootstrap_nodes:
      - "192.168.1.100:8000"
      - "192.168.1.101:8000"
    listen_port: 8000
    max_peers: 50
    
  data_store:
    path: "/mnt/mcp_data"
    max_size: "100GB"
    replication_factor: 3
    
  consensus:
    algorithm: "raft"
    election_timeout: "150ms"
    heartbeat_interval: "50ms"
    
  security:
    encryption: "AES-256"
    key_exchange: "ECDH"
    authentication: "RSA-2048"
```

### 6.2 MCP Client Implementation

```python
# mcp_client.py
import asyncio
import json
from mcp_node import MCPNode

class IsaacNexusMCPClient:
    def __init__(self, config_path):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.mcp_node = MCPNode(self.config['mcp_node'])
        
    async def start(self):
        """Start MCP node"""
        await self.mcp_node.start()
        
    async def share_robot_data(self, robot_id, data, metadata):
        """Share robot data with peers"""
        data_id = f"robot_{robot_id}_{datetime.utcnow().timestamp()}"
        
        # Create data block
        data_block = {
            "id": data_id,
            "robot_id": robot_id,
            "data": data,
            "metadata": metadata,
            "timestamp": datetime.utcnow().isoformat()
        }
        
        # Share with peers
        await self.mcp_node.share_data(data_id, data_block, metadata)
        
    async def get_robot_data(self, robot_id, data_type=None):
        """Get robot data from peers"""
        # Query for robot data
        query = {"robot_id": robot_id}
        if data_type:
            query["data_type"] = data_type
            
        results = await self.mcp_node.query_data(query)
        return results
```

## 7. SCADA Integration

### 7.1 Ignition Edge Setup

```bash
# Download Ignition Edge
wget https://files.inductiveautomation.com/ignition/8.1.30/ignition-edge-8.1.30-linux-x64-installer.run
chmod +x ignition-edge-8.1.30-linux-x64-installer.run

# Install Ignition Edge
sudo ./ignition-edge-8.1.30-linux-x64-installer.run

# Start Ignition Edge
sudo systemctl start ignition-edge
sudo systemctl enable ignition-edge
```

### 7.2 MQTT Bridge Configuration

**MQTT Bridge Configuration (`config/mqtt_bridge_config.yaml`):**
```yaml
mqtt_bridge:
  mqtt:
    broker: "192.168.1.100"
    port: 1883
    username: "isaac_nexus"
    password: "password123"
    
  ignition:
    host: "localhost"
    port: 8088
    username: "admin"
    password: "admin123"
    
  topics:
    robot_telemetry: "/isaac_nexus/telemetry/+"
    robot_commands: "/isaac_nexus/commands/+"
    system_status: "/isaac_nexus/status/+"
    
  mappings:
    - mqtt_topic: "/isaac_nexus/telemetry/drone_001"
      ignition_tag: "System/Robots/Drone001/Telemetry"
    - mqtt_topic: "/isaac_nexus/commands/drone_001"
      ignition_tag: "System/Robots/Drone001/Commands"
```

### 7.3 OPC UA Server Setup

```python
# opcua_server.py
import asyncio
from opcua import Server, ua
from opcua.common.node import Node

class IsaacNexusOPCUAServer:
    def __init__(self, endpoint="opc.tcp://0.0.0.0:4840"):
        self.server = Server()
        self.server.set_endpoint(endpoint)
        self.server.set_server_name("Isaac-Nexus OPC UA Server")
        
        # Setup namespace
        self.namespace = self.server.register_namespace("IsaacNexus")
        
    async def start(self):
        """Start OPC UA server"""
        await self.server.start()
        
        # Create robot nodes
        self.robot_nodes = {}
        for robot_id in ["drone_001", "crawler_001", "rov_001"]:
            self.robot_nodes[robot_id] = await self._create_robot_node(robot_id)
            
    async def _create_robot_node(self, robot_id):
        """Create OPC UA node for robot"""
        robot_node = self.server.nodes.objects.add_object(self.namespace, robot_id)
        
        # Add telemetry variables
        telemetry_node = robot_node.add_object(self.namespace, "Telemetry")
        telemetry_node.add_variable(self.namespace, "Position", [0.0, 0.0, 0.0])
        telemetry_node.add_variable(self.namespace, "Battery", 100.0)
        telemetry_node.add_variable(self.namespace, "Status", "Idle")
        
        # Add command variables
        commands_node = robot_node.add_object(self.namespace, "Commands")
        commands_node.add_variable(self.namespace, "Takeoff", False)
        commands_node.add_variable(self.namespace, "Land", False)
        commands_node.add_variable(self.namespace, "EmergencyStop", False)
        
        return robot_node
        
    async def update_telemetry(self, robot_id, telemetry_data):
        """Update robot telemetry"""
        if robot_id in self.robot_nodes:
            robot_node = self.robot_nodes[robot_id]
            telemetry_node = robot_node.get_child(f"{self.namespace}:Telemetry")
            
            # Update position
            position_var = telemetry_node.get_child(f"{self.namespace}:Position")
            position_var.set_value(telemetry_data.get("position", [0.0, 0.0, 0.0]))
            
            # Update battery
            battery_var = telemetry_node.get_child(f"{self.namespace}:Battery")
            battery_var.set_value(telemetry_data.get("battery", 100.0))
            
            # Update status
            status_var = telemetry_node.get_child(f"{self.namespace}:Status")
            status_var.set_value(telemetry_data.get("status", "Unknown"))
```

## 8. Video Processing Setup

### 8.1 FFmpeg Installation

```bash
# Install FFmpeg
sudo apt update
sudo apt install ffmpeg -y

# Install additional codecs
sudo apt install libx264-dev libx265-dev libvpx-dev libfdk-aac-dev -y
```

### 8.2 Video Processing Pipeline

**Video Processing Configuration (`config/video_processing_config.yaml`):**
```yaml
video_processing:
  input:
    sources:
      - type: "rtsp"
        url: "rtsp://192.168.1.100:554/stream1"
        resolution: "1920x1080"
        fps: 30
      - type: "usb_camera"
        device: "/dev/video0"
        resolution: "1280x720"
        fps: 30
        
  processing:
    object_detection:
      enabled: true
      model: "yolov8n.pt"
      confidence_threshold: 0.5
      nms_threshold: 0.4
      
    compression:
      algorithm: "h264"
      bitrate: "2M"
      quality: "medium"
      
    streaming:
      protocol: "rtsp"
      port: 8554
      quality_levels: ["high", "medium", "low"]
      
  output:
    storage:
      path: "/mnt/video_storage"
      format: "mp4"
      retention_days: 30
      
    streaming:
      enabled: true
      endpoints:
        - "rtsp://192.168.1.100:8554/stream1"
        - "http://192.168.1.100:8080/stream1"
```

### 8.3 Real-Time Processing Implementation

```python
# video_processor.py
import cv2
import asyncio
from ultralytics import YOLO

class VideoProcessor:
    def __init__(self, config_path):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Load YOLO model
        self.model = YOLO(self.config['video_processing']['processing']['object_detection']['model'])
        
        # Initialize video capture
        self.cap = cv2.VideoCapture(self.config['video_processing']['input']['sources'][0]['url'])
        
    async def process_video_stream(self):
        """Process video stream in real-time"""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
                
            # Run object detection
            results = self.model(frame)
            
            # Draw bounding boxes
            annotated_frame = self._draw_detections(frame, results)
            
            # Stream processed frame
            await self._stream_frame(annotated_frame)
            
            # Store frame if needed
            await self._store_frame(annotated_frame)
            
            await asyncio.sleep(1/30)  # 30 FPS
            
    def _draw_detections(self, frame, results):
        """Draw detection bounding boxes on frame"""
        annotated_frame = frame.copy()
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                
                # Draw bounding box
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # Draw label
                label = f"{self.model.names[class_id]}: {confidence:.2f}"
                cv2.putText(annotated_frame, label, (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
        return annotated_frame
        
    async def _stream_frame(self, frame):
        """Stream frame to clients"""
        # Implementation for streaming frame
        pass
        
    async def _store_frame(self, frame):
        """Store frame to storage"""
        # Implementation for storing frame
        pass
```

## 9. System Integration

### 9.1 Docker Compose Configuration

**Docker Compose File (`docker-compose.yml`):**
```yaml
version: '3.8'

services:
  ros2-bridge:
    build: ./docker/ros2-bridge
    container_name: isaac_nexus_ros2_bridge
    volumes:
      - ./config:/config
      - ./logs:/logs
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - isaac_nexus_network
      
  mqtt-broker:
    image: eclipse-mosquitto:2.0
    container_name: isaac_nexus_mqtt
    ports:
      - "1883:1883"
      - "9001:9001"
    volumes:
      - ./config/mosquitto.conf:/mosquitto/config/mosquitto.conf
    networks:
      - isaac_nexus_network
      
  nfs-server:
    build: ./docker/nfs-server
    container_name: isaac_nexus_nfs
    ports:
      - "2049:2049"
    volumes:
      - ./data:/data
    networks:
      - isaac_nexus_network
      
  hammerspace-anvil:
    build: ./docker/hammerspace-anvil
    container_name: isaac_nexus_anvil
    ports:
      - "8080:8080"
    volumes:
      - ./hammerspace_data:/hammerspace_data
    networks:
      - isaac_nexus_network
      
  video-processor:
    build: ./docker/video-processor
    container_name: isaac_nexus_video
    volumes:
      - ./config:/config
      - ./video_data:/video_data
    networks:
      - isaac_nexus_network
      
  scada-bridge:
    build: ./docker/scada-bridge
    container_name: isaac_nexus_scada
    ports:
      - "4840:4840"
    volumes:
      - ./config:/config
    networks:
      - isaac_nexus_network

networks:
  isaac_nexus_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
```

### 9.2 System Startup Script

**Startup Script (`scripts/start_system.sh`):**
```bash
#!/bin/bash

# Isaac-Nexus System Startup Script

set -e

echo "Starting Isaac-Nexus System..."

# Check system requirements
echo "Checking system requirements..."
./scripts/check_requirements.sh

# Start Docker services
echo "Starting Docker services..."
docker-compose up -d

# Wait for services to be ready
echo "Waiting for services to be ready..."
sleep 30

# Start ROS 2 nodes
echo "Starting ROS 2 nodes..."
source /opt/ros/humble/setup.bash
source ~/isaac_nexus_ws/install/setup.bash

# Start robot control nodes
ros2 launch isaac_nexus_ros2 robot_control.launch.py &
ros2 launch isaac_nexus_ros2 sensor_drivers.launch.py &
ros2 launch isaac_nexus_ros2 ai_processing.launch.py &

# Start MCP nodes
echo "Starting MCP nodes..."
python3 ~/mcp_node/mcp_client.py --config config/mcp_node_config.yaml &

# Start video processing
echo "Starting video processing..."
python3 ~/video_processor/video_processor.py --config config/video_processing_config.yaml &

# Start SCADA bridge
echo "Starting SCADA bridge..."
python3 ~/scada_bridge/opcua_server.py --config config/opcua_server_config.yaml &

echo "Isaac-Nexus System started successfully!"
echo "Access the web interface at: http://localhost:8080"
echo "Access SCADA at: http://localhost:8088"
```

## 10. Troubleshooting

### 10.1 Common Issues

**ROS 2 Issues:**
```bash
# Check ROS 2 installation
ros2 doctor

# Check node status
ros2 node list
ros2 topic list
ros2 service list

# Check for errors
ros2 node info /node_name
```

**NFS Issues:**
```bash
# Check NFS exports
sudo exportfs -v

# Check NFS mount
mount | grep nfs

# Test NFS connectivity
showmount -e 192.168.1.100
```

**Docker Issues:**
```bash
# Check container status
docker ps -a

# Check container logs
docker logs container_name

# Restart services
docker-compose restart
```

### 10.2 Performance Monitoring

**System Monitoring Script (`scripts/monitor_system.sh`):**
```bash
#!/bin/bash

# Isaac-Nexus System Monitoring Script

echo "=== Isaac-Nexus System Status ==="
echo "Timestamp: $(date)"
echo

# Check Docker services
echo "Docker Services:"
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
echo

# Check ROS 2 nodes
echo "ROS 2 Nodes:"
ros2 node list
echo

# Check system resources
echo "System Resources:"
echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo "Memory Usage: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
echo "Disk Usage: $(df -h / | awk 'NR==2{printf "%s", $5}')"
echo

# Check network connectivity
echo "Network Connectivity:"
ping -c 1 192.168.1.100 > /dev/null && echo "NFS Server: OK" || echo "NFS Server: FAIL"
ping -c 1 192.168.1.101 > /dev/null && echo "MQTT Broker: OK" || echo "MQTT Broker: FAIL"
echo

# Check log files for errors
echo "Recent Errors:"
tail -n 10 /var/log/syslog | grep -i error
echo
```

### 10.3 Log Management

**Log Rotation Configuration (`config/logrotate.conf`):**
```
/var/log/isaac_nexus/*.log {
    daily
    missingok
    rotate 30
    compress
    delaycompress
    notifempty
    create 644 root root
    postrotate
        systemctl reload rsyslog > /dev/null 2>&1 || true
    endscript
}
```

---

*Document Version: 1.0*  
*Last Updated: January 2024*  
*Authors: Isaac-Nexus Software Team*
