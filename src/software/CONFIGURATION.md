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

## 6. MCP (Model Context Protocol) Implementation

### 6.1 MCP Server Setup

```bash
# Install MCP dependencies
pip install mcp-server-sdk asyncio-mqtt cryptography websockets

# Create MCP server configuration
mkdir -p ~/mcp_server
cd ~/mcp_server
```

**MCP Server Configuration (`config/mcp_server_config.yaml`):**
```yaml
mcp_server:
  server_id: "isaac_nexus_mcp_server"
  version: "2024-11-05"
  host: "0.0.0.0"
  port: 8000
  
  # Agent Communication
  agent_communication:
    protocol: "websocket"
    max_connections: 100
    heartbeat_interval: 30
    connection_timeout: 60
    
  # Tool Registry
  tool_registry:
    discovery_enabled: true
    tool_timeout: 30
    max_concurrent_tools: 50
    tool_validation: true
    
  # Context Management
  context:
    retention_period: "24h"
    max_context_size: "1MB"
    context_compression: true
    shared_memory: true
    
  # Security
  security:
    authentication: "oauth2_jwt"
    authorization: "rbac"
    encryption: "tls_1.3"
    key_management: "hsm_based"
    audit_logging: true
    session_management: "stateless"
    
  # Fleet Control
  fleet_control:
    max_robots: 100
    task_delegation: true
    agent_coordination: true
    real_time_monitoring: true
    emergency_override: true
```

### 6.2 MCP Tool Calling System

**MCP Tool Registry (`src/mcp/tool_registry.py`):**
```python
import asyncio
import json
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
from datetime import datetime
import logging

@dataclass
class ToolDefinition:
    name: str
    description: str
    parameters: Dict[str, Any]
    return_type: str
    timeout: int
    category: str
    robot_types: List[str]
    requires_auth: bool

class MCPToolRegistry:
    def __init__(self):
        self.tools: Dict[str, ToolDefinition] = {}
        self.tool_handlers: Dict[str, callable] = {}
        self.logger = logging.getLogger(__name__)
        
    def register_tool(self, tool_def: ToolDefinition, handler: callable):
        """Register a new tool with the MCP server"""
        self.tools[tool_def.name] = tool_def
        self.tool_handlers[tool_def.name] = handler
        self.logger.info(f"Registered tool: {tool_def.name}")
        
    def discover_tools(self, robot_type: str = None, category: str = None) -> List[ToolDefinition]:
        """Discover available tools for a robot type or category"""
        filtered_tools = []
        
        for tool in self.tools.values():
            if robot_type and robot_type not in tool.robot_types:
                continue
            if category and tool.category != category:
                continue
            filtered_tools.append(tool)
            
        return filtered_tools
        
    async def execute_tool(self, tool_name: str, parameters: Dict[str, Any], 
                          context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Execute a tool with given parameters and context"""
        if tool_name not in self.tools:
            raise ValueError(f"Tool {tool_name} not found")
            
        tool_def = self.tools[tool_name]
        handler = self.tool_handlers[tool_name]
        
        # Validate parameters
        self._validate_parameters(tool_def, parameters)
        
        # Execute tool with timeout
        try:
            result = await asyncio.wait_for(
                handler(parameters, context),
                timeout=tool_def.timeout
            )
            
            return {
                "success": True,
                "result": result,
                "tool_name": tool_name,
                "timestamp": datetime.utcnow().isoformat()
            }
            
        except asyncio.TimeoutError:
            return {
                "success": False,
                "error": f"Tool {tool_name} timed out after {tool_def.timeout}s",
                "tool_name": tool_name,
                "timestamp": datetime.utcnow().isoformat()
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "tool_name": tool_name,
                "timestamp": datetime.utcnow().isoformat()
            }
            
    def _validate_parameters(self, tool_def: ToolDefinition, parameters: Dict[str, Any]):
        """Validate tool parameters against definition"""
        required_params = tool_def.parameters.get("required", [])
        
        for param in required_params:
            if param not in parameters:
                raise ValueError(f"Required parameter {param} not provided")
```

### 6.3 Agent Fleet Control Tools

**Fleet Control Tools (`src/mcp/fleet_tools.py`):**
```python
from mcp.tool_registry import MCPToolRegistry, ToolDefinition
from typing import Dict, List, Any
import asyncio

class FleetControlTools:
    def __init__(self, tool_registry: MCPToolRegistry):
        self.tool_registry = tool_registry
        self._register_fleet_tools()
        
    def _register_fleet_tools(self):
        """Register all fleet control tools"""
        
        # Robot Status Tool
        robot_status_tool = ToolDefinition(
            name="get_robot_status",
            description="Get current status of a robot in the fleet",
            parameters={
                "required": ["robot_id"],
                "optional": ["include_telemetry", "include_errors"]
            },
            return_type="dict",
            timeout=5,
            category="fleet_control",
            robot_types=["aerial_drone", "ground_crawler", "submarine_drone"],
            requires_auth=True
        )
        self.tool_registry.register_tool(robot_status_tool, self._get_robot_status)
        
        # Robot Command Tool
        robot_command_tool = ToolDefinition(
            name="send_robot_command",
            description="Send a command to a robot in the fleet",
            parameters={
                "required": ["robot_id", "command", "parameters"],
                "optional": ["priority", "timeout"]
            },
            return_type="dict",
            timeout=10,
            category="fleet_control",
            robot_types=["aerial_drone", "ground_crawler", "submarine_drone"],
            requires_auth=True
        )
        self.tool_registry.register_tool(robot_command_tool, self._send_robot_command)
        
        # Fleet Coordination Tool
        fleet_coordination_tool = ToolDefinition(
            name="coordinate_fleet",
            description="Coordinate multiple robots for a task",
            parameters={
                "required": ["task_id", "robot_ids", "task_parameters"],
                "optional": ["coordination_strategy", "priority"]
            },
            return_type="dict",
            timeout=30,
            category="fleet_control",
            robot_types=["aerial_drone", "ground_crawler", "submarine_drone"],
            requires_auth=True
        )
        self.tool_registry.register_tool(fleet_coordination_tool, self._coordinate_fleet)
        
        # Emergency Stop Tool
        emergency_stop_tool = ToolDefinition(
            name="emergency_stop",
            description="Emergency stop for robots or entire fleet",
            parameters={
                "required": ["scope"],
                "optional": ["robot_ids", "reason"]
            },
            return_type="dict",
            timeout=2,
            category="emergency",
            robot_types=["aerial_drone", "ground_crawler", "submarine_drone"],
            requires_auth=True
        )
        self.tool_registry.register_tool(emergency_stop_tool, self._emergency_stop)
        
    async def _get_robot_status(self, parameters: Dict[str, Any], context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Get robot status implementation"""
        robot_id = parameters["robot_id"]
        include_telemetry = parameters.get("include_telemetry", False)
        include_errors = parameters.get("include_errors", False)
        
        # Implementation would connect to robot via ROS 2 or MQTT
        # This is a placeholder implementation
        status = {
            "robot_id": robot_id,
            "status": "active",
            "battery_level": 85.0,
            "position": {"x": 10.5, "y": 20.3, "z": 5.2},
            "timestamp": datetime.utcnow().isoformat()
        }
        
        if include_telemetry:
            status["telemetry"] = {
                "cpu_usage": 45.0,
                "memory_usage": 60.0,
                "temperature": 35.5,
                "network_latency": 12.0
            }
            
        if include_errors:
            status["errors"] = []
            
        return status
        
    async def _send_robot_command(self, parameters: Dict[str, Any], context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Send robot command implementation"""
        robot_id = parameters["robot_id"]
        command = parameters["command"]
        cmd_params = parameters["parameters"]
        priority = parameters.get("priority", "normal")
        timeout = parameters.get("timeout", 30)
        
        # Implementation would send command via ROS 2 or MQTT
        # This is a placeholder implementation
        result = {
            "robot_id": robot_id,
            "command": command,
            "status": "sent",
            "command_id": f"cmd_{datetime.utcnow().timestamp()}",
            "priority": priority,
            "timeout": timeout
        }
        
        return result
        
    async def _coordinate_fleet(self, parameters: Dict[str, Any], context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Fleet coordination implementation"""
        task_id = parameters["task_id"]
        robot_ids = parameters["robot_ids"]
        task_params = parameters["task_parameters"]
        strategy = parameters.get("coordination_strategy", "sequential")
        priority = parameters.get("priority", "normal")
        
        # Implementation would coordinate multiple robots
        # This is a placeholder implementation
        result = {
            "task_id": task_id,
            "robot_ids": robot_ids,
            "coordination_strategy": strategy,
            "status": "coordinated",
            "estimated_duration": 300,  # seconds
            "priority": priority
        }
        
        return result
        
    async def _emergency_stop(self, parameters: Dict[str, Any], context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Emergency stop implementation"""
        scope = parameters["scope"]  # "robot", "fleet", "all"
        robot_ids = parameters.get("robot_ids", [])
        reason = parameters.get("reason", "emergency_stop")
        
        # Implementation would send emergency stop commands
        # This is a placeholder implementation
        result = {
            "scope": scope,
            "robot_ids": robot_ids,
            "reason": reason,
            "status": "emergency_stop_sent",
            "timestamp": datetime.utcnow().isoformat()
        }
        
        return result
```

### 6.4 MCP Client Implementation

```python
# mcp_client.py
import asyncio
import websockets
import json
from typing import Dict, List, Any, Optional
from datetime import datetime

class MCPClient:
    def __init__(self, config_path: str):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.websocket = None
        self.connected = False
        
    async def connect(self):
        """Connect to MCP server"""
        server_url = f"ws://{self.config['mcp_server']['host']}:{self.config['mcp_server']['port']}"
        
        try:
            self.websocket = await websockets.connect(server_url)
            self.connected = True
            print(f"Connected to MCP server at {server_url}")
        except Exception as e:
            print(f"Failed to connect to MCP server: {e}")
            raise
            
    async def discover_tools(self, robot_type: str = None, category: str = None) -> List[Dict[str, Any]]:
        """Discover available tools"""
        if not self.connected:
            await self.connect()
            
        discovery_request = {
            "type": "tool_discovery",
            "robot_type": robot_type,
            "category": category,
            "timestamp": datetime.utcnow().isoformat()
        }
        
        await self.websocket.send(json.dumps(discovery_request))
        response = await self.websocket.recv()
        return json.loads(response)["tools"]
        
    async def call_tool(self, tool_name: str, parameters: Dict[str, Any], 
                       context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Call a tool on the MCP server"""
        if not self.connected:
            await self.connect()
            
        tool_call = {
            "type": "tool_call",
            "tool_name": tool_name,
            "parameters": parameters,
            "context": context or {},
            "timestamp": datetime.utcnow().isoformat()
        }
        
        await self.websocket.send(json.dumps(tool_call))
        response = await self.websocket.recv()
        return json.loads(response)
        
    async def close(self):
        """Close connection to MCP server"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
```

## 7. SCADA Integration with Digital Twins

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

### 7.2 Digital Twin Integration

**Digital Twin Configuration (`config/digital_twin_config.yaml`):**
```yaml
digital_twin:
  platform: "nvidia_omniverse"
  omniverse_url: "http://localhost:8080"
  
  factory_models:
    - name: "factory_a"
      model_path: "/models/factory_a.usd"
      scale: [1.0, 1.0, 1.0]
      position: [0.0, 0.0, 0.0]
      
    - name: "factory_b"
      model_path: "/models/factory_b.usd"
      scale: [1.0, 1.0, 1.0]
      position: [100.0, 0.0, 0.0]
      
  robot_models:
    - name: "scara_arm_001"
      model_path: "/models/scara_arm.usd"
      robot_type: "scara"
      workspace: "factory_a"
      
    - name: "drone_001"
      model_path: "/models/drone.usd"
      robot_type: "aerial_drone"
      workspace: "factory_a"
      
  real_time_sync:
    enabled: true
    update_frequency: 30  # Hz
    data_sources:
      - "scada_tags"
      - "robot_telemetry"
      - "sensor_data"
      
  visualization:
    camera_positions:
      - name: "overview"
        position: [50.0, 50.0, 100.0]
        target: [0.0, 0.0, 0.0]
      - name: "production_line"
        position: [10.0, 5.0, 15.0]
        target: [10.0, 0.0, 0.0]
```

### 7.3 SCADA System Integration

**SCADA Configuration (`config/scada_config.yaml`):**
```yaml
scada_system:
  platform: "ignition_edge"
  version: "8.1.30"
  
  # Ignition Edge Configuration
  ignition:
    host: "localhost"
    port: 8088
    username: "admin"
    password: "admin123"
    ssl_enabled: false
    
  # OPC UA Configuration
  opcua:
    server_endpoint: "opc.tcp://0.0.0.0:4840"
    security_policy: "None"
    security_mode: "None"
    certificate_path: "/certs/opcua_cert.pem"
    private_key_path: "/certs/opcua_key.pem"
    
  # MQTT Bridge Configuration
  mqtt_bridge:
    broker: "192.168.1.100"
    port: 1883
    username: "isaac_nexus"
    password: "password123"
    qos: 1
    retain: true
    
  # Tag Configuration
  tags:
    robot_telemetry:
      - name: "Robot001/Position"
        data_type: "Float3"
        description: "Robot position (x, y, z)"
      - name: "Robot001/Battery"
        data_type: "Float"
        description: "Robot battery level"
      - name: "Robot001/Status"
        data_type: "String"
        description: "Robot operational status"
        
    factory_sensors:
      - name: "Factory/Temperature"
        data_type: "Float"
        description: "Factory ambient temperature"
      - name: "Factory/Humidity"
        data_type: "Float"
        description: "Factory humidity level"
      - name: "Factory/Pressure"
        data_type: "Float"
        description: "Factory atmospheric pressure"
        
    production_data:
      - name: "Production/Line1/Status"
        data_type: "String"
        description: "Production line 1 status"
      - name: "Production/Line1/Count"
        data_type: "Int"
        description: "Production line 1 item count"
      - name: "Production/Line1/Speed"
        data_type: "Float"
        description: "Production line 1 speed"
```

### 7.4 SCARA Robot Integration

**SCARA Robot Configuration (`config/scara_robots.yaml`):**
```yaml
scara_robots:
  - robot_id: "scara_001"
    name: "Assembly Robot 1"
    manufacturer: "FANUC"
    model: "SCARA SR-3iA"
    
    # Mechanical Specifications
    specifications:
      payload: 3.0  # kg
      reach: 400.0  # mm
      repeatability: 0.02  # mm
      max_speed: 1000.0  # mm/s
      degrees_of_freedom: 4
      
    # Workspace Configuration
    workspace:
      x_min: -400.0
      x_max: 400.0
      y_min: -400.0
      y_max: 400.0
      z_min: 0.0
      z_max: 200.0
      
    # Control Configuration
    control:
      controller: "FANUC R-30iB"
      communication: "Ethernet/IP"
      ip_address: "192.168.1.101"
      port: 44818
      
    # Tool Configuration
    tools:
      - name: "gripper_001"
        type: "pneumatic_gripper"
        payload: 2.0
        grip_force: 50.0  # N
      - name: "welder_001"
        type: "arc_welder"
        power: 200.0  # W
        voltage: 24.0  # V
        
    # Safety Configuration
    safety:
      emergency_stop: true
      light_curtain: true
      safety_rated_monitor_stop: true
      safe_operating_space: true
      
  - robot_id: "scara_002"
    name: "Pick and Place Robot"
    manufacturer: "ABB"
    model: "IRB 910SC"
    
    specifications:
      payload: 6.0
      reach: 500.0
      repeatability: 0.01
      max_speed: 1200.0
      degrees_of_freedom: 4
      
    workspace:
      x_min: -500.0
      x_max: 500.0
      y_min: -500.0
      y_max: 500.0
      z_min: 0.0
      z_max: 250.0
      
    control:
      controller: "ABB IRC5"
      communication: "Modbus TCP"
      ip_address: "192.168.1.102"
      port: 502
```

### 7.5 Legacy PLC Integration

**Legacy PLC Configuration (`config/legacy_plc.yaml`):**
```yaml
legacy_plc:
  # Siemens S7-1200 Configuration
  siemens_s7:
    - plc_id: "plc_001"
      name: "Main Production PLC"
      model: "S7-1200"
      ip_address: "192.168.1.201"
      rack: 0
      slot: 1
      
      # Communication Settings
      communication:
        protocol: "S7"
        port: 102
        timeout: 5000  # ms
        retry_count: 3
        
      # Data Blocks
      data_blocks:
        - db_number: 1
          name: "Production_Data"
          size: 100
        - db_number: 2
          name: "Robot_Control"
          size: 50
        - db_number: 3
          name: "Sensor_Data"
          size: 200
          
      # Input/Output Mapping
      io_mapping:
        inputs:
          - address: "I0.0"
            name: "Emergency_Stop"
            data_type: "BOOL"
          - address: "I0.1"
            name: "Start_Button"
            data_type: "BOOL"
          - address: "IW2"
            name: "Temperature_Sensor"
            data_type: "INT"
            
        outputs:
          - address: "Q0.0"
            name: "Conveyor_Motor"
            data_type: "BOOL"
          - address: "Q0.1"
            name: "Warning_Light"
            data_type: "BOOL"
          - address: "QW2"
            name: "Robot_Speed"
            data_type: "INT"
            
  # Allen-Bradley PLC Configuration
  allen_bradley:
    - plc_id: "plc_002"
      name: "Quality Control PLC"
      model: "CompactLogix L32E"
      ip_address: "192.168.1.202"
      
      communication:
        protocol: "Ethernet/IP"
        port: 44818
        timeout: 5000
        
      tags:
        - name: "Production_Count"
          address: "Program:MainProgram.ProductionCount"
          data_type: "DINT"
        - name: "Quality_Status"
          address: "Program:MainProgram.QualityStatus"
          data_type: "BOOL"
        - name: "Conveyor_Speed"
          address: "Program:MainProgram.ConveyorSpeed"
          data_type: "REAL"
```

### 7.6 SCADA-Digital Twin Bridge

**SCADA-Digital Twin Bridge (`src/scada/digital_twin_bridge.py`):**
```python
import asyncio
import json
from typing import Dict, List, Any
from datetime import datetime
import logging

class SCADADigitalTwinBridge:
    def __init__(self, scada_config: Dict[str, Any], digital_twin_config: Dict[str, Any]):
        self.scada_config = scada_config
        self.digital_twin_config = digital_twin_config
        self.logger = logging.getLogger(__name__)
        
        # Initialize connections
        self.scada_client = None
        self.digital_twin_client = None
        self.running = False
        
    async def start(self):
        """Start the SCADA-Digital Twin bridge"""
        self.running = True
        
        # Initialize SCADA connection
        await self._init_scada_connection()
        
        # Initialize Digital Twin connection
        await self._init_digital_twin_connection()
        
        # Start data synchronization
        await self._start_data_sync()
        
    async def _init_scada_connection(self):
        """Initialize SCADA system connection"""
        # Implementation for connecting to Ignition Edge
        self.logger.info("Connecting to SCADA system...")
        # Placeholder for actual SCADA connection
        
    async def _init_digital_twin_connection(self):
        """Initialize Digital Twin connection"""
        # Implementation for connecting to NVIDIA Omniverse
        self.logger.info("Connecting to Digital Twin platform...")
        # Placeholder for actual Digital Twin connection
        
    async def _start_data_sync(self):
        """Start real-time data synchronization"""
        while self.running:
            try:
                # Read data from SCADA
                scada_data = await self._read_scada_data()
                
                # Update Digital Twin
                await self._update_digital_twin(scada_data)
                
                # Read Digital Twin state
                twin_data = await self._read_digital_twin_state()
                
                # Update SCADA if needed
                await self._update_scada_from_twin(twin_data)
                
                await asyncio.sleep(1.0 / self.digital_twin_config['real_time_sync']['update_frequency'])
                
            except Exception as e:
                self.logger.error(f"Error in data synchronization: {e}")
                await asyncio.sleep(1.0)
                
    async def _read_scada_data(self) -> Dict[str, Any]:
        """Read data from SCADA system"""
        # Implementation for reading SCADA tags
        return {
            "timestamp": datetime.utcnow().isoformat(),
            "robot_positions": {},
            "factory_sensors": {},
            "production_data": {}
        }
        
    async def _update_digital_twin(self, data: Dict[str, Any]):
        """Update Digital Twin with SCADA data"""
        # Implementation for updating Digital Twin models
        self.logger.debug(f"Updating Digital Twin with data: {data}")
        
    async def _read_digital_twin_state(self) -> Dict[str, Any]:
        """Read current state from Digital Twin"""
        # Implementation for reading Digital Twin state
        return {
            "timestamp": datetime.utcnow().isoformat(),
            "simulation_state": "running",
            "robot_states": {},
            "factory_state": {}
        }
        
    async def _update_scada_from_twin(self, data: Dict[str, Any]):
        """Update SCADA system from Digital Twin state"""
        # Implementation for updating SCADA from Digital Twin
        self.logger.debug(f"Updating SCADA from Digital Twin: {data}")
```

### 7.7 OPC UA Server with Digital Twin Integration

```python
# opcua_server.py
import asyncio
from opcua import Server, ua
from opcua.common.node import Node
from typing import Dict, List, Any

class IsaacNexusOPCUAServer:
    def __init__(self, endpoint="opc.tcp://0.0.0.0:4840"):
        self.server = Server()
        self.server.set_endpoint(endpoint)
        self.server.set_server_name("Isaac-Nexus OPC UA Server")
        
        # Setup namespace
        self.namespace = self.server.register_namespace("IsaacNexus")
        
        # Digital Twin integration
        self.digital_twin_bridge = None
        
    async def start(self):
        """Start OPC UA server"""
        await self.server.start()
        
        # Create factory nodes
        self.factory_nodes = {}
        for factory in ["factory_a", "factory_b"]:
            self.factory_nodes[factory] = await self._create_factory_node(factory)
            
        # Create robot nodes
        self.robot_nodes = {}
        for robot_id in ["drone_001", "crawler_001", "rov_001", "scara_001", "scara_002"]:
            self.robot_nodes[robot_id] = await self._create_robot_node(robot_id)
            
        # Create PLC nodes
        self.plc_nodes = {}
        for plc_id in ["plc_001", "plc_002"]:
            self.plc_nodes[plc_id] = await self._create_plc_node(plc_id)
            
    async def _create_factory_node(self, factory_id):
        """Create OPC UA node for factory"""
        factory_node = self.server.nodes.objects.add_object(self.namespace, factory_id)
        
        # Add environmental sensors
        sensors_node = factory_node.add_object(self.namespace, "Sensors")
        sensors_node.add_variable(self.namespace, "Temperature", 20.0)
        sensors_node.add_variable(self.namespace, "Humidity", 50.0)
        sensors_node.add_variable(self.namespace, "Pressure", 1013.25)
        
        # Add production data
        production_node = factory_node.add_object(self.namespace, "Production")
        production_node.add_variable(self.namespace, "Line1_Status", "Running")
        production_node.add_variable(self.namespace, "Line1_Count", 0)
        production_node.add_variable(self.namespace, "Line1_Speed", 1.0)
        
        # Add digital twin status
        digital_twin_node = factory_node.add_object(self.namespace, "DigitalTwin")
        digital_twin_node.add_variable(self.namespace, "Sync_Status", "Active")
        digital_twin_node.add_variable(self.namespace, "Last_Update", "")
        
        return factory_node
        
    async def _create_robot_node(self, robot_id):
        """Create OPC UA node for robot"""
        robot_node = self.server.nodes.objects.add_object(self.namespace, robot_id)
        
        # Add telemetry variables
        telemetry_node = robot_node.add_object(self.namespace, "Telemetry")
        telemetry_node.add_variable(self.namespace, "Position", [0.0, 0.0, 0.0])
        telemetry_node.add_variable(self.namespace, "Battery", 100.0)
        telemetry_node.add_variable(self.namespace, "Status", "Idle")
        telemetry_node.add_variable(self.namespace, "Temperature", 25.0)
        
        # Add command variables
        commands_node = robot_node.add_object(self.namespace, "Commands")
        commands_node.add_variable(self.namespace, "Takeoff", False)
        commands_node.add_variable(self.namespace, "Land", False)
        commands_node.add_variable(self.namespace, "EmergencyStop", False)
        commands_node.add_variable(self.namespace, "MoveTo", [0.0, 0.0, 0.0])
        
        # Add SCARA-specific variables
        if robot_id.startswith("scara"):
            scara_node = robot_node.add_object(self.namespace, "SCARA")
            scara_node.add_variable(self.namespace, "Joint_Angles", [0.0, 0.0, 0.0, 0.0])
            scara_node.add_variable(self.namespace, "Tool_Position", [0.0, 0.0, 0.0])
            scara_node.add_variable(self.namespace, "Gripper_Open", False)
            scara_node.add_variable(self.namespace, "Gripper_Force", 0.0)
        
        return robot_node
        
    async def _create_plc_node(self, plc_id):
        """Create OPC UA node for PLC"""
        plc_node = self.server.nodes.objects.add_object(self.namespace, plc_id)
        
        # Add PLC status
        status_node = plc_node.add_object(self.namespace, "Status")
        status_node.add_variable(self.namespace, "Connected", False)
        status_node.add_variable(self.namespace, "Last_Scan", 0)
        status_node.add_variable(self.namespace, "Error_Count", 0)
        
        # Add I/O data
        io_node = plc_node.add_object(self.namespace, "IO")
        io_node.add_variable(self.namespace, "Inputs", {})
        io_node.add_variable(self.namespace, "Outputs", {})
        
        return plc_node
        
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
            
            # Update SCARA-specific data
            if robot_id.startswith("scara") and "joint_angles" in telemetry_data:
                scara_node = robot_node.get_child(f"{self.namespace}:SCARA")
                joint_angles_var = scara_node.get_child(f"{self.namespace}:Joint_Angles")
                joint_angles_var.set_value(telemetry_data["joint_angles"])
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

