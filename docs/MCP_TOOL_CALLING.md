# MCP Tool Calling Documentation

## Overview

The Model Context Protocol (MCP) enables agents to discover, call, and coordinate tools across the Isaac-Nexus robotic fleet. This document provides comprehensive guidance on implementing and using MCP for agent-based fleet control.

## Architecture

### MCP Server
- Central registry for tool discovery and execution
- WebSocket-based communication with agents
- OAuth2 JWT authentication and RBAC authorization
- Real-time tool execution and monitoring

### MCP Agents
- Autonomous agents that can discover and call tools
- Context-aware tool selection and execution
- Multi-agent coordination and task delegation
- Emergency response and fleet management

### Tool Registry
- Dynamic tool registration and discovery
- Parameter validation and type checking
- Tool execution with timeout and error handling
- Tool chaining and workflow orchestration

## Tool Categories

### Fleet Control Tools
- `get_robot_status`: Get current status of any robot
- `send_robot_command`: Send commands to specific robots
- `coordinate_fleet`: Coordinate multiple robots for tasks
- `emergency_stop`: Emergency stop for robots or fleet

### Robot Management Tools
- `deploy_robot`: Deploy a robot to a specific location
- `configure_robot`: Configure robot parameters
- `update_robot_firmware`: Update robot software
- `calibrate_robot`: Calibrate robot sensors and actuators

### Task Coordination Tools
- `create_task`: Create a new task for the fleet
- `assign_task`: Assign task to specific robots
- `monitor_task`: Monitor task progress
- `complete_task`: Mark task as completed

### Emergency Response Tools
- `emergency_stop`: Stop all robots immediately
- `evacuate_area`: Evacuate robots from specific area
- `emergency_override`: Override normal operations
- `alert_operators`: Send alerts to human operators

## Usage Examples

### Basic Tool Calling
```python
# Initialize MCP agent
agent = MCPAgent("fleet_manager_001", config)

# Get status of a specific robot
status = await agent.call_tool("get_robot_status", {
    "robot_id": "drone_001",
    "include_telemetry": True
})

# Send command to robot
result = await agent.call_tool("send_robot_command", {
    "robot_id": "drone_001",
    "command": "takeoff",
    "parameters": {"altitude": 10.0}
})
```

### Fleet Coordination
```python
# Coordinate multiple robots for a task
coordination_result = await agent.call_tool("coordinate_fleet", {
    "task_id": "inspection_task_001",
    "robot_ids": ["drone_001", "crawler_001"],
    "task_parameters": {
        "area": "building_a",
        "duration": 3600
    },
    "coordination_strategy": "parallel"
})
```

### Emergency Response
```python
# Emergency stop all robots
emergency_result = await agent.call_tool("emergency_stop", {
    "scope": "fleet",
    "reason": "safety_violation"
})
```

## Security

### Authentication
- OAuth2 JWT tokens for agent authentication
- Token refresh and rotation
- Multi-factor authentication support

### Authorization
- Role-based access control (RBAC)
- Tool-level permissions
- Robot-specific access controls

### Encryption
- TLS 1.3 for all communications
- End-to-end encryption for sensitive data
- Key management via HSM

## Monitoring and Logging

### Tool Execution Monitoring
- Real-time tool execution status
- Performance metrics and latency tracking
- Error rates and success rates

### Audit Logging
- Immutable audit logs for all tool calls
- Security event logging
- Compliance reporting

## Best Practices

### Tool Design
- Design tools to be stateless and idempotent
- Use clear, descriptive tool names
- Provide comprehensive parameter validation
- Implement proper error handling

### Agent Coordination
- Use context-aware tool selection
- Implement proper task delegation
- Handle agent failures gracefully
- Monitor agent health and performance

### Security
- Always validate tool parameters
- Use least-privilege access
- Monitor for suspicious activity
- Regular security audits

## Troubleshooting

### Common Issues
- Tool execution timeouts
- Agent connection failures
- Authentication errors
- Tool parameter validation errors

### Debugging
- Enable detailed logging
- Use MCP server monitoring tools
- Check agent health status
- Verify tool registry status

## API Reference

### Tool Definition Schema
```json
{
  "name": "string",
  "description": "string",
  "parameters": {
    "required": ["string"],
    "optional": ["string"]
  },
  "return_type": "string",
  "timeout": "number",
  "category": "string",
  "robot_types": ["string"],
  "requires_auth": "boolean"
}
```

### Tool Call Request Schema
```json
{
  "type": "tool_call",
  "tool_name": "string",
  "parameters": "object",
  "context": "object",
  "timestamp": "string"
}
```

### Tool Call Response Schema
```json
{
  "success": "boolean",
  "result": "object",
  "error": "string",
  "tool_name": "string",
  "timestamp": "string"
}
```

## Integration Examples

### ROS 2 Integration
```python
# ROS 2 node integration with MCP
import rclpy
from rclpy.node import Node
from mcp.mcp_client import MCPClient

class ROS2MCPBridge(Node):
    def __init__(self):
        super().__init__('ros2_mcp_bridge')
        self.mcp_client = MCPClient('config/mcp_client_config.yaml')
        
    async def handle_robot_command(self, msg):
        # Convert ROS 2 message to MCP tool call
        result = await self.mcp_client.call_tool("send_robot_command", {
            "robot_id": msg.robot_id,
            "command": msg.command,
            "parameters": msg.parameters
        })
        
        # Publish result back to ROS 2
        self.publish_result(result)
```

### MQTT Integration
```python
# MQTT integration with MCP
import asyncio
import json
from mcp.mcp_client import MCPClient

class MQTTMCPBridge:
    def __init__(self):
        self.mcp_client = MCPClient('config/mcp_client_config.yaml')
        self.mqtt_client = None
        
    async def on_mqtt_message(self, topic, payload):
        # Parse MQTT message
        message = json.loads(payload)
        
        # Call MCP tool
        result = await self.mcp_client.call_tool(
            message['tool_name'],
            message['parameters']
        )
        
        # Publish result to MQTT
        await self.publish_result(topic, result)
```

## Performance Optimization

### Tool Caching
- Cache frequently used tools
- Implement tool result caching
- Use connection pooling

### Load Balancing
- Distribute tool calls across multiple servers
- Implement failover mechanisms
- Monitor server load

### Monitoring
- Track tool execution metrics
- Monitor agent performance
- Alert on system issues

## Deployment

### Docker Deployment
```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY src/ ./src/
COPY config/ ./config/

CMD ["python", "src/mcp/mcp_server.py"]
```

### Kubernetes Deployment
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: mcp-server
spec:
  replicas: 3
  selector:
    matchLabels:
      app: mcp-server
  template:
    metadata:
      labels:
        app: mcp-server
    spec:
      containers:
      - name: mcp-server
        image: isaac-nexus/mcp-server:latest
        ports:
        - containerPort: 8000
        env:
        - name: MCP_CONFIG_PATH
          value: "/config/mcp_server_config.yaml"
```

## Testing

### Unit Tests
```python
import pytest
from mcp.tool_registry import MCPToolRegistry, ToolDefinition

def test_tool_registration():
    registry = MCPToolRegistry()
    tool_def = ToolDefinition(
        name="test_tool",
        description="Test tool",
        parameters={"required": ["param1"]},
        return_type="string",
        timeout=5,
        category="test",
        robot_types=["test_robot"],
        requires_auth=False
    )
    
    registry.register_tool(tool_def, lambda p, c: "test_result")
    assert "test_tool" in registry.tools
```

### Integration Tests
```python
import pytest
import asyncio
from mcp.mcp_client import MCPClient

@pytest.mark.asyncio
async def test_tool_call():
    client = MCPClient('config/test_mcp_config.yaml')
    await client.connect()
    
    result = await client.call_tool("get_robot_status", {
        "robot_id": "test_robot"
    })
    
    assert result["success"] == True
    await client.close()
```

## Conclusion

The MCP tool calling system provides a powerful framework for agent-based fleet control in the Isaac-Nexus platform. By following the guidelines and examples in this documentation, developers can effectively implement and use MCP for robotic fleet management.

For additional support and examples, refer to the Isaac-Nexus GitHub repository and community forums.
