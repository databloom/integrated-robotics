# Isaac-Nexus Hardware Bill of Materials (BOM)

## Overview

This document provides comprehensive hardware specifications and cost analysis for the Isaac-Nexus robotics platform, including aerial drones, ground crawlers, submarine drones, and SCADA integration components.

## 1. Aerial Drone Platform

### 1.1 Core Components

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Flight Controller** | ESP32-S3-WROOM-1 (Dual-core, Wi-Fi/BLE) | 1 | $8.00 | $8.00 | Espressif | ESP32-S3-WROOM-1 |
| **Compute Module** | Raspberry Pi CM4 (2GB RAM, 16GB eMMC) | 1 | $35.00 | $35.00 | Raspberry Pi | CM4002000 |
| **Power Distribution** | 4-in-1 30A ESC with BEC | 1 | $45.00 | $45.00 | T-Motor | F55A Pro II |
| **Motors** | T-Motor MN3515 400KV Brushless | 4 | $25.00 | $100.00 | T-Motor | MN3515-400KV |
| **Propellers** | 1045 Carbon Fiber (CW/CCW) | 4 | $3.50 | $14.00 | T-Motor | T1045C |
| **Battery** | 6S 10,000mAh LiPo | 1 | $120.00 | $120.00 | Tattu | R-Line 6S 10000mAh |
| **Frame** | 600mm Carbon Fiber Quadcopter | 1 | $85.00 | $85.00 | Custom | CF-600-QD |

### 1.2 Sensors & Peripherals

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **LIDAR** | Velodyne Puck LITE | 1 | $4,000.00 | $4,000.00 | Velodyne | VLP-16 |
| **Depth Camera** | Intel RealSense D455 | 1 | $200.00 | $200.00 | Intel | D455 |
| **IMU** | MPU-6050 6-Axis | 1 | $3.00 | $3.00 | InvenSense | MPU-6050 |
| **Barometer** | BMP280 | 1 | $5.00 | $5.00 | Bosch | BMP280 |
| **GPS** | NEO-6M GPS Module | 1 | $15.00 | $15.00 | u-blox | NEO-6M |
| **Telemetry** | 915MHz 500mW Radio | 1 | $25.00 | $25.00 | RFD | 900uS |
| **Camera** | Raspberry Pi Camera V3 | 1 | $25.00 | $25.00 | Raspberry Pi | IMX708 |

**Total Aerial Drone Cost: ~$4,640.00**

## 2. Ground Crawler Platform

### 2.1 Core Components

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Main Controller** | ESP32-S3-WROOM-1 | 1 | $8.00 | $8.00 | Espressif | ESP32-S3-WROOM-1 |
| **Compute Module** | Raspberry Pi Zero 2 W (1GB RAM) | 1 | $15.00 | $15.00 | Raspberry Pi | SC0915 |
| **Motor Driver** | L298N Dual H-Bridge | 2 | $5.00 | $10.00 | STMicroelectronics | L298N |
| **Motors** | 12V 200RPM Gear Motor | 4 | $12.00 | $48.00 | Pololu | 37Dx52L |
| **Wheels** | 65mm Mecanum Wheels | 4 | $8.00 | $32.00 | Pololu | 65mm Mecanum |
| **Battery** | 3S 11.1V 5000mAh LiPo | 1 | $30.00 | $30.00 | Tattu | 3S 5000mAh |
| **Chassis** | Aluminum Robot Platform | 1 | $45.00 | $45.00 | Custom | AL-400x300 |

### 2.2 Sensors & Peripherals

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **IMU** | MPU-6050 | 1 | $3.00 | $3.00 | InvenSense | MPU-6050 |
| **Ultrasonic** | HC-SR04 | 4 | $2.50 | $10.00 | Generic | HC-SR04 |
| **Camera** | Raspberry Pi Camera V2 | 1 | $25.00 | $25.00 | Raspberry Pi | IMX219 |
| **LIDAR** | RPLIDAR A1M8 | 1 | $100.00 | $100.00 | Slamtec | RPLIDAR A1M8 |
| **Manipulator** | 6DOF Robotic Arm | 1 | $120.00 | $120.00 | UFactory | xArm 6 |
| **Thermal Camera** | FLIR Lepton 3.5 | 1 | $200.00 | $200.00 | FLIR | Lepton 3.5 |

**Total Ground Crawler Cost: ~$647.00**

## 3. Submarine Drone Platform (ROV)

### 3.1 Core Components

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Main Controller** | ESP32-S3-WROOM-1 (Waterproof) | 1 | $15.00 | $15.00 | Espressif | ESP32-S3-WROOM-1 |
| **Compute Module** | Raspberry Pi 4 (4GB RAM, Industrial) | 1 | $75.00 | $75.00 | Raspberry Pi | RPI4-MODBP-4GB |
| **Pressure Housing** | 100m Depth Rated Acrylic Tube | 1 | $300.00 | $300.00 | Custom | PH-100M-150MM |
| **Thrusters** | T200 Blue Robotics | 6 | $150.00 | $900.00 | Blue Robotics | T200 |
| **Thruster ESCs** | Basic ESC 30A | 6 | $25.00 | $150.00 | Blue Robotics | Basic ESC 30A |
| **Power System** | 6S 10,000mAh Li-Ion (Pressure Compensated) | 1 | $450.00 | $450.00 | Custom | PC-6S-10AH |
| **Buoyancy Foam** | Syntactic Foam | 1 | $120.00 | $120.00 | Custom | SF-100M |
| **Frame** | Anodized Aluminum | 1 | $250.00 | $250.00 | Custom | AL-ROV-FRAME |

### 3.2 Sensors & Navigation

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Depth Sensor** | MS5837-30BA | 1 | $45.00 | $45.00 | Measurement Specialties | MS5837-30BA |
| **IMU** | 9-DOF (BNO085) | 1 | $35.00 | $35.00 | Bosch | BNO085 |
| **DVL** | Water Linked DVL A50 | 1 | $1,200.00 | $1,200.00 | Water Linked | DVL A50 |
| **Sonar** | Blueprint Oculus M750d | 1 | $3,500.00 | $3,500.00 | Blueprint | Oculus M750d |
| **Underwater Camera** | 4K Low-Light w/ LED Lights | 1 | $600.00 | $600.00 | Custom | UW-CAM-4K |
| **Leak Detector** | Optical Water Sensor | 4 | $15.00 | $60.00 | Custom | OWS-001 |

### 3.3 Communication & Control

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Tether** | 100m Neutrally Buoyant | 1 | $800.00 | $800.00 | Custom | TB-100M-NB |
| **Tether Interface** | Gigabit Ethernet | 1 | $150.00 | $150.00 | Custom | TI-GBE |
| **Acoustic Modem** | Water Linked Modem M64 | 1 | $1,200.00 | $1,200.00 | Water Linked | Modem M64 |
| **Surface Buoy** | Wi-Fi/4G Gateway | 1 | $350.00 | $350.00 | Custom | SB-WIFI-4G |

### 3.4 Manipulation & Tools

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Robotic Arm** | 4DOF Underwater Arm | 1 | $1,200.00 | $1,200.00 | Custom | UW-ARM-4DOF |
| **Tool Changer** | Quick Release | 1 | $300.00 | $300.00 | Custom | TC-QR |
| **Sampling Bottle** | Niskin-Type | 2 | $120.00 | $240.00 | Custom | SB-NISKIN |
| **Magnetometer** | TCM-XB3 | 1 | $280.00 | $280.00 | PNI | TCM-XB3 |

**Total Submarine Drone Cost: ~$12,242.00**

## 4. SCADA Integration Components

### 4.1 Central Server

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Server** | Dell PowerEdge R350 | 1 | $1,200.00 | $1,200.00 | Dell | PowerEdge R350 |
| **Switch** | Cisco Catalyst 1000 | 1 | $500.00 | $500.00 | Cisco | WS-C1000-24P |
| **UPS** | APC 1500VA | 1 | $200.00 | $200.00 | APC | SMT1500RM2U |

### 4.2 Field Components

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **PLC** | Siemens S7-1200 | 1 | $600.00 | $600.00 | Siemens | 6ES7 212-1AE40-0XB0 |
| **HMI** | Weintek MT8071iE | 1 | $350.00 | $350.00 | Weintek | MT8071iE |
| **RTU** | Moxa ioLogik 2542 | 2 | $450.00 | $900.00 | Moxa | ioLogik 2542 |
| **Ethernet Gateway** | MQTT-SN Gateway | 1 | $120.00 | $120.00 | Custom | MQTT-SN-GW |

### 4.3 Software

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **SCADA** | Ignition Edge | 1 | $1,200.00 | $1,200.00 | Inductive Automation | Ignition Edge |
| **MQTT Broker** | HiveMQ | 1 | $0.00 | $0.00 | Open Source | HiveMQ CE |
| **Database** | InfluxDB | 1 | $0.00 | $0.00 | Open Source | InfluxDB OSS |

**Total SCADA System Cost: ~$5,070.00**

## 5. Development & Testing Tools

| Item | Description | Qty | Unit Price (USD) | Total (USD) | Vendor | Part Number |
|------|-------------|-----|-----------------|-------------|--------|-------------|
| **Oscilloscope** | Rigol DS1104Z | 1 | $400.00 | $400.00 | Rigol | DS1104Z |
| **Logic Analyzer** | Saleae Logic 8 | 1 | $250.00 | $250.00 | Saleae | Logic 8 |
| **Power Supply** | Adjustable DC 30V/5A | 1 | $80.00 | $80.00 | Generic | PS-30V-5A |
| **Multimeter** | Fluke 115 | 2 | $150.00 | $300.00 | Fluke | 115 |

**Total Tools Cost: ~$1,030.00**

## 6. Total System Costs

| Platform | Quantity | Unit Cost (USD) | Total (USD) |
|----------|----------|-----------------|-------------|
| Aerial Drone | 3 | $4,640.00 | $13,920.00 |
| Ground Crawler | 3 | $647.00 | $1,941.00 |
| Submarine Drone | 1 | $12,242.00 | $12,242.00 |
| SCADA System | 1 | $5,070.00 | $5,070.00 |
| Development Tools | 1 | $1,030.00 | $1,030.00 |
| **Grand Total** | | | **$34,203.00** |

## 7. Integration Notes

### 7.1 Communication
- All robots use MQTT over Wi-Fi for command and control
- ESP32 handles real-time control loops
- Raspberry Pi runs ROS 2 nodes and Hammerspace client
- 5G/Wi-Fi 6 for high-bandwidth applications

### 7.2 SCADA Integration
- OPC UA server on Raspberry Pi
- MQTT bridge to Ignition SCADA
- Real-time monitoring of all robot parameters
- Historical data logging and analysis

### 7.3 Power Management
- Smart charging station integration
- Battery monitoring via SCADA
- Automated battery swap for continuous operation
- Power consumption optimization

### 7.4 Safety Systems
- Emergency stop on all platforms
- Geofencing for aerial drones
- Collision avoidance system
- Fail-safe communication protocols

## 8. Procurement Timeline

### Phase 1 (Weeks 1-2): Core Components
- ESP32-S3 controllers
- Raspberry Pi modules
- Basic sensors and actuators
- Development tools

### Phase 2 (Weeks 3-4): Advanced Sensors
- LIDAR systems
- Cameras and vision systems
- Specialized sensors (DVL, sonar)
- Communication equipment

### Phase 3 (Weeks 5-6): Integration Components
- SCADA hardware
- Network equipment
- Power systems
- Safety equipment

### Phase 4 (Weeks 7-8): Testing and Validation
- System integration testing
- Performance validation
- Safety certification
- Documentation completion

## 9. Vendor Information

### Primary Vendors
- **Espressif Systems**: ESP32 controllers and modules
- **Raspberry Pi Foundation**: Compute modules and accessories
- **T-Motor**: Motors, ESCs, and propellers
- **Blue Robotics**: Underwater thrusters and sensors
- **Velodyne**: LIDAR systems
- **Intel**: Depth cameras and vision systems

### Secondary Vendors
- **Pololu**: Motors, wheels, and mechanical components
- **Slamtec**: 2D LIDAR systems
- **Water Linked**: Underwater navigation and communication
- **Dell**: Server hardware
- **Cisco**: Network equipment
- **Siemens**: Industrial automation components

## 10. Cost Optimization Strategies

### 10.1 Volume Discounts
- Negotiate bulk pricing for quantities >10 units
- Establish preferred vendor relationships
- Consider alternative suppliers for non-critical components

### 10.2 Open Source Alternatives
- Use open-source software where possible
- Consider 3D printing for custom enclosures
- Leverage community-developed solutions

### 10.3 Phased Procurement
- Start with minimum viable product (MVP)
- Add advanced features incrementally
- Validate performance before full deployment

---

*Document Version: 1.0*  
*Last Updated: January 2024*  
*Authors: Isaac-Nexus Hardware Team*

