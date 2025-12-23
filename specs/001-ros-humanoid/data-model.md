# Data Model: ROS 2 Humanoid Robot Module

## Entities

### ROS 2 Node
**Description**: Communication entity that publishes/subscribe to topics or provides/request services, representing different robot subsystems
**Attributes**:
- node_name: string (unique identifier for the node)
- namespace: string (optional namespace for the node)
- parameters: dictionary (configuration parameters)
- publishers: list (topics this node publishes to)
- subscribers: list (topics this node subscribes to)
- services: list (services this node provides)
- clients: list (services this node calls)

**Relationships**:
- One-to-many with ROS 2 Topic (publishes/subscribes to)
- One-to-many with ROS 2 Service (provides/calls)

### ROS 2 Topic
**Description**: Communication channel for real-time data flow between nodes
**Attributes**:
- topic_name: string (unique identifier for the topic)
- message_type: string (ROS 2 message type)
- qos_profile: object (quality of service settings)
- publishers: list (nodes that publish to this topic)
- subscribers: list (nodes that subscribe to this topic)

### ROS 2 Service
**Description**: Request-response communication pattern for synchronous operations
**Attributes**:
- service_name: string (unique identifier for the service)
- request_type: string (request message type)
- response_type: string (response message type)
- server: string (node providing the service)
- clients: list (nodes calling this service)

### URDF Model
**Description**: XML-based representation of robot physical structure including links, joints, and their relationships for simulation and control
**Attributes**:
- robot_name: string (name of the robot)
- links: list (physical components of the robot)
- joints: list (connections between links)
- materials: list (visual materials)
- transmissions: list (actuator connections)
- gazebo_extensions: list (simulation-specific extensions)

### URDF Link
**Description**: Physical component of the robot (e.g., torso, thigh, shin, foot)
**Attributes**:
- link_name: string (unique identifier for the link)
- visual: object (visual representation)
- collision: object (collision properties)
- inertial: object (mass, center of mass, inertia)
- parent_joint: string (reference to connecting joint)

### URDF Joint
**Description**: Connection between two links allowing relative motion
**Attributes**:
- joint_name: string (unique identifier for the joint)
- joint_type: string (revolute, prismatic, fixed, etc.)
- parent_link: string (link the joint connects from)
- child_link: string (link the joint connects to)
- origin: object (position and orientation of joint)
- axis: object (rotation or translation axis)
- limits: object (min/max values for joint movement)

### Python Agent
**Description**: High-level AI logic written in Python that interfaces with ROS controllers
**Attributes**:
- agent_name: string (unique identifier for the agent)
- rclpy_node: object (ROS 2 node interface)
- input_topics: list (topics the agent subscribes to)
- output_topics: list (topics the agent publishes to)
- services_used: list (services the agent calls)
- ai_model: object (underlying AI model or logic)

## State Transitions

### ROS 2 Node States
- Unconfigured → Inactive: After node creation
- Inactive → Active: After configuration and activation
- Active → Inactive: After deactivation
- Any state → Finalized: When node is destroyed

### Robot Joint States
- Initialization → Ready: After calibration and homing
- Ready → Moving: When joint commands are issued
- Moving → Ready: When target position is reached
- Ready → Error: When joint limits are exceeded or hardware fault occurs

## Validation Rules

### From Functional Requirements
- **FR-001**: ROS 2 Node MUST have valid node_name following ROS naming conventions
- **FR-002**: ROS 2 Topic MUST have valid message_type from ROS 2 message packages
- **FR-003**: URDF Model MUST contain at least one link and one joint for a valid robot
- **FR-004**: URDF Joint MUST have valid parent_link and child_link references
- **FR-005**: URDF Joint limits MUST be within physical constraints of the real robot
- **FR-006**: Python Agent MUST successfully connect to ROS controllers via rclpy
- **FR-007**: All URDF elements MUST pass validation with check_urdf tool

## Relationships Summary

- ROS 2 Nodes communicate through Topics and Services
- URDF Links are connected by Joints to form the robot kinematic structure
- Python Agents interface with ROS 2 Nodes to bridge AI logic with robot control
- URDF Models define the physical structure used in simulation and control