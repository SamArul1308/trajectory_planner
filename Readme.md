# trajectory_planner

## Description
The Trajectory Planner is a important component for autonomous vehicle navigation. It computes a path for the vehicle to follow based on a predefined set of waypoints and dynamically adjusts steering and speed commands to ensure smooth motion. Additionally, it incorporates obstacle detection to halt vehicle movement when necessary, ensuring safety during navigation.

##  Criteria 6: Host vehicle is able to drive on curved lanes
### User Story  M5.6.1 : Navigation on Curved Lanes
As the trajectory planning component of the host vehicle,
I want the vehicle to navigate curved lanes by following a predefined set of waypoints,so that the host vehicle can travel along curved paths.

### Acceptance Criteria:
1. [x] The trajectory planning component must generate a path from a predefined list of hardcoded waypoints.
2. [x] The system must accurately identify the current position of the vehicle using the /ego_pose5 topic and map it to the nearest waypoint.
3. [x] The trajectory planner must compute and publish Ackermann steering commands (/ackermann_drive) based on the next waypoint in the path.

## Module 6: 
#### Sam Schwartz Arul Agastus
### User Story M6.3: The Follower Vehicle follows predefined route
As a developer, I want the follower vehicle to follow the lead vehicle along predefined routes, so that it stays on the designated road and avoids obstacles.
### Acceptance Criteria:
1. [x] Path Following: The vehicle follows a predefined path using waypoints instead of heading directly to the lead vehicle.
2. [x] Waypoint Tracking: The system assigns the closest waypoint to the follower and updates it dynamically.
3. [x] Lead Vehicle Handling: The vehicle stops within 0.5m of the lead vehicle’s waypoint.

### User Story  M6.6 : Lead Vehicle Coordination based on left, right and straight movement
As the follower vehicle, I want to coordinate with the lead vehicle position by following its left, right, and straight movements, so that I can maintain a safe distance and stay aligned with the lead vehicle's trajectory.
### Acceptance Criteria:

1. [x] The system tracks the lead vehicle's position using the /other_vehicle_pose topic.The system updates the target waypoint based on the lead vehicle's position.
2. [x] If no updates from the lead vehicle are received for more than 1 second, the follower vehicle stops.
3. [x] The system logs updates on the lead vehicle's position for monitoring and debugging.
