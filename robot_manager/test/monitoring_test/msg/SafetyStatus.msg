# speed sent to the robot after testing for safety, positive forward, m/s
float32 safeSpeed

# Obstacle status:
# 0: OK
# 1: danger
# 2: critical
int8 obstacle

# Obstacle distance (m)
# This is only valid if there was an obstacle (see obstacle).
float32 obstacleDistance

# Obstacle bearing (rad, positive to the right)
# This is only valid if there was an obstacle (see obstacle).
float32 obstacleBearing

# Avoidance mode:
# 0: Only speed is controlled, and reduced if the robot is getting too close to an obstacle
# 1: The steering is also changed from the input steering to turn away from obstacles
# 2: The steering is fully controlled to turn away from obstacles.
# 99: Unguarded, the steering is not touched and the speed is set to a very low speed.
int8 avoidanceMode

# Laser data:
# 0: OK
# 1: late
# 2: missing
int8 laserStatus
