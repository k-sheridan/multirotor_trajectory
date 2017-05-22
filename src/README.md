# virtual-flight-controller
This simulates a quadrotor with actuator and sensor noise. Accepts force commands.

This is meant to be used with ROS. 

Publishes Pose and Twist messages with or without state estimate noise.

Accepts an array message.

[M1 +x -y]

[M2 -x -y]

[M3 -x +y]

[M4 +x +y]
