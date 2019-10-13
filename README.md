# DED_Quad
Autonomous Navigation and Obstacle Avoidance algorithm for Quadcopter in GPS-Denied environments. 
Includes both custom built Python and Gazebo simulation environments and the real ROS based implementation.

By Pablo Hermoso Moreno

Project Abstract:

As part of a project aimed at building a tethered quadcopter capable of autonomous navigation and perching in unexplored environments, this paper focuses on the conserations and solutions to some of the most challenging aspects of robot navigation, namely: sensing, acting, planning, communication architectures, hardware, computational efficiencies and problem solving. 
A comparative study of path planning algorithms was conducted and Artificial Potential Fields was selected due to its mathematical elegance, implementation simplicity and computational efficiency, which allow real-time obstacle avoidance without prior knowledge of the environment using exclusively on-board computations and Time-of-Flight sensors. 
Simulated Annealing was employed to solve one of the inherent shortcomings of such approach; Local Minima.
Effectiveness of the proposed algorithm was validated on custom built Python and Gazebo simulation platforms before demonstrating its potential in a real aerial platform using Robot Operating System (ROS) based communication. 
Full integration of hardware, the robustness of the Simulated Annelaing algorithm and a necessity to further optimise the Potential Field parameters were identified as key areas for improvement. 
Lastly, this paper outlines the need for a laser range data segmentation and feature extraction algorithm for a more rigorous implementation of Potential Fields in structured environments and discusses the challenges faced in the future implementation of a Simultaneous Localisation and Mapping (SLAM) algorithm.


Branches Inlcude:
- Differential Evolution Optimisation
- Final Python Worlds
- Gazebo Simulations Script
- Gazebo Simulations Setup Files
- Real Flight Test Scripts
- Report
