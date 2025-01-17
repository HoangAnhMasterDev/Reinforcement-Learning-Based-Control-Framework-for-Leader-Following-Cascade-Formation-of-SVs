# Introduction
This simulation is designed to verify the effectiveness of our proposed method "Reinforcement-Learning-Based Control Framework for Leader-Following Cascade Formation of Multiple Perturbed Surface Vehicles" by comparing its performance with other existing methods:
1. [Fixed-time extended state observer-based trajectory tracking and point stabilization control for marine surface vessels with uncertainties and disturbances](https://www.sciencedirect.com/science/article/abs/pii/S0029801819302938)
2. [An enhanced tracking control of marine surface vessels based on
adaptive integral sliding mode control and disturbance observer](https://www.sciencedirect.com/science/article/abs/pii/S0019057818305421)

# Method 
This is a cascade control scheme with two loops:
- The outer loop is the high-level trajectory generation, which generates the trajectory reference for the inner loop to achieve the target formation.
- The inner loop is the low-level RL-based optimal controller, which minimizes the performance index. 
![Control strategy diagram](https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/Control%20strategy.jpg?raw=true)

# Result
![targetFormation](https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/FormationTrajectory.jpg?raw=true)

<img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/FormationTrajectory.jpg?raw=true" alt="Description" width="700" height="600">





