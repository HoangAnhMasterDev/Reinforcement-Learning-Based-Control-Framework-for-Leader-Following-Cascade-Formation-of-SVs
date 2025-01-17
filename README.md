# Introduction
This simulation is designed to verify the effectiveness of our proposed method " (RL) Reinforcement-Learning-Based Control Framework for Leader-Following Cascade Formation of Multiple Perturbed Surface Vehicles" by comparing its performance with other existing methods:
1. [(FXESO) Fixed-time extended state observer-based trajectory tracking and point stabilization control for marine surface vessels with uncertainties and disturbances](https://www.sciencedirect.com/science/article/abs/pii/S0029801819302938)
2. [(AISMC) An enhanced tracking control of marine surface vessels based on
adaptive integral sliding mode control and disturbance observer](https://www.sciencedirect.com/science/article/abs/pii/S0019057818305421)

# Method 
This is a cascade control scheme with two loops:
- The outer loop is the high-level trajectory generation, which generates the trajectory reference for the inner loop to achieve the target formation.
- The inner loop is the low-level RL-based optimal controller, which minimizes the performance index.
  
![Control strategy diagram](https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/Control%20strategy.jpg?raw=true)

# Result
To begin with, we have the trajectory of all five SVs achieving the target formation. The picture presents snapshots of the simulation at different time instants.

<img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/FormationTrajectory.jpg?raw=true" alt="Description" width="700" height="650">

Subsequently, the following pictures depict the comparison of the control input in the x-axis, y-axis, and yaw angle of SV 4, from three different control schemes:

<div style="display: flex; justify-content: space-around;">
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/tau_AISMC.jpg?raw=true" alt="AISMC" width="250"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/tau_FXESO.jpg?raw=true" alt="FXESO" width="250"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/tau_RL.jpg?raw=true" alt="RL" width="250"/>
</div>






