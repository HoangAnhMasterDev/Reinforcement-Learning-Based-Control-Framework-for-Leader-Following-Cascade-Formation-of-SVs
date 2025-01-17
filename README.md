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

# Result and Comparison
To begin with, we have the trajectory of all five SVs achieving the target formation. The picture presents snapshots of the simulation at different time instants.

<div style="text-align: center;">
<img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/FormationTrajectory.jpg?raw=true" alt="Description" width="700" height="650">
</div>

Subsequently, the following pictures depict the comparison of the control input in the x-axis, y-axis, and yaw angle of SV 4, from three different control schemes (AISMC, FXESO, and RL, respectively):

<div style="display: flex; justify-content: space-around;">
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/tau_AISMC.jpg?raw=true" alt="AISMC" width="330"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/tau_FXESO.jpg?raw=true" alt="FXESO" width="330"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/tau_RL.jpg?raw=true" alt="RL" width="320"/>
</div>

Then, we have the tracking error performance in the x-axis, y-axis, and yaw angle, respectively of SV 4. It is observable that our control strategy is the only method that can guarantee the prescribed performance:

<div style="display: flex; justify-content: space-around;">
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/trackingError_x.jpg?raw=true" alt="AISMC" width="330"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/trackingError_y.jpg?raw=true" alt="FXESO" width="330"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/trackingError_psi.jpg?raw=true" alt="RL" width="320"/>
</div>

Finally, we present the evolution of both Actor & Critic training weights of SV 4:

<div style="display: flex; justify-content: space-around;">
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/Wa4.jpg?raw=true" alt="AISMC" width="490"/>
  <img src="https://github.com/HoangAnhMasterDev/Reinforcement-Learning-Based-Control-Framework-for-Leader-Following-Cascade-Formation-of-SVs/blob/main/Images/Wc4.jpg?raw=true" alt="FXESO" width="490"/>
</div>

It should be noted that we only show the results collected from SV 4 because the results of each SV are similar to those of other SVs. 

# Contact
Feel free to contact me via [my email](hoanganhlyk26@gmail.com) 



