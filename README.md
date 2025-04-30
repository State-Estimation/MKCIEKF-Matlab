# A Multikernel Correntropy Approach for Invariant Extended Kalman Filtering

## Overview  
This work proposes a multikernel correntropy invariant extended Kalman filter (MKCIEKF) that integrates multikernel correntropy with IEKF methodology to enhance robustness against heavy-tailed extreme noise. It is consistently better than the IEKF in a unicycle simulation and improves by 22.31\% and 25.46\% in the biped robot simulation and the quadruped experiment, demonstrating great potential for applications.

---

## System Requirements  
- **MATLAB 2022a** or newer  
 
---

## Repository Structure 
```
MKCIEKF_Matlab/
├── unicycle/                           # Unicycle experiment
│   ├── ExtendedKalmanFilter.m         - Implementation of the Extended Kalman Filter
│   ├── InvariantEKF.m                 - Implementation of the IEKF, MCIEKF and MKCIEKF
│   ├── UnicycleSystem.m               - Defines the unicycle system
│   └── main.m                         - Filter comparison under unicycle 
├── quadruped robot/                    # Quadruped robot simulation and experiment
│   ├── iekf_mc_sim.m                  - Implementation of the MCIEKF for simulation
│   ├── iekf_mck_real.m                - Implementation of the MKCIEKF with real data
│   ├── iekf_mck_sim.m                 - Implementation of the MKCIEKF for simulation
│   ├── iekf_real.m                    - Implementation of the IEKF with real data
│   └── iekf_sim.m                     - Implementation of the IEKF for simulation
└── images/                             # Experimental result visualization
```
---

## Citation
- If you use this work in your research, please cite:
---
