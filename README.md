# A Multikernel Correntropy Approach for Invariant Extended Kalman Filtering

## Overview  
This work proposes a multikernel correntropy invariant extended Kalman filter (MKCIEKF) that integrates multikernel correntropy with IEKF methodology to enhance robustness against heavy-tailed extreme noise. It is consistently better than the IEKF in a unicycle simulation and improves by 22.31\% and 25.46\% in the biped robot simulation and the quadruped experiment, demonstrating great potential for applications.

---

## System Requirements
- **MATLAB 2017b**

---

## Example Codes and Descriptions

### Unicycle Robot
- **Objective**: Orientation and velocity estimation.
- **Implementation**: Run "main.m" to compare the performance of EKF, IEKF, MCIEKF, and MKCIEKF on the unicycle robot.

### Biped Robot
- **Objective**: Critical orientation, pose, and velocity estimation.
- **Implementation**: Run "run_RIEKF_test.m" to test the algorithm on the Cassie-series biped robot. The algorithm fuses IMU information and kinematic data.

### Quadruped Robot
- **Objective**: Performance comparison of IEKF, MCIEKF, and MKCIEKF.
- **Implementation**:
  - **Simulation**: Use "iekf_mc_sim.m" for MCIEKF, "iekf_sim.m" for IEKF and "iekf_mck_sim.m" for MKCIEKF.
  - **Real-world experiments**: Use "iekf_mck_real.m" for MKCIEKF and "iekf_real.m" for IEKF.
    
### Image  
Pre-generated figures in /image folder show.

---

## Citation
If you use this work in your research, please cite the relevant publication.

