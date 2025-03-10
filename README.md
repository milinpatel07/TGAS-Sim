# TGAS-Sim: Simulation-Based Validation of Transverse Guidance Assist System (TGAS)

## Overview
**TGAS-Sim** is an open-source repository containing **MATLAB Simulink models, simulation scenarios, and analysis results** for the **Transverse Guidance Assist System (TGAS) applicable to Highly Automated Driving (HAD)**. The goal is to evaluate **trajectory planning, lane-change maneuvers, and perception-based decision-making** using simulation-based verification and validation.

This repository provides:
- **Simulink models & MATLAB scripts** for TGAS simulation.
- **Predefined highway scenarios** for validation.
- **Bird’s Eye View & simulation results** for performance analysis.
- **Published research papers & documentation** related to TGAS.

## Research Significance
This repository is based on **published research work** and is a result of my **Master’s Thesis research**, which successfully led to two conference publications. I created this repository to **support and help others working in or interested in TGAS, trajectory planning, and autonomous vehicle safety**. 

I welcome **suggestions, discussions, and contributions** from the community. I also aim to take this topic further for research and would love to collaborate with researchers and engineers working in this field.

1. **Master’s Thesis**: *Simulation-Based Verification and Validation of TGAS for HAD* ([DOI](https://doi.org/10.13140/RG.2.2.21734.74567))
2. **Conference Paper 1**: *Simulation-Based Analysis Of Highway Trajectory Planning* ([DOI](https://doi.org/10.1109/iceccme52200.2021.9591044))
3. **Conference Paper 2**: *A Simulation-Based Analysis for Transverse Guidance* ([DOI](https://doi.org/10.1109/hora52670.2021.9461313))

## Repository Structure
```
📂 TGAS-Sim  
│── 📜 README.md  # Introduction & Instructions  
│── 📜 LICENSE  # Apache 2.0 License  
│── 📜 CONTRIBUTING.md  # How to contribute  
│── 📜 CODE_OF_CONDUCT.md  # Community guidelines  
│  
├── 📂 Simulink-Files/  # MATLAB & Simulink simulation models  
│   ├── instructions.mlx  # Live script to open & load scenario  
│   ├── main_model.slx  # Simulink model file  
│   ├── plotResults.m  # MATLAB script for visualization  
│   ├── scenario.mat  # Predefined simulation scenario  
│  
├── 📂 Results-and-Media/  # Pre-generated results & analysis  
│   ├── Simulation-Visualizations/  # Screenshots, Bird’s Eye View, etc.  
│   ├── Data-Logs/  # MATLAB logs & tracking data  
│  
├── 📂 Publications/  # Research papers related to the project  
│   ├── Thesis-Paper.pdf  # Your Master’s thesis  
│   ├── Conference-Paper-1.pdf  # ICECCME paper  
│   ├── Conference-Paper-2.pdf  # HORA paper  
│  
├── 📂 Documentation/  # Explanations & technical insights  
│   ├── TGAS-Overview.md  # Introduction to TGAS  
│   ├── Highway-Scenario.md  # Lane change planning & methodology  
│   ├── Controller-Design.md  # Model Predictive Control (MPC) strategy  
│  
└── 📂 Videos/  # Optional - Demo videos of the simulations  
    ├── lane_change_simulation.mp4  # Video of highway lane change  
    ├── TGAS_sim_demo.mp4  # Overview of TGAS in Simulink  
```

## Running the Simulation
To **run the TGAS simulation**, follow these steps:
1. **Open MATLAB & Simulink**.
2. **Run `instructions.mlx`** to load the model & scenario.
3. **Open Simulink model** and navigate to **Bird's Eye Scope**.
4. Click **Find Signals** and **Run Simulation**.
5. **Execute `plotResults.m`** to visualize simulation results.

## Snapshots of the Model, Scenario & Results
Below are images representing **the scenario, approach, and results**:

### **1️⃣ Scenario Overview**
![Scenario Snapshot](image.png)

### **2️⃣ Simulink Model**
![Simulink Model](image.png)

### **3️⃣ Approach - Trajectory Planning & Control**
![Approach](image.png)

### **4️⃣ Results - Lane Change Maneuver**
![Results](image.png)

## How to Contribute
We welcome contributions from researchers and engineers! Please check **CONTRIBUTING.md** for guidelines.

## License
This project is **licensed under Apache 2.0**. See **LICENSE** for details.

## Contact & Discussion
If you have any questions, suggestions, or would like to collaborate on future research, feel free to reach out to me:
- 📧 **Email:** milinp101996@gmail.com
- 🔗 **ResearchGate:** [Milin Patel](https://www.researchgate.net/profile/Milin-Patel?ev=hdr_xprf)

I look forward to engaging with the community and expanding research in this field! 🚀
