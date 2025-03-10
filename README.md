# TGAS-Sim: Simulation-Based Validation of Transverse Guidance Assist System (TGAS)

## Overview
**TGAS-Sim** is an open-source repository containing **MATLAB Simulink models, simulation scenarios, and analysis results** for the **Transverse Guidance Assist System (TGAS) applicable to Highly Automated Driving (HAD)**. The goal is to evaluate **trajectory planning, lane-change maneuvers, and perception-based decision-making** using simulation-based verification and validation.

This repository provides:
- **Simulink models & MATLAB scripts** for TGAS simulation.
- **Predefined highway scenarios** for validation.
- **Graphical representations of scenarios, model design, and results**.
- **Published research papers (linked) related to TGAS**.

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
│   ├── lane_change_simulation.mp4  # Video of highway lane change  
│   ├── TGAS_sim_demo.mp4  # Overview of TGAS in Simulink  
│   ├── scenario_visual.png  # Graphical representation of scenario  
│   ├── model_design.png  # Graphical representation of model design  
│   ├── simulation_results.png  # Graphs and plots of results  
│  
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

### **1️⃣ Driving Scenario Overview**
![image](https://github.com/user-attachments/assets/d4a61796-72cc-4331-bd64-6f30b4a3ea6e)


### **3️⃣ Approach - Trajectory Planning & Control**
![behaviour planner](https://github.com/user-attachments/assets/309da0c1-4375-4640-b852-0d9e7b6fd0d7)

### **4️⃣ Results - Lane Change Maneuver**
![all phases acceleration](https://github.com/user-attachments/assets/ac5820bc-6994-42d7-b881-fb4e60126a2d)
![velocity](https://github.com/user-attachments/assets/b566043c-b1af-44b7-bcca-711a4fede80a)
![Optimized trajectory](https://github.com/user-attachments/assets/43b4ba31-f281-4550-bb01-4a6688ce1dd3)


## How to Contribute
We welcome contributions from researchers and engineers! Please check **CONTRIBUTING.md** for guidelines.

## License
This project is **licensed under Apache 2.0**. See **LICENSE** for details.

## Contact & Discussion
If you have any questions, suggestions, or would like to collaborate on future research, feel free to reach out to me:
- 📧 **Email:** milinp101996@gmail.com
- 🔗 **ResearchGate:** [Milin Patel](https://www.researchgate.net/profile/Milin-Patel?ev=hdr_xprf)

I look forward to engaging with the community and expanding research in this field! 🚀
