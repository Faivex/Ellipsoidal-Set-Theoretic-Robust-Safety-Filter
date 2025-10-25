# Ellipsoidal Set-Theoretic Robust Safety Filter for Quadrotor

This repository contains MATLAB code for robust attitude control design and simulation of quadrotor unmanned aerial vehicles (UAVs) with ellipsoidal set-theoretic safety guarantees.

## Overview

The code implements a robust control framework for quadrotor attitude stabilization using Linear Matrix Inequalities (LMI) and safety filtering techniques. The system includes:

- Nonlinear quadrotor dynamics modeling
- Linearization around equilibrium points
- Robust controller design with disturbance rejection
- Safety-critical control with set-invariance guarantees
- Comparative simulation scenarios

## Files Description

### Main Simulation Scripts
- `quadrotorSenario_I.m` - Primary simulation scenario with safety filter comparison
- `quadrotorSenario_II.m` - Alternative simulation scenario
- `quadrotorSenario_III.m` - Extended simulation scenario

### Core Functions
- `loadSystemParameters.m` - System parameter definitions (mass, inertia, etc.)
- `defineEquilibriumPoint.m` - Equilibrium point calculation
- `linearizeModel.m` - System linearization around operating points
- `robustControlDesign.m` - LMI-based robust controller synthesis
- `computeAttitudeLinearizationErrorBound.m` - Error bound computation
- `plotResults.m` - Visualization and result plotting
- `formatFigureIEEE.m` - IEEE-standard figure formatting

## Usage

1. Run any of the main scenario scripts:
   ```matlab
   quadrotorSenario_I
   ```

2. The simulation will:
   - Load system parameters
   - Design a robust controller
   - Simulate both safe and unsafe control scenarios
   - Generate comparative plots

## Requirements

- MATLAB R2020a or later
- Control System Toolbox
- Robust Control Toolbox
- YALMIP (for LMI optimization)
- SeDuMi or similar SDP solver

## Key Features

- **Robust Control Design**: Uses Linear Matrix Inequalities for controller synthesis
- **Safety Filtering**: Implements set-invariance based safety constraints
- **Disturbance Rejection**: Handles external disturbances and model uncertainties
- **Comparative Analysis**: Evaluates performance with and without safety guarantees

## System Model

The quadrotor model includes:
- 12-state nonlinear dynamics (position, velocity, attitude, angular rates)
- Attitude control with roll, pitch, and yaw torque inputs
- External disturbances and linearization errors

## Citation

If you use this code in your research, please cite:

```bibtex
@misc{quadrotor_robust_control_2024,
  title={Ellipsoidal Set-Theoretic Robust Safety Filter for Quadrotor},
  author={Reza Pordal},
  year={2025},
  url={https://github.com/Faivex/Ellipsoidal-Set-Theoretic-Robust-Safety-Filter-for-Quadrotor}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

Reza Pordal - rezapordal@gmail.com