# Ellipsoidal Set-Theoretic Robust Safety Filter

This repository provides MATLAB implementations of robust safety filters for constrained linear systems, as presented in our paper "[Ellipsoidal Set-Theoretic Design of Robust Safety Filters for Constrained Linear Systems](https://arxiv.org/abs/2510.22790)."

## Overview

Safety filters are supervisory control mechanisms that ensure safety-critical constraints are satisfied while minimally interfering with nominal controller performance. This implementation demonstrates an ellipsoidal set-theoretic approach for synthesizing robust safety filters that:

- Compute robust controlled invariant (RCI) ellipsoidal sets via convex LMI optimization
- Provide formal safety guarantees under bounded disturbances
- Enable smooth transitions between nominal and backup controllers
- Scale efficiently to high-dimensional systems

The method is validated on a 6-DOF quadrotor system with hierarchical position-attitude control under wind disturbances.

## Files Description

### Main Simulation Scripts
- `quadrotorScenario_I.m` - Nominal operation with small position errors
- `quadrotorScenario_II.m` - Large initial position errors requiring safety intervention
- `quadrotorScenario_III.m` - Aggressive high-frequency circular trajectory tracking

### Core Functions
- `loadSystemParameters.m` - Quadrotor parameters (mass, inertia, etc.) for Crazyflie 2.0
- `defineEquilibriumPoint.m` - Hover equilibrium point calculation
- `linearizeModel.m` - Attitude dynamics linearization around equilibrium
- `robustControlDesign.m` - LMI-based RCI set and backup controller synthesis
- `computeAttitudeLinearizationErrorBound.m` - Rigorous linearization error bounds
- `plotResults.m` - Comparative visualization of safe vs. unsafe scenarios
- `formatFigureIEEE.m` - IEEE-standard figure formatting

## Usage

**Run a simulation scenario**:
```matlab
   quadrotorScenario_II  % Large position errors scenario
```

## Requirements

- MATLAB R2020a or later
- [CVX](http://cvxr.com/cvx/) - Convex optimization toolbox
- [MOSEK](https://www.mosek.com/) or SeDuMi - SDP solver
- Control System Toolbox

### Installation
1. Install CVX from http://cvxr.com/cvx/download/
2. Install MOSEK solver (academic license available)
3. Add CVX to MATLAB path:
```matlab
   addpath('path/to/cvx')
   cvx_setup
```


## Citation

If you use this code in your research, please cite:
```bibtex
@misc{pordal2025ellipsoidalsettheoreticdesignrobust,
      title={Ellipsoidal Set-Theoretic Design of Robust Safety Filters for Constrained Linear Systems}, 
      author={Reza Pordal and Alireza Sharifi and Ali Baniasad},
      year={2025},
      eprint={2510.22790},
      archivePrefix={arXiv},
      primaryClass={eess.SY},
      url={https://arxiv.org/abs/2510.22790}, 
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

**Reza Pordal**  
Email: rezapordal@gmail.com

For questions, issues, or contributions, please open an issue on GitHub.
