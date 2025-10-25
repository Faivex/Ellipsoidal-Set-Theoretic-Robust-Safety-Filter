# Ellipsoidal Set-Theoretic Robust Safety Filter for Quadrotor

This repository provides MATLAB implementations of robust safety filters for constrained linear systems, as presented in our paper "Ellipsoidal Set-Theoretic Design of Robust Safety Filters for Constrained Linear Systems."

## Overview



## Files Description

### Main Simulation Scripts
- `quadrotorSenario_I.m` - Scenario I
- `quadrotorSenario_II.m` - Scenario II
- `quadrotorSenario_III.m` - Scenario III

### Core Functions
- `loadSystemParameters.m` - System parameter definitions (mass, inertia, etc.)
- `defineEquilibriumPoint.m` - Equilibrium point calculation
- `linearizeModel.m` - System linearization around operating points
- `robustControlDesign.m` - LMI-based robust safety filter synthesis
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
   - Design a robust safety filter
   - Simulate both safe and unsafe control scenarios
   - Generate comparative plots

## Requirements

- MATLAB R2020a or later
- YALMIP (for LMI optimization)
- SeDuMi or similar SDP solver
- ??


## Citation

If you use this code in your research, please cite:

```bibtex
@misc{quadrotor_robust_control_2024,
  title={Ellipsoidal Set-Theoretic Design of Robust Safety Filters for Constrained Linear Systems},
  author={Reza Pordal, Alireza Sharifi, Ali Baniasad},
  year={2025},
  url={https://github.com/Faivex/Ellipsoidal-Set-Theoretic-Robust-Safety-Filter-for-Quadrotor}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

Reza Pordal - rezapordal@gmail.com