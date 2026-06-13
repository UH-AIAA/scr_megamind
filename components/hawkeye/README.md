# Hawkeye

Hawkeye is a sounding/hobby rocket GNC library written by [Nathan Samuell](https://linkedin.com/in/nathan-samuell). 
Current capabilties include:
- FSM event detection
- simple filters used for noise smoothing/sensor calibration
- math utilities

Eventual goals include:
- Numerical integration (RK45)
- EKF/GPS-INS integration
- active control (roll, airbrakes, etc)
- SIL simulation framework

The codbase is thoroughly tested using **cmocka** - tests can be found in the `test` directory and built using the directions below

## Building
1. Install GCC, Make, and CMake onto your machine
2. clone the project, I recommend using an SSH key:
    - `git clone --recurse-submodules git@github.com:UH-AIAA/scr_Hawkeye.git`
2. Run `make lib` in the root of the project. This will build a static (`.a) library for your project to link to
3. Run `make test_exec` to build the unit test executable for the project. Run with `./hawkeye_test`!
    - alternatively run `make all` to do both of these things!

## Contributing
If you're interested in contributing, please see [issues](https://github.com/UH-AIAA/hawkeye/issues) for open work items, and/or email nathan.samuell@duck.com if you need any help! Also feel free to open your own issue if you have suggestions.
