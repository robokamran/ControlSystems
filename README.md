<img src="icon.png" align="right" />

# ControlSystems
A pack of control system algorithms implemented in C to be used in embedded systems. In this project, MATLAB is used as simulation platform.

Pull-requests are welcome. If you find such implementations useful, feel free to submit a *feature request* and I'll try to provide more sample implementations (LQR, LQG, sliding-mode, etc.).

> Please make sure that you have the latest version of [MinGW](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-compiler) installed in MATLAB so you can compile the C codes inside MATLAB.

    Header file prototype.h is used as an interface between SIMULINK and C codes.

## Model Reference Adaptive Controller (MRAC)
MRAC controller based on MIT rule structure. References and controller topology can be found at:
- [Model Reference Adaptive Controller
](https://www.mathworks.com/help/physmod/sps/ref/modelreferenceadaptivecontroller.html)
- [Simple Adaptive Control Example](https://www.mathworks.com/matlabcentral/fileexchange/44416-simple-adaptive-control-example)

## Fuzzy Gain Scheduled PID Controller (FGS)
This is a classic controller based on the old topology can be found here:
- [Fuzzy gain scheduling of PID controllers](https://ieeexplore.ieee.org/document/260670)
- [Fuzzy PID Controller](https://www.mathworks.com/matlabcentral/fileexchange/52970-fuzzy-pid-controller)
- [A Course in Fuzzy Systems and Control](https://books.google.co.jp/books/?id=wbJQAAAAMAAJ)

## PID Controller
Both FGS and MRAC topologies utilize an internal PID as well. In this project a tuning based on [Ziegler–Nichols](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method) method is shown. Please pay enough attention to the ***sampling time*** and how it should be utilized in the tuning variable ***Tu***.

## Active Disturbance Rejection Controller (ADRC)
Both MATLAB and C implementation of the controller are provided here based on the recent publications and most optimized topologies. A multi-order profile generator is needed for this controller. Currently an open-source one is used. I will add optimized multi-order profile generators as well.

- [Modeling and Simulation of a Single Gain Tuning ADRC Controller in Matlab/Simulink](https://ieeexplore.ieee.org/abstract/document/9152398)
- [Advanced Setpoints for Motion Systems](https://www.mathworks.com/matlabcentral/fileexchange/16352-advanced-setpoints-for-motion-systems)

## Second Order Filters and Lead-Lag
Since high-pass (washout), low-pass, band-pass and band-stop (notch) filters are commonly used in embedded systems, all simple implementations based on [second-order transfer functions](https://controlsystemsacademy.com/0024/0024.html) are provided and the formulation can be found [here](Classic%20Controllers/SecondOrderFilters.pdf). 

A filtered derivative is commonly used in PIDF structure. Instead of using a two-state differentiation and putting a first-order low-pass, use a native second-order low-pass differentiator.

Also a lead-lag controller/filter is provided based on the [common theories](https://en.wikipedia.org/wiki/Lead%E2%80%93lag_compensator).