<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

[![Stargazers][stars-shield]][stars-url]
[![LinkedIn][linkedin-shield]][linkedin-url]





<!-- ABOUT THE PROJECT -->
# About The Project
This project presents the modeling, simulation, and control of a plus configuration of quadcopter using proportional–integral–derivative (PID) controllers implemented in MATLAB/Simulink.


## PID Design for Plus-Configurated Quadcopter
Project contains kinematics and dynamics of quadcopter, as well as mixture control mechnaism for throttle commnands for each rotors. The model incorporates gravitational, gyroscopic, and aerodynamic effects to capture the essential flight dynamics. Simplifications (linearized motors, ignored disturbances, empirical constants) make the model less realistic but effective for conceptual validation.

A hierarchical control architecture is developed, consisting of an outer position control loop and an inner attitude and altitude stabilization loop. PID and PI controllers are tuned to achieve stable trajectory tracking in the X–Y plane and altitude regulation along the Z-axis. Simulation results demonstrate satisfactory flight performance.

### Built With

* [![MATLAB][MATLAB]][MATLAB-url]
* [![SIMULINK][SIMULINK]][SIMULINK-url]


### Prerequisites

Make sure that MATLAB and Simulink R2023a or newer version is already downloaded. Some functions or blocks will not work in older versions.



<!-- USAGE EXAMPLES -->
## Usage

`CS.mat` : Parameters for simulation

`Quad_dynamics.m` : Kinematics and Dynamics formulation for Simulation block.

`Qudcopter.slx` : Main simulation

`path.mat` : Command for path and yaw angle

Please refer to the [Report](https://github.com/gulsoynes/PID-for-Quadcopter/blob/main/REPORT.pdf). Essential formulation and details and results of simulation are reported. 




<!-- CONTACT -->
## Contact

Neslihan Gülsoy - gulsoyneslihan0@gmail.com



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: www.linkedin.com/in/neslihan-gülsoy

[MATLAB]: https://img.shields.io/badge/matlab-001000?style=for-the-badge&logo=matlab&logoColor=blue
[MATLAB-url]: https://www.mathworks.com/products/matlab.html

[SIMULINK]: https://img.shields.io/badge/simulink-001000?style=for-the-badge&logo=matlab&logoColor=blue
[SIMULINK-url]: https://www.mathworks.com/products/simulink.html

[stars-shield]: https://img.shields.io/github/stars/gulsoynes/PID-for-Quadcopter.svg?style=for-the-badge
[stars-url]: https://github.com/gulsoynes/PID-for-Quadcopter/stargazers
