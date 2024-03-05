# motoko ice dragon line follower

![logo](doc/images/ice_dragon_logo.jpg)
![robot](doc/images/robot/robot.gif)

![3d_print](doc/images/robot/robot_0_b.jpg)
![3d_print](doc/images/robot/robot_4.jpg)


# source structure
- [3d_print](3d_print)
- [hardware](hardware)
- [simulations](simulations)
- [firmware](firmware)


# LQR controller 

![lqr_structure](doc/diagrams/control-lqri_synth.png)

## motors velocity control

- step response based identification
- LQR synthesis
- [lqg_discrete notebook](simulations/motor_controller/lqg_discrete.ipynb)
- step response and disturbance rejection
![lqg_discrete](simulations/motor_controller/lqg_discrete_output.png)


## position controll

- kalman based identification
- LQR synthesis
- [lqr_discrete notebook](simulations/robot_controller/robot_controller_a.ipynb)
- step response and disturbance rejection
![lqr_discrete](simulations/robot_controller/lqr_discrete_output.png)


# photos

- [photos](doc/images/README.md)


![animation](doc/images/robot_lqr.gif)

- brushless motors
- LQG - linear quadratic regulator + Kalman filter
- LSTM for line shape prediction
- multiple local optimal LQR controllers selecting

![animation](doc/images/robot_mlqr.gif)

# software architecture
![block_diagram](doc/diagrams/sw_architecture.png)

