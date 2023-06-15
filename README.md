# motoko ice dragon line follower

![logo](doc/images/ice_dragon_logo.jpg)


![animation](doc/images/robot_lqr.gif)

- brushless motors
- LQG - linear quadratic regulator + Kalman filter
- LSTM for line shape prediction
- multiple local optimal LQR controllers selecting

![animation](doc/images/robot_mlqr.gif)

 
# LQR controller

![lqr_structure](doc/diagrams/control-lqri_synth.png)

![lqr_results](utils/lqr_controller/results/poles.png)
![lqr_results](utils/lqr_controller/results/poles_mesh_cl.png)

![lqr_results](utils/lqr_controller/results/closed_loop_response.png)

# meta LQR control
![lqr_structure](doc/diagrams/control-mlqri.png)
