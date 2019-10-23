rqt_paramedit
=============

A graphical parameter editor for ROS.

Paramedit can edit params and send to server automatically. If paramedit find service update_params for param's parent this service may be called by yourself(press the button near parent).


Build and Install

Create your own catkin workspace(1) or git clone into existing(2)

1. Create your own catkin workspace
  ```bash
  mkdir temp_ws && cd temp_ws 
  mkdir src && cd src
  catkin_init_workspace
  ```

2. Move into existing directory
  ```bash
  cd <your_workspace>/src
  ```
  
3.  Clone and build
  ```bash
  git clone https://github.com/Jihadist/rqt_paramedit && cd ..
  catkin_make
  ```
  
4. Run
  ```bash
  rosron rqt_paramedit rqt_paramedit
  ```
