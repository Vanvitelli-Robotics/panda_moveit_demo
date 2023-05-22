# Panda MoveIt! demo

[![N|Solid](https://www.unicampania.it/doc/img/logo_vanvitelli.jpg)](https://www.ingegneria.unicampania.it/roboticslab)

Demo rete ros con planning MoveIt! per il corso di Programmazione dei Robot.

## Install

- Prerequisiti: Installare Moveit!

Nella cartella src del ros workspace
```bash
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel
git clone https://github.com/Vanvitelli-Robotics/panda_moveit_demo_support.git
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

## Avviare la demo

Avviare il rosmaster:
```bash
roscore
```

In un nuovo terminale avviare il simulatore:
```bash
roslaunch panda_moveit_demo_support demo.launch
```

In un nuovo terminale chiamare il servizio di generazione scena (dovreste farlo da codice!):
```bash
rosservice call /build_scene "{}"
```

## License

GNU General Public License v3.0
