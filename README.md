# SKAR Rover Software

This repository contains source code of our software that runs on the on-board computers of Sirius rover family.

## Requirements
* ROS Noetic running on Ubuntu 20.04
* Python 3
* Docker (optionally)

## Building
Clone the repository onto your local machine (using https is easier, but you can use ssh as well):
```bash
$ git clone https://gitlab.com/ska_robotics/sirius/rover-soft
```

Enter the project's workspace:
```bash
$ cd rover-soft
```

Use `rosdep` to install any missing dependencies:
```bash
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
```

Build the project using:
```bash
$ catkin build
```

Always remember to source ros:
```bash
$ source devel/setup.bash
```

## Launching
> *Coming soon...&trade;*

## Docker
Our pipeline automatically creates docker images based on the newest version of the project available on `develop` and `main` branches. You can run Rover Soft inside a container using Docker.

First, you need to login into our container registry:
```bash
$ docker login registry.gitlab.com -u <gitlab_email> -p <access_token>
```

Now you can use docker compose to start Rover Soft:
```bash
$ docker-compose up
```

Alternatively, you can enter an interactive shell with ready environment using:
```bash
$ docker run -it registry.gitlab.com/ska_robotics/sirius/rover-soft/rover-soft:stable
```

## Documentation
Our project's documentation is avaiable [here](https://ska_robotics.gitlab.io).
Further information about the software is also available in detailed descriptions of each package.

#
Copyright &copy; 2021, SKA Robotics Warsaw University of Technology
