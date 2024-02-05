![](./docs/images/banner.png)

# Development Notice

**WARNING:** This is a development repo that we are using to port the stretch_funmap ROS package to a Python 3 PyPi module We plan to write unit and system tests, improve the algorithms, and create robust Python demos in this port. The code in this repo may not be in a usable state. The code in this repo may be unstable, since we are actively conducting development in this branch. Since we have performed limited testing, you may encounter unexpected behaviors.

# Overview

stretch_funmap is an implementation of Fast Unified Navigation, Manipulation And Planning (FUNMAP). FUNMAP provides navigation, manipulation, and planning capabilities for the Stretch RE1 mobile manipulator. stretch_funmap includes examples of efficient ways to take advantage of the Stretch RE1's unique properties.

Previous commercially-available mobile manipulators have consisted of a serial manipulator (i.e., links connected by rotary joints) placed on a mobile base [1]. Widely used software (e.g., the Robot Operating System (ROS)) [1] typically expects a velocity-controlled mobile base that can be held still while the arm manipulates [3, 4].

In contrast, the Stretch RE1's mobile base is integral to manipulation and typically moves throughout a task. It can also perform high-fidelity position control with its mobile base. FUNMAP uses approximate geometric models and computer-vision algorithms to efficiently find plans that take advantage of its prismatic joints (e.g., telescoping arm) and Cartesian structure. In contrast to typical approaches that treat navigation (e.g., ROS Navigation Stack ) and manipulation (e.g., MoveIt! [5, 6]) separately, FUNMAP does both.

**References:**
 - [1] Bostelman, Roger, Tsai Hong, and Jeremy Marvel. "Survey of research for performance measurement of mobile manipulators." Journal of Research of the National Institute of Standards and Technology 121, no. 3 (2016): 342-366.

 - [2] Quigley, Morgan, Ken Conley, Brian Gerkey, Josh Faust, Tully Foote, Jeremy Leibs, Rob Wheeler, and Andrew Y. Ng. "ROS: an open-source Robot Operating System." In ICRA workshop on open source software, vol. 3, no. 3.2, p. 5. 2009.

 - [3] Sachin Chitta, Eitan Marder-Eppstein, Wim Meeussen, Vijay Pradeep, Adolfo Rodríguez Tsouroukdissian, et al.. ros_control: A generic and simple control framework for ROS. The Journal of Open Source Software, 2017, 2 (20), pp.456 - 456.

 - [4] Guimarães, Rodrigo Longhi, André Schneider de Oliveira, João Alberto Fabro, Thiago Becker, and Vinícius Amilgar Brenner. "ROS navigation: Concepts and tutorial." In Robot Operating System (ROS), pp. 121-160. Springer, Cham, 2016.

 - [5] Chitta, Sachin, Ioan Sucan, and Steve Cousins. "Moveit! [ros topics]." IEEE Robotics & Automation Magazine 19, no. 1 (2012): 18-19.

 - [6] Chitta, Sachin. "MoveIt!: an introduction." In Robot Operating System (ROS), pp. 3-27. Springer, Cham, 2016.

# Testing and Development

## Developing

The source code for Stretch PyFUNMAP resides within the `src/stretch_pyfunmap/` folder. You can install Stretch PyFUNMAP as "editable", and directly edit the source code to test changes.

In Python3, run `python3 -m pip install -e .`

For example, to test changes to `stretch_pyfunmap.max_height_image.MaxHeightImage`, run:

```bash
$ python3 -m pip uninstall hello-robot-stretch-pyfunmap # ensure previous Stretch PyFUNMAP installations are removed
$ git clone https://github.com/hello-robot/stretch_pyfunmap.git
$ cd stretch_pyfunmap/src
$ python3 -m pip install -e .
```

Now, make desired edits to the [stretch_pyfunmap/src/stretch_pyfunmap/max_height_image.py](./src/stretch_pyfunmap/max_height_image.py) file. Software using Stretch PyFUNMAP is now using the modified `stretch_pyfunmap.max_height_image.MaxHeightImage` class.

## Running Tests

There are a number of unit, functional, and performance tests within the `src/test/` folder, separated into test suites by different files. Suites are separated by a functionality within Stretch PyFUNMAP that is being tested.

In Python3, [install pip locally](#developing) and run `python3 -m unittest test.test_<suite-name>`.

For example, to run the `stretch_pyfunmap.max_height_image.MaxHeightImage` unit tests, run:

```bash
# ensure PyFUNMAP installed locally
$ cd stretch_pyfunmap/src
$ python3 -m unittest test.test_mhi
```

## Changelog

See the [changelog](./CHANGELOG.md) for information on what changed with each release.

# License

Each subdirectory contains a LICENSE.md file that applies to the directory's contents. This software is intended for use with the Stretch mobile manipulatorss, which are robots produced and sold by Hello Robot Inc. For further information including inquiries about dual licensing, please contact Hello Robot Inc.


------
<div align="center"> All materials are Copyright 2022-2024 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks. </div>

