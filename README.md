# Integration of a Millimeter-Wave Radar into the Open-Source Robotic Lawnmower Platform Open Mower

## Overview

![IMG_20250519_164715](https://github.com/user-attachments/assets/bb0fcd7a-072e-48c4-9725-0293424676ff)

This bachelor's thesis focuses on the integration of a millimeter-wave radar sensor (mmWave) into the Open Mower platform – an open-source autonomous robotic lawnmower. The goal is to enhance the robot’s autonomy and safety by enabling real-time obstacle detection.

### 3D-models for portable base station and mmwave enclosure
- These can be found under step_models.zip or step_models.tar. Feel free to use and edit them :)

### Packages used
- lib/ # External/shared libraries
- mower_comms_v1/ # Legacy communication package
- mower_comms_v2/ # Updated mower communication interface
- mower_logic/ # Main mowing logic
- mower_map/ # Mapping and RTK GNSS support
- mower_msgs/ # ROS message definitions
- mower_simulation/ # Simulation tools
- mower_utils/ # Utility functions
- open_mower/ # Main system launch and configurations
- open_mower_radar_integration/ # Radar integration (created in this thesis)
- serial/ # Serial communication handler
- ti_mmwave_rospkg/ # TI mmWave ROS driver
- xr_usb_serial/ # XR USB-serial integration

## Technologies Used
- ROS Noetic (Robot Operating System) -> https://wiki.ros.org/noetic
- Ubuntu 20.04 (Linux) -> https://releases.ubuntu.com/focal/
- RTK-GNSS (Ardusimple F9P) -> https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/
- D3 Engineering + Texas Instruments mmWave radar (D3 Engineering RS1843AOPU) -> https://www.d3embedded.com/product/designcore-rs-1843aopu-mmwave-radar-sensor/
- Open Mower open-source platform - https://openmower.de/
- OnShape CAD -> https://www.onshape.com/en/
- u-blox u-center -> https://www.u-blox.com/en/product/u-center
- ti_mmWave Visualizer -> https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/

## Reference links for Open Mower
- Documenatation page -> https://openmower.de/
- Discord server -> https://discord.gg/Hp7qyQe9
- Open Mower github -> https://github.com/ClemensElflein/OpenMower
- Open Mower Patreon -> https://www.patreon.com/ClemensElflein

## Bachelor thesis document (TBD whether it is allowed here :) )


## Expressions of gratidude
- My thesis supervisor, for guidance, teaching and advice throughout the work
- Open Mower community for teaching, guidance
- ChatGPT artificial intelligence tool for the correctness and refinement of the thesis document.

**Karl Rahn**  
University of Tartu, [Institute of Technology, Computer Engineering]  
2025
