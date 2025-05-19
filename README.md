# Millimeeterlaine radari integreerimine avatud lähtekoodiga muruniiduki platvormil Open Mower
### Integration of a Millimeter-Wave Radar into the Open-Source Robotic Lawnmower Platform Open Mower

## Ülevaade / Overview

**ET:**  
See bakalaureusetöö keskendub millimeeterlaine radari (mmWave) lisamisele Open Moweri platvormile – avatud lähtekoodiga autonoomsele robotmuruniidukile. Töö eesmärk on suurendada roboti autonoomsust ja ohutust, võimaldades takistuste reaalajas tuvastamist.

**EN:**  
This bachelor's thesis focuses on the integration of a millimeter-wave radar sensor (mmWave) into the Open Mower platform – an open-source autonomous robotic lawnmower. The goal is to enhance the robot’s autonomy and safety by enabling real-time obstacle detection.

- lib/ # Väline või jagatud kood / External/shared libraries
- mower_comms_v1/ # Varasem sideprotokoll / Legacy communication package
- mower_comms_v2/ # Uus sideprotokoll / Updated mower communication interface
- mower_logic/ # Määrab roboti loogika ja käitumise / Main mowing logic
- mower_map/ # Kaardistamine ja RTK GNSS tugi / Mapping and RTK GNSS support
- mower_msgs/ # ROS-i sõnumide kirjeldused / ROS message definitions
- mower_simulation/ # Simulatsioonikeskkond / Simulation tools
- mower_utils/ # Abifunktsioonid ja tööriistad / Utility functions
- open_mower/ # Põhikonfiguratsioon ja launch-failid / Main system launch and configuration
- open_mower_radar_integration/ # Selle töö käigus loodud pakett / Radar integration (created in this thesis)
- serial/ # Seeriaühenduse käsitlemine / Serial communication handler
- ti_mmwave_rospkg/ # Texas Instruments mmWave ROS-draiver / TI mmWave ROS driver
- xr_usb_serial/ # XR USB-Serial konverteri tugi / XR USB-serial integration

## Kasutatud tehnoloogiad / Technologies Used
- ROS Noetic (Robot Operating System) -> https://wiki.ros.org/noetic
- Ubuntu 20.04 (Linux) -> https://releases.ubuntu.com/focal/
- RTK-GNSS (Ardusimple F9P) -> https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/
- D3 Engineering + Texas Instruments mmWave radar (D3 Engineering RS1843AOPU) -> https://www.d3embedded.com/product/designcore-rs-1843aopu-mmwave-radar-sensor/
- Open Mower avatud platvorm / Open Mower open-source platform - https://openmower.de/
- OnShape CAD -> https://www.onshape.com/en/
- u-blox u-center -> https://www.u-blox.com/en/product/u-center
- ti_mmWave Visualizer -> https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/

## Lingid Open Mower kommuunile / Reference links for Open Mower
- Discord server -> https://discord.gg/Hp7qyQe9
- Open Mower github -> https://github.com/ClemensElflein/OpenMower
- Open Mower Patreon -> https://www.patreon.com/ClemensElflein

## Bakalaureusetöö dokument / Bachelor thesis document (TODO/TBD!)

## Tänuavaldused / Expressions of gratidude (TODO)
