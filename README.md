# compact_drone
compact contra rotating drone

## Table of Contents
- [Requirements](#requirements)
- [Rationale](#rationale)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Future_Additions](#future_additions)
- [Configuration](#configuration)
- [Known_Issues](#known_issues)

## Requirements
- Software:
    - Freecad (for 3D part design)
    - KICAD (PCB design)
    - ST Cube IDE (ST account required for download)
- Hardware:
    - BOM to come
    - Access to a 3D printer

## Rationale
The design idea for this drone is not my own. Despite it being a rare design I got inspiration from projects such as [backcountrydrones](https://www.youtube.com/watch?v=82phDhYGT_0) and [Tdrone](https://github.com/ShenZhenAccelerationTechCo/Tdrone). Backcountry Drones guys have now changed to Ascent Aerosystems and make military/law inforcement and industrial coaxial drones. The NASA mars copter is another example of a contra rotating drone design.

The idea is that the propellers can fold down which means the drone can fit in smaller storage solutions. Another idea would be to tube launch the drone for rapid remote deployment from a ground or air vehicle.

Using a coaxial design is more efficient in theory. Having two large propeller spinning about the center of the drone increases the disc area compared to having four propellers next to each other that will waste space. A coaxial drone with a tube shape also reduces its surface area when tilting forwards to move making it more streamlined than a quadcopter that shows more surface area to drag when moving forward. I reccomend reading this paper from Ascent Aerosystems about all the advantages and its got diagrams https://ascentaerosystems.com/wp-content/uploads/2024/12/Coaxial-Advantage-White-Paper-Ascent-AeroSystems.pdf.

Some applications for the drone would be for survey work in remote areas, filming, compact and rapid deployment via stackable launch cells. 

- STM32f411:
    - Same 

https://github.com/user-attachments/assets/415317f5-2f4a-4eaa-9d52-048ebe863607

MCU I used for my turret project. Its fast, powerful, and has many timers for pwm output. Since I used it for the turret it makes the PCB development for the drone much faster.

Progress Update:
![Picture of Kicad PCB](/Pictures/KiCADPCBPrototypeV1.png)
![Picture of FreeCAD design](/Pictures/FreecadScreenshotPrototypeV1.png)
![electronics picture](/Pictures/IMG_0116.png)
![electronics picture](/Pictures/IMG_0120.png)
![electronics picture](/Pictures/IMG_0121.png)



## Installation
program command for usb in BOOT 0 mode

dfu-util -a 0 -s 0x08000000:leave -D build/debug/Drone_Code.bin

or use an stlinkv2.

## Usage

## Features

## Future_Additions

## Configuration

## Known Issues
