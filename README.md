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
The design idea for this drone is not my own. Despite it being a rare design I got inspiration from projects such as [backcountrydrones](https://www.youtube.com/watch?v=82phDhYGT_0) and [Tdrone](https://github.com/ShenZhenAccelerationTechCo/Tdrone). Backcountry Drones guys have now changed to Ascent Aerosystems and make military/law enforcement and industrial coaxial drones. The NASA mars copter is another example of a contra rotating drone design.

The idea is that the propellers can fold down which means the drone can fit in smaller storage solutions. Not only does the the tube shape enable a smaller size when storing the drone but it also helps make it more rugged and resistant to poor treatment. A final goal for this project would be to have a drone I can strap to the outside of a hiking pack without having to worry about damage. 

Using a coaxial design is more efficient in theory. Having two large propeller spinning about the center of the drone increases the lifting disc area compared to having four propellers next to each other that will waste space. A coaxial drone with a tube shape also reduces its surface area when tilting forwards to move making it more streamlined than a quadcopter that shows more surface area to drag when moving forward. I recommend reading this paper from Ascent Aerosystems about all the advantages and its got diagrams (https://ascentaerosystems.com/wp-content/uploads/2024/12/Coaxial-Advantage-White-Paper-Ascent-AeroSystems.pdf).

Figures from the Ascent Aerosystems paper (https://ascentaerosystems.com/wp-content/uploads/2024/12/Coaxial-Advantage-White-Paper-Ascent-AeroSystems.pdf).
<img width="947" height="389" alt="Screenshot_20260910_214609" src="https://github.com/user-attachments/assets/ec02294e-31c1-4dc9-8513-ab6d6a43b3a5" />
<img width="971" height="874" alt="Screenshot_20260910_215506" src="https://github.com/user-attachments/assets/eca8ac8c-41ee-40d6-9e26-2b3267afdb6f" />

Some applications for the drone would be for survey work in remote areas, filming, compact and rapid deployment via stackable launch cells. In theory it could even be launched from a plane like the Atlantique2 which already has tube shaped launchers on the belly for sonars. 

- STM32f411:
    - Same MCU I used for my turret project. Its fast, powerful, and has many timers for pwm output. Since I used it for the turret it makes the PCB development for the drone much faster.
    - Another great advantage of this MCU is that it can do up to 100MHz clock speed which gives me plenty of headroom for high speed communication (UART, I2C,  and heavy RTOS tasks to run without too much 

Progress Update (09/09/26):
- The yaw controls are working. I have found some PID gains that seem stable at low and high rotor speeds.
- I used hot glue to fix the servo motors more securely and printed some new link rods to reduce play in the swash-plate.
- The swash-plate currently has too much play where the drone cannot correct error when its too small and leads to uncontrollable oscillation.

https://github.com/user-attachments/assets/415317f5-2f4a-4eaa-9d52-048ebe863607

Below are some pictures of the PCB and 3D design to be printed.
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
