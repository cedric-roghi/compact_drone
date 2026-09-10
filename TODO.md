### Prototype V1 checklist

- ✅ PCB design done
- ✅ Drone mechanical structure done
- ✅ ELRS receiver connected and receiving
- ✅ Flight controller receiving packets from ELRS receiver
- ✅ ELRS transmitter receives telemetry packet from drone
- ✅ IMU (Just the accelerometer and gyroscope) is working and sensor fusion to get stable data
- ✅ Motor control file to limit motor operations within safe  physical ranges (ie. dont move servo more than swashplate can physically handle)
- 🚧 PID controllers for yaw, pitch and roll
- ⬜ Add magnetometer and barometer functionality to the flight controller code 
- ⬜ Persistent flight controller settings to be stored in stm32 flash memory
- ⬜ PyQT application to adjust flight controller settings without flashing the microcontroller. 
