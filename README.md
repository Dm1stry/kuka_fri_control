# Kuka FRI control
A Linux-based project aimed at teleoperation of the KUKA LBR IIWA collaborative manipulator with the Kuka Sunrise 1.7 OS via UDP. It allows setting target angles on the manipulator and receiving joint torque data from it.

## Credentails
Made in Innopolis University
By:
- Alik Valiullin
- Dmitrii Mistrikov
- Ruslan Damindarov

## Dependencies
(Linux system)
- cmake
- Eigen 3.3
- nlohamnn_json 3.11.3
- Boost 1.71

### Installation of dependencies (on ubuntu)
```bash
sudo apt install -y libboost-dev nlohmann-json3-dev libeigen3-dev cmake
```

## How to use

## Hardware
- Kuka LBR IIWA 7
- Comuter with ethernet port
- Ethernet cable

## Project Structure
- computer_side - Software running on the computer side (currently operational)
- manipulator_side - Software running on the manipulator side (currently non-operational)

  
