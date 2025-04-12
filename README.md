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
### Build
```bash
mkdir computer_side/build  \ 
cd computer_side/build     \
cmake ..                   \
cmake --build . --parallel
```
### Run

From build folder:
1. Set IP on manipulator
2. Start controlling program (control program must be on localhost:1245 and this one would be on localhost:1246)
3. Start compiled program
```bash
./FRI_control
```


## Hardware
- Kuka LBR IIWA 7
- Comuter with ethernet port
- Ethernet cable

## Project Structure
- computer_side - Software running on the computer side (currently operational)
- manipulator_side - Software running on the manipulator side (currently non-operational)

  
