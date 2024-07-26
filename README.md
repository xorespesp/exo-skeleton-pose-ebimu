# exo-skeleton-pose-ebimu
Exo skeleton pose estimator via `EBIMU24GV52` sensors

## Build Environment
- Win11 23H2 x64 OS
- MSVC Compiler (2022 x64)
- Vcpkg (2023-11-16-4c1df40a3c5c5e18de299a99e9accb03c2a82e1e)

## Getting Started
To get started with this project, follow these steps:

1. Clone this repository
```bash
$ git clone https://github.com/xorespesp/exo-skeleton-pose-ebimu.git
```

2. Install [Vcpkg](https://github.com/microsoft/vcpkg) (and make sure the `VCPKG_ROOT` environment variable is correct)

3. Perform CMake build procedure

## Notes
```
When receiving data from multiple sensors on a single receiver:
The RF channel ID of the receiver and the RF channel IDs of the sensors must all be the same.
The RF sensor IDs between sensors must be different from each other.

To check the receiver's RF channel ID
  -> <cfg> command

To set the receiver's RF channel ID
  -> <sch> command

To check the sensor's RF channel ID and RF sensor ID
  -> <??cfg> command
  (Transmit with all other sensors turned OFF except for the target sensor)

To set the sensor's RF channel ID
  -> <??sch> command
  e.g) To set the sensor's RF channel ID to 100: <??sch100>
  (Transmit with all other sensors turned OFF except for the target sensor)

To set the sensor's RF sensor ID
  -> <??sid> command
  e.g) To set the sensor's RF sensor ID to 99: <??sid99>
  (Transmit with all other sensors turned OFF except for the target sensor)
```
