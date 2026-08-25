ESPFlight Firmware

Open-source flight controller firmware for ESP-based drones.

ESPFlight is an open platform for makers, developers, students, and independent hardware builders who want to build and experiment with small ESP-based drones.

This repository contains the official open-source ESPFlight flight controller firmware.

ESPFlight Platform

ESPFlight provides the main building blocks needed to create an ESP-based drone:

Open-source flight controller firmware

Open hardware reference design

ESPFlight control application

Documentation and build guides

ESPFlight is a platform, not an official hardware product or kit. Makers and companies are free to create their own compatible hardware based on the platform.

Firmware v1.0.0

The first public ESPFlight firmware release includes:

250 Hz flight control loop

Roll, Pitch and Yaw PID control

IMU-based stabilization

ARM / DISARM control

Motor safety handling

Communication failsafe

Automatic landing failsafe

Tilt safety shutdown

Wi-Fi communication

UDP discovery

WebSocket control interface

Telemetry

PID configuration

Head Lock support

Beginner and Advanced control modes

Optional altitude-assisted flight

Takeoff and Landing commands

Repository Structure

Main firmware files:

espflight.ino
config.h

firmware.cpp
firmware.h

flight_control.cpp
flight_control.h

imu.cpp
imu.h

pid.cpp
pid.h

failsafe.cpp
failsafe.h

network.cpp
network.h

altitude.cpp
altitude.h

indicators.cpp
indicators.h

Additional documentation:

BUILDING.md
VALIDATION.md
ASSISTED_FLIGHT.md
LICENSE

Getting Started

1. Configure Wi-Fi

Open:

config.h

and replace:

YOUR_WIFI_SSID
YOUR_WIFI_PASSWORD

with the Wi-Fi network that will be used by the ESPFlight controller.

2. Install the Required Development Environment

See BUILDING.md for the required Arduino environment, ESP board support and libraries.

3. Build and Flash

Open:

espflight.ino

in the Arduino IDE, select the correct ESP board and upload the firmware.

4. Connect the ESPFlight Application

After the controller connects to the configured Wi-Fi network, the ESPFlight Application can automatically discover compatible ESPFlight controllers on the local network.

The application communicates with the controller using the ESPFlight communication protocol.

Current protocol version:

Protocol 2

Hardware

ESPFlight is designed around an open hardware philosophy.

The ESPFlight Hardware Reference provides a starting point for building compatible drones. Makers are encouraged to study, modify, improve and create their own hardware implementations.

The hardware reference is maintained separately from this firmware repository.

ESPFlight Application

ESPFlight Application provides the user interface for:

Drone control

ARM / DISARM

Takeoff and Landing

Beginner / Advanced modes

Head Lock

Telemetry

PID configuration

Flight information

Firmware communication

The ESPFlight Application is provided free of charge but is not open-source.

Safety

ESPFlight is an experimental and educational flight control platform.

Always test new builds carefully.

Before flight:

Verify motor direction

Verify propeller direction

Verify IMU orientation

Verify control directions

Verify ARM / DISARM operation

Verify failsafe operation

Test without propellers before powered flight

Keep people and property away from the Drone during testing

Use ESPFlight at your own risk.

Documentation

For build instructions:

BUILDING.md

For firmware validation:

VALIDATION.md

For altitude-assisted flight:

ASSISTED_FLIGHT.md

More documentation will also be available at:

https://espflight.com

Philosophy

ESPFlight exists to make experimental ESP-based flight control more accessible.

Instead of requiring makers to build every part of a small drone platform from scratch, ESPFlight provides a reusable foundation consisting of firmware, hardware references, a control application and documentation.

The goal is not to create a closed hardware ecosystem.

ESPFlight is intended to become infrastructure that independent makers, developers and companies can build on top of.

Contributing

Contributions, bug reports and technical feedback are welcome.

If you discover a firmware issue, please open a GitHub Issue with:

Hardware configuration

Firmware version

Steps to reproduce

Expected behavior

Actual behavior

Pull requests should remain focused, documented and compatible with the ESPFlight architecture.

Version

Current public firmware release:

ESPFlight Firmware v1.0.0

License

ESPFlight Firmware is released under the MIT License.

See LICENSE for the full license text.

ESPFlight

Open platform for building ESP-based drones.

https://espflight.com
