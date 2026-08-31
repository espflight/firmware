<div align="center">

ESPFlight Firmware

Open-source flight-control firmware for ESP-based drones

Learn. Experiment. Build.





Website · Documentation · Hardware Reference · Brand Policy

</div>

Overview

ESPFlight Firmware is the open-source flight-control core of the ESPFlight platform.

It is designed for enthusiasts, school and university students, teachers, instructors, Makers, developers, and engineers who want to learn about, build, experiment with, and develop ESP-based drones.

ESPFlight provides a reusable starting point instead of requiring every builder to develop the complete flight stack from zero. The platform combines:

Component

Role

ESPFlight Firmware

Flight control, stabilization, safety logic, networking, telemetry, and configuration

ESPFlight Hardware Reference

Open hardware starting point for compatible drone designs

ESPFlight Application

Free companion application for control, telemetry, setup, and tuning

Protocol & Documentation

Communication interface plus build, setup, validation, and usage guidance

ESPFlight is a platform, not an official hardware kit or a fixed drone design. The firmware and Hardware Reference are intended to be studied, modified, and built upon.

A project can remain educational, experimental, or personal. Builders who later choose to take a mature design further may also create independently branded kits or products under the applicable licenses.

Firmware v1.0.0

The first public ESPFlight firmware release provides the following capabilities:

Area

Capability

Description

Flight control

250 Hz control loop

Runs the primary flight-control loop at 250 Hz

Flight control

Roll / Pitch / Yaw PID

Closed-loop attitude control for the three rotational axes

Stabilization

IMU-based stabilization

Uses IMU data for stabilized flight behavior

Control modes

Beginner / Advanced

Provides two control profiles for different user experience levels

Safety

ARM / DISARM

Explicit motor arming and disarming state handling

Safety

Motor safety handling

Applies firmware-side safeguards around motor operation

Safety

Communication failsafe

Responds to loss of the active control connection

Safety

Automatic landing failsafe

Supports an automatic landing response during applicable failsafe conditions

Safety

Tilt safety shutdown

Stops unsafe operation when excessive tilt conditions are detected

Networking

Wi-Fi communication

Connects the flight controller to the local wireless network

Networking

UDP discovery

Allows compatible clients to discover ESPFlight controllers on the network

Networking

WebSocket control interface

Provides the real-time communication channel used for control and state exchange

Protocol

Protocol 2

Uses ESPFlight communication protocol version 2

Telemetry

Live telemetry

Reports flight and system information to compatible clients

Configuration

PID configuration

Allows supported PID values to be configured through the platform interface

Flight features

Head Lock

Provides heading-relative control support

Flight features

Takeoff / Landing commands

Supports platform commands for takeoff and landing workflows

Assisted flight

Optional altitude assistance

Provides the optional altitude-assisted flight functionality documented in ASSISTED_FLIGHT.md

Important: ESPFlight is an experimental and educational flight-control platform. A listed feature should not be interpreted as certification or as a guarantee that a particular custom build is safe or production-ready.

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

Quick Start

1. Configure Wi-Fi

Open config.h and replace:

YOUR_WIFI_SSID
YOUR_WIFI_PASSWORD

with the Wi-Fi network that will be used by the ESPFlight controller.

2. Prepare the development environment

See BUILDING.md for the required Arduino environment, ESP board support, and libraries.

3. Build and flash

Open espflight.ino in the Arduino IDE, select the correct ESP board, and upload the firmware.

4. Connect the ESPFlight Application

After the controller joins the configured Wi-Fi network, the ESPFlight Application can automatically discover controllers running compatible ESPFlight Firmware on the local network.

The application communicates with the controller through the ESPFlight communication protocol.

Current protocol: Protocol 2

For complete setup and validation guidance, use the documentation linked below before attempting powered flight.

Hardware

ESPFlight follows an open-hardware philosophy.

The ESPFlight Hardware Reference is a known starting point for compatible drone designs. It helps builders understand the electrical design and firmware-facing connections before changing the PCB, frame, components, or overall implementation.

The reference is not the only possible ESPFlight-compatible hardware design.

Hardware: https://espflight.com/hardware/

Hardware license: https://espflight.com/licenses/hardware/

ESPFlight Application

The ESPFlight Application is the companion interface for supported ESPFlight functionality, including:

drone control;

ARM / DISARM;

takeoff and landing;

Beginner / Advanced modes;

Head Lock;

telemetry;

PID configuration;

flight information;

firmware communication.

The ESPFlight Application is provided free of charge but is not open source. It is distributed separately from the firmware and is not covered by the MIT License used for this repository.

Learn, Experiment, Build

ESPFlight Firmware is intentionally readable and modifiable.

It can be used to study flight-control architecture, understand how sensors and PID control interact, explore networking and telemetry, examine failsafe behavior, and adapt the firmware to compatible custom hardware.

The intended progression is simple:

Learn → Experiment → Build → Develop → Create

The first goal is learning through real experimentation and building. How far a project goes after that is up to its creator.

A mature project may optionally become an independently branded kit or product. The MIT License permits commercial use, modification, and redistribution subject to its terms, but commercial use is not the primary purpose of ESPFlight.

Independent Hardware and Branding

Using ESPFlight Firmware does not make an independent board, drone, kit, or product an official ESPFlight hardware product.

Independent projects should use their own primary product name and identity and describe their relationship with ESPFlight accurately. Examples include:

Compatible with ESPFlight

Based on ESPFlight

Built with ESPFlight

Works with ESPFlight Application

The MIT License covers the firmware code; it does not grant unrestricted rights to the ESPFlight name, logo, Partner marks, or other protected brand assets.

Brand-use rules: https://espflight.com/brand-policy/

Safety

ESPFlight controls real motors and flying hardware. Incorrect configuration, hardware faults, software defects, communication loss, battery problems, or improper testing can cause injury or property damage.

Before powered flight:

verify motor direction;

verify propeller direction;

verify IMU orientation;

verify control directions;

verify ARM / DISARM behavior;

verify failsafe behavior;

perform bench testing without propellers where appropriate;

keep people, animals, and property clear of the drone during testing.

Always test new hardware and firmware cautiously and follow applicable local laws and safety requirements.

Do not treat experimental functionality as production-ready unless it has been validated for the specific hardware and use case.

Use ESPFlight at your own risk.

Documentation

Guide

Purpose

BUILDING.md

Development environment, dependencies, build, and flashing instructions

VALIDATION.md

Firmware and hardware validation workflow

ASSISTED_FLIGHT.md

Optional altitude-assisted flight behavior and guidance

ESPFlight Documentation

Platform-level documentation and guides

Contributing

Contributions, bug reports, compatibility testing, and technical feedback are welcome.

When reporting a firmware issue, include enough information to reproduce and evaluate the problem:

hardware configuration;

firmware version;

steps to reproduce;

expected behavior;

actual behavior;

relevant logs, measurements, or observations when available.

Pull requests should remain focused, documented, and compatible with the ESPFlight architecture.

Version

Current public firmware release:

ESPFlight Firmware v1.0.0

License

ESPFlight Firmware is released under the MIT License.

The MIT License permits personal, educational, experimental, and commercial use, including modification and redistribution, provided its conditions are followed and the required copyright and license notices are preserved.

See LICENSE for the controlling license text.

ESPFlight firmware license guide: https://espflight.com/licenses/firmware/

The firmware license does not grant unrestricted rights to ESPFlight trademarks or brand assets. See the ESPFlight Brand Policy for brand-use rules.

<div align="center">

ESPFlight

Learn. Experiment. Build.

Open platform for learning, experimenting, building, and developing ESP-based drones.

https://espflight.com

</div>
