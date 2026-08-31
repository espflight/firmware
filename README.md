<div align="center">

ESPFlight Firmware

Open-source flight-control firmware for ESP-based drones

Learn. Experiment. Build.





Website · Documentation · Hardware Reference

</div>

Firmware v1.0.0

ESPFlight Firmware is the open-source flight-control core of ESPFlight. It handles stabilization, motor control, safety logic, telemetry, configuration, and communication with the ESPFlight Application.

<table>
  <thead>
    <tr>
      <th>Area</th>
      <th>Features</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>Flight control</strong></td>
      <td>250 Hz control loop · Roll / Pitch / Yaw PID · IMU-based stabilization</td>
    </tr>
    <tr>
      <td><strong>Control modes</strong></td>
      <td>Beginner · Advanced · Head Lock</td>
    </tr>
    <tr>
      <td><strong>Flight functions</strong></td>
      <td>Takeoff · Landing · Optional altitude-assisted flight</td>
    </tr>
    <tr>
      <td><strong>Safety</strong></td>
      <td>ARM / DISARM · Motor safety handling · Communication failsafe · Automatic landing failsafe · Tilt safety shutdown</td>
    </tr>
    <tr>
      <td><strong>Communication</strong></td>
      <td>Wi-Fi · UDP discovery · WebSocket control interface · Protocol 2</td>
    </tr>
    <tr>
      <td><strong>Telemetry & setup</strong></td>
      <td>Live telemetry · PID configuration · Flight-state reporting</td>
    </tr>
  </tbody>
</table>

ESPFlight is an experimental and educational flight-control platform. Validate every hardware configuration carefully before powered flight.

Quick Start

1. Configure Wi-Fi

Open config.h and set:

YOUR_WIFI_SSID
YOUR_WIFI_PASSWORD

2. Prepare the build environment

Follow BUILDING.md to install the required Arduino environment, ESP board support, and libraries.

3. Build and flash

Open espflight.ino in the Arduino IDE, select the correct ESP board, and upload the firmware.

4. Connect the ESPFlight Application

After the controller joins the configured Wi-Fi network, the ESPFlight Application can discover controllers running compatible ESPFlight Firmware on the local network.

Current communication version: Protocol 2.

Repository

espflight.ino              Arduino entry point
config.h                   User and hardware configuration
firmware.cpp / .h          Firmware core
flight_control.cpp / .h    Flight-control loop
imu.cpp / .h               IMU handling
pid.cpp / .h               PID control
failsafe.cpp / .h          Safety and failsafe logic
network.cpp / .h           Wi-Fi and communication
altitude.cpp / .h          Altitude-assisted flight
indicators.cpp / .h        Status indicators

Documentation included with the release:

BUILDING.md — build environment and flashing

VALIDATION.md — firmware and hardware validation

ASSISTED_FLIGHT.md — altitude-assisted flight

Safety

ESPFlight controls real motors and flying hardware. Incorrect configuration, hardware faults, software defects, communication loss, or battery problems can cause injury or property damage.

Before powered flight:

test on the bench without propellers where appropriate;

verify motor and propeller direction;

verify IMU orientation and control directions;

verify ARM / DISARM and failsafe behavior;

keep people, animals, and property clear of the drone during testing.

Use ESPFlight at your own risk and follow applicable local laws and safety requirements.

License

ESPFlight Firmware is released under the MIT License. See LICENSE for the complete terms.

The MIT License applies to the firmware code and does not grant rights to ESPFlight trademarks or brand assets.

<div align="center">

ESPFlight Firmware v1.0.0
https://espflight.com

</div>
