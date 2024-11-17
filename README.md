# OBD2-with-CAN-protocol
OBD-II with CAN Protocol
Project Scope
Objective: Extract specific diagnostic data such as fuel level, engine speed (RPM), vehicle speed, and other parameters from a car's OBD-II system.
Target Vehicles: Hyundai i10 and Honda City, with plans for other vehicles for validation.
Hardware:
Microcontrollers: STM32 (Discovery Board, Blue Pill) and ESP32-WROOM-32V.
CAN Transceivers: SN65HVD230 (replaced MCP2551 for better compatibility).
Tools: CAN analyzer (hardware requested for debugging and verification).
Progress Overview
Hardware Setup:

Configured the STM32 and ESP32 boards for CAN communication.
Successfully interfaced the SN65HVD230 transceiver for communication with OBD-II.
CAN Communication Configuration:

Baud rate set to 500 kbps to match OBD-II standards.
Used identifier 0x7DF for broadcast queries to the OBD-II system.
Configured filters and masks, including testing exact match filters (e.g., 0x7FF).
Data Retrieval:

Successfully retrieved key PIDs (Parameter IDs) from Hyundai i10:
Engine RPM
Vehicle Speed
Intake Air Temperature
Throttle Position
Mass Air Flow Rate
Run Time Since Engine Start
Ambient Air Temperature
Challenges encountered in retrieving specific fuel-related data, though speed and RPM responses were consistent.
Software Development:

Developed code to parse OBD-II responses, extracting relevant data using index-based parsing.
Implemented conditional checks for specific PIDs like speed, RPM, and fuel level.
Optimized loops for response parsing and corrected delays in data retrieval.
Debugging Efforts:

Debugged issues with:
Fuel data responses (still ongoing for Honda City).
SWV (Serial Wire Viewer) output configuration for STM32CubeIDE.
DFU (Device Firmware Upgrade) mode on ST-Link causing disconnection issues.
Validation:

Data acquisition verified on Hyundai i10.
Awaiting Honda City validation; requires a DBC file for accurate data decoding.
Challenges Faced
Fluctuating Data:
Inconsistent speed and fuel-level readings during initial tests.
Fuel Data Retrieval:
Limited or no responses for PIDs related to:
Fuel Level
Engine Coolant Temperature
Fuel Rail Pressure
Focused efforts on identifying and resolving these issues.
Tool Limitations:
Need for a CAN analyzer to debug and verify data traffic efficiently.
Dependency on a DBC file for Honda City to map CAN signals correctly.
