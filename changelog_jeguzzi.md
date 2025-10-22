# Changelog in fork https://github.com/jeguzzi/enki

## Branch proximity_sensors

Cleaner implementation of prox and prox-comm based on `ircomm` branch.

- added IRComm
- added IRSensorRealistic
- exposed number of rays and aperture in IRSensor constructor
- added a different range and search_range in IRSensor
- added virtual methods to specialize initialization and finalization of global interactions