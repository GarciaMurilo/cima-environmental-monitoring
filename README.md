# CIMA – Environmental Monitoring Systems

A collection of embedded devices, data acquisition systems and software tools developed for environmental monitoring as part of a research project at the Center for Environmental Information and Monitoring (CIMA), Federal University of Rio de Janeiro (UFRJ).

## About the Project

The project focuses on developing accessible environmental monitoring solutions by integrating electronic instrumentation, embedded systems, data acquisition and visualization.

It currently comprises three main components: a water quality monitoring device, an air quality monitoring device and a web-based platform for environmental data visualization.

## Project Components

### 1. GarCIMA – Water Quality Monitoring Device

An Arduino-based device developed to monitor water quality through three parameters:

- pH
- Temperature
- Turbidity

The system integrates sensors for data acquisition and provides an accessible approach to environmental monitoring.

**Status:** Completed implementation.

### 2. Air Quality Monitoring

An ESP32-based monitoring device designed for environmental data acquisition, integrating sensors for particulate matter and meteorological measurements.

The system is being developed to collect and record environmental data for subsequent analysis and visualization.

**Status:** Under development.

### 3. Environmental Data Visualization Platform

A web-based platform designed to display and share environmental measurements collected by monitoring devices.

Current features include:

- Interactive map visualization using Leaflet.
- Integration with Supabase for data retrieval.
- Georeferenced monitoring stations.
- Interactive pop-ups displaying environmental measurements.
- Indicative water quality classification based on pH and turbidity.

**Status:** Prototype. The current implementation includes database integration, geospatial visualization and simulated sensor updates.

## Technologies

**Embedded Systems**
- Arduino
- ESP32
- Electronic sensors and instrumentation
- C/C++

**Data Acquisition and Integration**
- Sensor data acquisition
- Supabase
- Database integration

**Web Development and Visualization**
- HTML5
- CSS3
- JavaScript
- Leaflet.js

## Repository Structure

```text
CIMA/
├── water-monitor/
│   └── Water monitoring device firmware
├── air-monitor/
│   └── Air monitoring device firmware
├── visualization-platform/
│   └── Web application prototype
└── README.md
```

## Project Context

Developed within CIMA (Center for Environmental Information and Monitoring), at the Federal University of Rio de Janeiro (UFRJ), as part of a research initiative focused on accessible environmental monitoring, technological development and citizen science.

## Future Improvements

- Integration of live sensor data with the visualization platform.
- Historical environmental data visualization.
- Expansion of monitored environmental parameters.
- Improved data validation and reliability.
- Integration of water and air monitoring systems into a unified platform.

## Author

Murilo Garcia  
Electronics and Computer Engineering – UFRJ
