# ESP32-Based WiFi CSI Research Tools
This repository provides a simple and affordable software and hardware interface for capturing WiFi CSI for research applications. It provides guidance on setting up hardware and tools for automatically configuring firmware. In addition, it provides a web interface for logging WiFi CSI data. This system can be extended with its built-in ROS2 capability for automated systems. For a full usage guide, visit the [wiki](https://github.com/kuederleR/CSIModule/wiki)

## Web Interface
A dockerized web user interface is included in this repo, which is the primary tool for logging and collecting data from the ESP32 modules. This enables users to generate log files in CSV, JSON, or NumPy formats. It shows debug details like packet counts and a display of recent CSI packet data. In addition, users can enable a ROS2 publisher to stream CSI data. 

Checkout the [Web UI Page](https://github.com/kuederleR/CSIModule/tree/main/webui) to learn about this tool and how to use it. 
<img width="600" src="https://github.com/user-attachments/assets/50b99a1b-cf93-4e11-b5ae-a6206fc7811a" />



## Firmware Tools
### ESP32 Flash Tool
Enabling WiFi CSI on ESP32 devices requires use of the ESP-IDF tool and specific configurations. This tool provides an automated, containerized process for flashing ESP32 devices for CSI use cases. To learn more, checkout the [wiki](https://github.com/kuederleR/CSIModule/wiki/ESP32-Flash-Tool).

### Raspberry Pi Auto Config
The ESP32 CSI modules are designed to be handed around between researchers, so a Raspberry Pi acts as a stand-alone computer for serving the web-based interface for capturing and downloading collected data. This repository contains a dedicated tool for configuring a Raspberry Pi as a WiFi access point and ESP32 interface, without ever having to connect to the Raspberry Pi's command line. Follow [this guide](https://github.com/kuederleR/CSIModule/wiki/Raspberry-Pi-Configuration) to see it in action.

