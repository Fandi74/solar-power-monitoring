***Solar Monitoring System***

This repository contains the source code for a Solar Monitoring System using ESP32 for data acquisition, storage, and logging. The system reads data from solar panels, calculates power values, and logs the information locally or sends it to the cloud.

**Features**
- Real-time monitoring of voltage, current, and power from two solar panels.
- Data logging to an SD card in JSON format.
- Time synchronization with NTP servers.
- LCD display for live data visualization.
- Modular design with FreeRTOS tasks.

**Project Structure**
- main.cpp: ESP32-based code for the actual deployment.
- hardware/: Contains schematics, footprints PCBs, Simulation.

**Software Dependencies**

- PlatformIO (for development).
- Libraries:
  - Adafruit_ADS1X15
  - ArduinoJson
  - LiquidCrystal_I2C
  - WiFi
  - HTTPClient
  - RTC3231
- Proteus for simulation (using Arduino Uno).

**Future Enhancements**
- Implement cloud integration for remote monitoring.
- Add additional panels and sensors for scalability.
- Enhance data visualization with a web dashboard.

****I Made all of this just for fun****
Feel free give any feedback

**3D PCB**

![image](https://github.com/user-attachments/assets/385ede2e-5562-44b3-b791-395de2d55ff2)

**Schematic**

![Solar Panel Monitoring_page-0001](https://github.com/user-attachments/assets/2244f3c1-5ff8-4796-b8df-9c2aaa513c88)

