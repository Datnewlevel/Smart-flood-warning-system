# 🌊 Smart Flood Warning System

A project to design and build a smart flood warning system model using ESP32 microcontrollers and LoRa wireless communication.

---

## 🎯 **Objective**

To build a system capable of real-time water level measurement and delivering timely warnings to residents through various methods (LEDs, speakers, Telegram messages) to minimize flood-related damages.

---

## ⚙️ **How It Works**

The system comprises two main stations:

1.  **Measuring Station:**
    *   Uses an ESP32 and an ultrasonic sensor to continuously measure the water level.
    *   Transmits the data to the Control Station via LoRa wireless communication.

2.  **Control Station:**
    *   Uses an ESP32-S3 to receive data from the Measuring Station.
    *   Displays warning levels on an LCD and through status LEDs.
    *   Activates a loudspeaker to play a pre-recorded warning.
    *   Sends alert notifications to a configured Telegram channel.

---

## 🖼️ **System Diagrams**

### Block Diagram
| Measuring Station Block Diagram |
| :-----------------------------: |
| ![Block Diagram](./so_do_khoi_Tramdo.PNG) |

### Measuring Station (Trạm Đo)
| Schematic | PCB Layout | 3D View |
| :---: | :---: | :---: |
| ![Measuring Station Schematic](./sch_tram_do.png) | ![Measuring Station PCB](./pcb_tram_do.png) | ![Measuring Station 3D View](./3d_view_Tramdo.PNG) |

### Control Station (Trạm Kiểm Soát)
| Schematic | PCB Layout | 3D View |
| :---: | :---: | :---: |
| ![Control Station Schematic](./sch_tram_kiem_soat.png) | ![Control Station PCB](./pcb_tram_kiem_soat.png) | ![Control Station 3D View](./3d_view_Tramkiemsoat.png) |

### Pinout Diagrams
| ESP32-D Pinout | ESP32-S3 DevKitC-1 Pinout |
| :---: | :---: |
| ![ESP32D Pinout](./esp32d.jpg) | ![ESP32-S3 Pinout](./esp32-s3_devkitc-1_pinlayout_v1.1.jpg) |

---

## 📌 **Hardware Components**

### Measuring Station

| Component | Details |
| :--- | :--- |
| **Microcontroller** | ESP32-WROOM-32D |
| **Sensor** | Waterproof Ultrasonic Sensor SR04T |
| **Communication** | LoRa Ra-02 (SX1278) |
| **Power** | 2 x 18650 Batteries, TP4056 Charging Module |
| **Other** | MOSFET 2N7000, Capacitors, Resistors, LEDs, Switch |

### Control Station

| Component | Details |
| :--- | :--- |
| **Microcontroller** | ESP32-S3 |
| **Communication** | LoRa Ra-02 (SX1278) |
| **Display** | LCD with I2C, 3x Status LEDs |
| **Audio** | Loudspeaker for audio alerts |
| **Other** | Resistors |

---

## 💻 **Source Code** (Use ESP-IDF)

*   **Measuring Station:** `Code/Code measuring station/Tram_do`
*   **Control Station:** `Code/Code main station/Tram_kiem_soat`

---

## 🛠️ **Setup and Usage**

*(You can add detailed instructions here on how to compile and flash the code to the ESP32s, as well as how to configure Wi-Fi and the Telegram token for the control station.)*