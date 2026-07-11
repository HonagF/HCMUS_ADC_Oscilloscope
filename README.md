[Project Information](https://docs.google.com/document/d/1wfRh9YXK25s_UUH2y_-TGw_fSxXpMR-pIrb6TqXOmT0/edit?tab=t.250vvrg7rc1h)


https://github.com/user-attachments/assets/9503c086-c768-495d-8c4e-90152d3b3db1

 ## I. Introduction
### Project Overview
This repository hosts a self-developed **Digital Storage Oscilloscope (DSO)** project. It was built as a core team assignment for the Microcontrollers course within the Faculty of Electronics and Telecommunications at the VNU-HCM University of Science (Academic Year 2025-2026).

The architecture utilizes a distributed processing layout across a hybrid multi-microcontroller setup to ensure smooth, high-speed performance:
* **Data Acquisition Block (STM32F303RE):** Handles high-speed analog signal sampling through its internal 12-bit ADC, utilizing DMA to offload data storage from the CPU pipeline.
* **Processing & Display Block (ESP32):** Manages incoming data packets over SPI, executes processing tasks, hosts the web interface, and handles graphical UI rendering.
* **Human-Machine Interface (HMI):** Displays waveforms smoothly on a 3.5-inch TFT touch screen powered by the ILI9488 controller (320x480 pixel resolution).

The system combines concepts from Basic Electronics, Digital Electronics, Analog Electronics, and Microcontrollers to deliver a functional measurement tool tailored for educational settings and fundamental laboratory testing.

---

Our **DualCore Scope** team:

| Name | Roles & Responsibilities |
| :--- | :--- |
| [**Vũ Quốc Hùng**](https://github.com/HungVulkan) | <ul><li>Team Leader</li><li>Hardware Development</li><li>GitHub Management</li></ul> |
| [**Nguyễn Minh Hoàng**](https://github.com/HonagF) | <ul><li>Firmware (STM32, ESP32)</li><li>System Functions & UI Display</li><li>GitHub Management</li></ul> |
| **Đặng Hữu Trung Kiên** | <ul><li>3D design</li><li>Slide Preparation</li></ul> |
| **Đỗ Hoàng Đạt** | <ul><li>Firmware (ESP32), web interface</li><li>Hardware Assistance</li></ul> |

 ## II. Hardware

| Component | Model | Quantity |
|---|---|---|
| Microcontroller | STM32F303RE (Nucleo-64) | 1 |
| WiFi MCU | ESP32 DevKit V1 | 1 |
| Display | ILI9488 3.5" TFT SPI | 1 |
| Signal input | BNC Female Header | 2 |
| User input | Momentary pushbutton | 3 |
| User input | Latching pushbutton | 2 |
| User input | Rotary encoder | 1 |
| Signal conditioning | Custom signal adjustment circuit | 2 |
| Power | 7.4V power supply → 8.4V boosted | 1 |

<p align="center">
  <img src="https://github.com/user-attachments/assets/1656fa71-f535-4f75-9b79-cdda9fc12dd3" alt="NUCLEO-F303RE Board" width="25%"/>
</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/be0393f5-5c70-4d16-99f6-d63a9f7baf6b" alt="ESP32_DevKit_V1" width="50%"/>
</p>
<p align="center">
<img src="https://github.com/user-attachments/assets/4c54a986-af12-4d0c-8bfb-2371441a5be4" width="80%"/>
</p>

Although we designed this circuit, when we built it on a perfboard, it worked initially but broke down a few days later. Due to time constraints, we had to abandon it. We hope you will carry on this work in our place.
<p align="center">
<img width="549" height="266" alt="image" src="https://github.com/user-attachments/assets/175a57ee-710c-4d1e-96d9-d2e2e369a887" alt="Signal_adjustment_circuit" width="150%"/>
</p>

---
 ## III. Software
For the software, besides the signal processing algorithm on the STM32F303RE, we designed an interface on a 3.5-inch TFT SPI display using the ILI9488. Due to local stock limitations, a touchscreen version was provisionally used instead of the standard non-touch model. (Users could also opt for more common alternatives like the 2.8-inch ILI9341 to fit their needs)
<p align="center">
<img width="1014" height="349" alt="image" src="https://github.com/user-attachments/assets/229210b9-efe4-4bc7-a90d-ae7fef2adf83" />
</p>

Additionally, we have developed an interface on the ESP32, along with other features, making it open-source for further development.
<p align="center">
<img width="1347" height="569" alt="image" src="https://github.com/user-attachments/assets/bbcd02cd-5e4e-4657-8f03-542adac14dfd" />
</p>
