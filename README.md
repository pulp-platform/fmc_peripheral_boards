# FMC Peripheral Board

# Introduction

This repository houses source files and PDF schematics for a series of PCB designs.
The purpose of these boards is to facilitate peripheral verification of SoC designs, which is crucial for engineers who need to validate and test FPGA interfaces and achieve smooth integration with various peripheral devices.

Ideal for hardware developers and testers, these designs facilitate accurate and efficient verification processes, enhancing reliability and performance.

# Structure of the repository

This repository contains the following folders:

- **`01_Carrier_Board`**
    - The described FMC (FPGA Mezzanine Card) Carrier board is designed to enhance FPGA development platforms by adding essential peripheral interfaces. This board is tailored for developers looking to experiment with and implement storage and audio functionalities in their FPGA projects.

    **Features of the FMC Board:**

    1. **Hyper Flash:**<br>
    The Hyper Flash provides high-speed non-volatile memory storage, ideal for applications requiring fast read and write operations with a high level of endurance. This makes it suitable for high-performance data buffering or quick-boot applications.

    2. **SPI Flash:**<br>
    The SPI Flash memory is included for reliable, low-latency access, serving as an excellent resource for storing firmware or small application codes in embedded systems. Its SPI interface ensures compatibility with a vast array of FPGA boards.

    3. **I2C EEPROM:**<br>
    Equipped with I2C EEPROM, this board offers a modest amount of memory but with very low power consumption, perfect for storing configuration settings or small amounts of data that must persist through power cycles, such as device calibration data or user preferences.

    4. **2x Microphones:**<br>
    The inclusion of two microphones allows for audio input and recording capabilities. This feature is especially useful for applications involving sound capture, voice command recognition, or environmental noise analysis. The dual setup can also support stereo audio recording or be used for advanced processing techniques like beamforming or noise reduction.

    **Design and Connectivity:**<br>
    The board is designed to conform to the FMC standard, ensuring it can be used with FMC-compatible FPGA development boards. It provides a high-pin-count (HPC) connector which allows it to interface seamlessly with the host FPGA, supporting high throughput and multiple I/O options.
    The board exposes 2x M.2 type E expansion connectors which allow to connect Expansion Module specifically designed for different targets

- **`02_FMC_Test_Module`**
    - The FMC test module described here is specifically designed for enhanced debugging and signal analysis.
    It can be attached directly on top of the carrier board to monitor and probe signals interfacing with various peripherals. It can also be configured to function as a GPIO (General Purpose Input/Output) expansion when connected directly to an FPGA, providing additional flexibility in signal management and control.

- **`03_Example_Module`**
    - This folder contains a design example of an M.2 type E expansion module

- **`04_Expansion_Modules`**
    - This folder contains the design of the following M.2 type E expansion modules

        - **`Ethernet_Module`**
            - The M.2 type E module described is a specialized network interface card (NIC) that incorporates Ethernet functionalities, specifically designed to fit into compact and modular systems. This module provides Ethernet connectivity via an RJ45 connector and features the RTL8211E-VL-CG Ethernet PHY transceiver, which supports various Ethernet standards including 10Base-T, 100Base-TX, and 1000Base-T as specified by the IEEE 802.3 standards.
            For data communication between the MAC (Media Access Control) layer and the PHY (Physical Layer), the module supports both RGMII (Reduced Gigabit Media Independent Interface) and GMII (Gigabit Media Independent Interface). RGMII is used for more compact and reduced pin count requirements, suitable for high-speed data operations at 1000Base-T, while GMII is applicable for full gigabit media independent data handling, including 1000Base-T, 10Base-T, and 100Base-TX.

        - **`PWM_Module`**
            - The M.2 type module described is an innovative expansion card specifically designed to test up to 8xPWM. It exposes four Crazyflie 2.X DC motor connectors which can be dinamically connected to all the PWM outputs.

        - **`USART_Module`**
            - The M.2 type module designed for communication with the STM32L412K8T6 microcontroller using the USART (Universal Synchronous/Asynchronous Receiver/Transmitter) protocol is a specialized interface card. This module serves as a bridge to facilitate robust serial communication between the design on the FPGA and the onboard STM32L412K8T6. This folder is organized as follows:
               - `fw/`, contains the firmware source code for the STM32L412K8T6 microcontroller
               - `hw/`, containing the Altium designer source files and pdf schematics of the USART_Module

        - **`uSD_Module`**
            - The M.2 type module described is an innovative expansion card that facilitate the development, testing, and debugging of Secure Digital Input Output (SDIO) protocol implementations, allowing engineers to easily interface with microSD cards and ensure compliance with SDIO standards.

- **`05_FMC_4_Hyperram_32MB`**
    - The FMC board described here is a specialized module designed to significantly enhance the memory capabilities.
    It features four 8MB HyperRAMs, providing a total of 32MB of additional low-pin-count RAM. 

# License

The Altium designer source files are released under Solderpad v0.51 (`SHL-0.51`) (see `01_Carrier_Board/LICENSE`)

The `04_Expansion_Modules/USART_Module/fw/`directory contain third-party sources that come with their own
licenses. See the respective folders and source files for the licenses used.

# Publications

If you use these PCBs in your work, please cite us:

```
@INPROCEEDINGS{9607006,
  author={Valente, Luca and Rossi, Davide and Benini, Luca},
  booktitle={2021 IFIP/IEEE 29th International Conference on Very Large Scale Integration (VLSI-SoC)}, 
  title={Hardware-In-The Loop Emulation for Agile Co-Design of Parallel Ultra-Low Power IoT Processors}, 
  year={2021},
  volume={},
  number={},
  pages={1-6},
  keywords={Program processors;Computational modeling;Emulation;Very large scale integration;Tools;Performance analysis;Sensors;Verification;FPGA emulation;IoT;CNN},
  doi={10.1109/VLSI-SoC53125.2021.9607006}}
```
