# Block Diagram – Measurement & Display System

## Hardware Block Diagram

```mermaid
flowchart LR
    A["BNC Female\nHeader"]
    B["Signal adjustment\ncircuit\nFilter + level shifting"]
    C["STM32F303RE\n12-bit ADC\nSampling + storage"]
    D["ESP32\nData processing\nUI rendering"]
    E["Display\nILI9488\n3.5&quot; TFT SPI"]

    A -->|"Analog"| B
    B -->|"ADC"| C
    C -->|"SPI"| D
    D -->|"SPI"| E

    style A fill:#D3D1C7,color:#2C2C2A,stroke:#5F5E5A
    style B fill:#9FE1CB,color:#04342C,stroke:#0F6E56
    style C fill:#B5D4F4,color:#042C53,stroke:#185FA5
    style D fill:#B5D4F4,color:#042C53,stroke:#185FA5
    style E fill:#F5C4B3,color:#4A1B0C,stroke:#993C1D
```

## Full System Diagram

```mermaid
flowchart TB
    subgraph INPUT ["Input"]
        A["BNC Female Header"]
        B["Signal adjustment circuit\nFilter + level shifting"]
        A -->|"Analog signal"| B
    end

    subgraph MCU ["Microcontrollers"]
        C["STM32F303RE\n12-bit ADC · Sampling + storage"]
        D["ESP32\nData processing · UI rendering"]
        C -->|"SPI (master → slave)"| D
    end

    subgraph OUTPUT ["Output"]
        E["Display ILI9488\n3.5&quot; TFT SPI"]
    end

    subgraph CTRL ["User control"]
        F["Button + Encoder\nGPIO / interrupt"]
    end

    subgraph PWR ["Power supply"]
        P["4.2V input → 7V boost converter\nSTM32 · ESP32 · ILI9488"]
    end

    B      -->|"ADC input"| C
    D      -->|"SPI"| E
    F      -->|"GPIO / interrupt"| C
    PWR    -.->|"3.3V"| MCU
    PWR    -.->|"3.3V"| OUTPUT
    PWR    -.->|"3.3V"| INPUT

    style A fill:#D3D1C7,color:#2C2C2A,stroke:#5F5E5A
    style B fill:#9FE1CB,color:#04342C,stroke:#0F6E56
    style C fill:#B5D4F4,color:#042C53,stroke:#185FA5
    style D fill:#B5D4F4,color:#042C53,stroke:#185FA5
    style E fill:#F5C4B3,color:#4A1B0C,stroke:#993C1D
    style F fill:#9FE1CB,color:#04342C,stroke:#0F6E56
    style P fill:#FAC775,color:#412402,stroke:#854F0B
```

## Protocol Summary

| Connection | Protocol | Direction |
|---|---|---|
| BNC → Signal adjustment | Analog | → |
| Signal adjustment → STM32F303RE | ADC input | → |
| STM32F303RE → ESP32 | SPI (master → slave) | → |
| ESP32 → ILI9488 | SPI | → |
| Button / Encoder → STM32F303RE | GPIO / interrupt | → |
| Power supply → All | 7V boosted | → |
