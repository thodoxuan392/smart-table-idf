# Smart Table

This repository contains source code for Smart Table firmware.

## Prerequisites

-   ESP-IDF 4.4 or 4.5

## Features

-   Collect data from Solar inverter
-   Pre-process data and send it to server
-   Host mini-webserver to configure some parameters for datalogger

## Protocol supported

1. Datalogger <-> Solar Inverter
    - Modbus 485
2. Datalogger <-> Server
    - MQTT

## Solar Inverter supported

-   Solar Inverter

## How to build

```
    idf.py build
```

## Flashing

```
    idf.py flash -p <PORT>
```

## References

-   ESP-IDF 4.4 documentation: https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/get-started/index.html
