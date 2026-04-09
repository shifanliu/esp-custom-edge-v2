| Supported Targets | ESP32-H2 | 
| ----------------- | -------- | 

ESP32 Edge Network Module
==================================
## Table of Contents
- [ESP32 Edge Network Module](#esp32-edge-network-module)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Hardware Components](#hardware-components)
  - [Software Components](#software-components)
  - [Setup and Configuration](#setup-and-configuration)
    - [1. Downloading ESP-IDF Extension on VSCode (*Recommended*)](#1-downloading-esp-idf-extension-on-vscode-recommended)
    - [2. Using Docker Images on ESP-IDF](#2-using-docker-images-on-esp-idf)
  - [Communication Protocols](#communication-protocols)
  - [Code Structure](#code-structure)
  - [Code Flow](#code-flow)
    - [1) Initialization](#1-initialization)
    - [2) UART Channel Logic Flow](#2-uart-channel-logic-flow)
    - [3) Network Commands - UART incoming](#3-network-commands---uart-incoming)
    - [4) Module to App level - UART outgoing](#4-module-to-app-level---uart-outgoing)
    - [5) Event Handler](#5-event-handler)
    - [Error Handling](#error-handling)
  - [Testing and Troubleshooting](#testing-and-troubleshooting)
  - [References](#references)

## Overview
The ESP32 Edge Module serves as an edge node in the BLE mesh network. It is responsible for receiving local data, forwarding application data to the root, relaying mesh traffic, and providing node-level observability for testing and debugging.

In the current implementation, the edge supports the following major features:

- **BLE Mesh Edge Node**
  - Joins the BLE mesh network as a provisioned node
  - Sends unicast and broadcast traffic through the mesh
  - Receives messages and responses from the root or other nodes

- **Local UART Gateway**
  - Receives JSON-formatted GPS data over UART from a Raspberry Pi or other local host
  - Converts that JSON into the internal `gps_data_t` struct
  - Sends the GPS struct to the root over BLE mesh

- **Reliability / Observability**
  - Detects ACK responses from the root
  - Computes round-trip time (RTT) using send and ACK receive timestamps
  - Logs RTT together with the current send mode (`df` or `flooding`)

- **Adaptive Send Mode**
  - Attempts Directed Forwarding first
  - Falls back to flooding after repeated Directed Forwarding failures
  - Resets back to Directed Forwarding after a successful send completes

- **Debug / Telemetry Logging**
  - Emits JSON log lines over UART such as:
    - `gps_sent`
    - `pass_count`
    - `rtt`
      
## Hardware Components
For more information please contact the author if interested on the Custom PCB or Antenna.

## Software Components
- ESP-IDF version 5.2.0 (Espressif IoT Development Framework)
  - Description: Official development framework for ESP32
  - Function: Provides libraries and tools for developing applications on the ESP32
    - Build, Flash, Monitor, etc.
  - Instalation: [link to ESP's website](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)

## Setup and Configuration
In this section, we will be explaining 2 ways on using our program, specifically ESP-IDF.

### 1. Downloading ESP-IDF Extension on VSCode (*Recommended*)
- Make sure you have [VS Code](https://code.visualstudio.com/download), it can be any operating system, or any version of VS Code.
- The next step is to download ESP-IDF Extension on VSCode. There are steps
on using ESP-IDF Extension in this [link](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md)
- **P.S. Make sure the ESP-IDF version is 5.2.0, without this version, our code would not be able to run.**
- Once you have follow the steps on installing ESP-IDF, you are ready to `build`, `flash`, and `monitor`.
- **P.S. If you are on windows, you need to install a driver to establish a serial connection with the ESP32 Board. You can find it on this [website](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html). The author use the `CP210x USB to UART Bridge Drivers` to connect the windows port to the ESP32.**

### 2. Using Docker Images on ESP-IDF
- If you don't want to download ESP-IDF Extension, you can also use a Docker Image to `build` and `flash` the program. However, this only works in `Linux` system since you need a port number that's connected to the ESP32 when `flashing`
- The steps are as follows:
  1. Make sure you have [Docker Desktop](https://www.docker.com/products/docker-desktop/) downloaded and running in the background 
  2. Go to the terminal, and go to the project directory. (If you want to make sure you are in your project directory, you can write `${PWD}`, if this returns the project directory, that means you're in the right palce)
  3. Run `docker run --rm -v ${PWD}:/project -w /project -e HOME=/tmp espressif/idf:v5.2 idf.py build`
  4. Once it's done building, then you can run `docker run --rm -v ${PWD}:/project -w /project -e HOME=/tmp espressif/idf:v5.2 idf.py -p -PORT flash`. to flash to a ESP32 Board

  The `-PORT` you can change it to the port you're ESP32 is connected to, for example `/dev/ttyS5`. For more information on the docker image ESP32, click [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-docker-image.html)

  Also, after you're done flashing, you could also write `idf.py monitor --no-reset -p -PORT`. For more information, you can check [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-monitor.html)
  
## Communication Protocols

### UART Incoming Commands (Host / Raspberry Pi -> Edge)
The edge can receive framed UART traffic from a local host such as a Raspberry Pi. The current edge-side code supports two major categories of UART input:

#### 1. Standard command framing
The edge UART command parser supports command-style framed messages such as:

- `SEND-`
- `BCAST`
- `RST-E`

Typical format:

`5-byte command | 2-byte destination | payload`

#### 2. JSON GPS input
The edge also listens for UART data that decodes as JSON. If valid JSON is received and it contains GPS fields such as:

- `gps_time`
- `fixType`
- `gnssFixOK`
- `diffSoln`
- `numSV`
- `lat`
- `lon`

the edge converts that JSON into an internal `gps_data_t` struct and sends it to the root over BLE mesh.

### UART Outgoing Messages (Edge -> Host / Raspberry Pi)
The edge outputs several types of UART data.

#### 1. Standard mesh payload forwarding
For received mesh traffic, the outgoing format is generally:

`2-byte node_addr | payload`

where `node_addr` is in network byte order.

#### 2. JSON debug / telemetry lines
The edge also sends JSON log lines using `edge_uart_send_json_line(...)`. These logs currently include:

- `gps_sent`
- `pass_count`
- `rtt`

## Code Structure
This repository contains several files and directories, but the most important ones for the edge module are listed below:

- **`/main`**
  - **`ble_mesh_config_edge.c`**  
    Contains edge-side BLE mesh initialization, send behavior, timeout handling, directed forwarding state, and utility APIs such as `send_message(...)`, `send_response(...)`, and `send_gps_data(...)`.
  - **`ble_mesh_config_edge.h`**
  - **`board.c`**  
    Contains UART helpers, button callbacks, UART JSON GPS parsing, board init, and edge-side JSON log output.
  - **`board.h`**
  - **`local_edge_device.c`**  
    Optional local device logic integrated on the DevKit module
  - **`main.c`**  
    Contains edge-side UART command parsing, high-level event handlers, ACK/RTT handling, `pass_count` logging, and app-level control flow.
  - **`idf_component.yml`**
  - **`CMakeLists.txt`**

- **`/Secret`**
  - Contains mesh network configuration headers and shared definitions

- **Top-level files**
  - **`CMakeLists.txt`**
  - **`sdkconfig.defaults`**

## Code Flow
### 1) Initialization
The module is initialized and configured in `app_main()` when power on or resetted. It initialized all the `hardware componets`, `ble-mesh configurations`, and `uart procssing thread`, then attachs all event handlers. After initialization, edge module sends and message to uart channel signaling edge module online.

In the code below, `line 3`, `esp_log_level_set(TAG_ALL, ESP_LOG_NONE);` disables esp logs that's used for develpment debug logging but will pollute uart channel on edge module.

```c
void app_main(void)
{
    esp_log_level_set(TAG_ALL, ESP_LOG_NONE); // disable esp logs
    
    esp_err_t err = esp_module_edge_init(prov_complete_handler, config_complete_handler, recv_message_handler, recv_response_handler, timeout_handler, broadcast_handler, connectivity_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_M, "Network Module Initialization failed (err %d)", err);
        uart_sendMsg(0, "Error: Network Module Initialization failed\n");
        return;
    }
    
    board_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);

    char message[15] = "[E]online\n";
    uart_sendData(0, (uint8_t *)message, strlen(message));
}
```
In `line 5`, `esp_module_edge_init` is called to initialize the ESP Module which includes multiple functions that are used as callback functions for Network events.
```c
esp_err_t esp_module_edge_init(
    void (*prov_complete_handler)(uint16_t node_index, const esp_ble_mesh_octet16_t uuid, 
                                  uint16_t addr, uint8_t element_num, uint16_t net_idx),
    void (*config_complete_handler)(uint16_t addr),
    void (*recv_message_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr),
    void (*recv_response_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr),
    void (*timeout_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint32_t opcode),
    void (*broadcast_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr),
    void (*connectivity_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr)
) { ... }
```
Each handler function will get trigers by corresponding event [link here](#event-handler)

After initialization, the edge starts board/UART tasks and becomes ready to receive local UART data and BLE mesh traffic. In the current implementation, the most important runtime paths are GPS injection from UART, ACK-based RTT measurement, and Directed Forwarding / flooding switching.

### 2) UART Channel Logic Flow
The module communicate with central PC via usb-uart port.

1. `UART byte encoding` - To ensure the message bytes' integrity, message encoding was applied to add `\0xFF` and `\0xFE` speical bytes at the begining and end of an uart message. Also the message byte encoding was applied to encode all bytes >= `\0xFA` into 2 byte with xor gate to reserve all bytes > `\0xFA` as speical bytes. Uart encoding, decoding, and write functions is defined in `board.h` file with detailed explainatiion.

2. `UART channel listening thread` - The function `rx_task()` on main.c defines the uart signal handling logic. It create an infinite scanning loop to check uart buffer's data avalaibility. Once the scanner read in datas, it passes to `uart_task_handler()` to scan for message start byte `\0xFF` and message end byte `\0xFE` to locate the message then decode the messsage and invoke `execute_uart_command()` to parse and execute the message received.

3. `execute_uart_command()` - This function responsible for executing commands from application level such as `BCAST`, `SEND-`, and etc. The module able to be extended for custom command by adding a case in this function.

### 3) Network Commands - UART incoming
The form of network commands sent to the edge is:

`5-byte network command | payload`

Current examples include:

- **`SEND-`**
  - Send a unicast BLE mesh message
- **`BCAST`**
  - Send a BLE mesh broadcast
- **`RST-E`**
  - Restart the edge module

Depending on the command, the payload may include a destination address and raw message bytes.

### 4) Module to App level - UART outgoing
The standard app-level UART output format for forwarded mesh data is:

`2-byte node_addr | payload`

This is the normal case when the edge forwards mesh traffic up to the local host.

In addition, the edge also emits JSON telemetry lines for testing and debugging, including:
- `gps_sent`
- `pass_count`
- `rtt`

### 5) GPS Forwarding from Raspberry Pi
One of the key new features is real GPS forwarding through the edge.

The flow is:

1. A Raspberry Pi or local host sends GPS data over UART as JSON
2. The edge UART RX task parses the JSON
3. The JSON is copied into the internal `gps_data_t` struct
4. The edge sends that GPS struct to the root using `send_gps_data(...)`
5. The edge emits a `gps_sent` JSON log line over UART for observability

The edge can also generate GPS payloads locally from the onboard button path for testing.

### 6) RTT Measurement and ACK Handling
Another major addition is RTT measurement using ACKs from the root.

The flow is:

1. Before sending a message, the edge stores a send timestamp
2. The root receives the message and sends an ACK back
3. The edge detects `ECS_193_MODEL_OP_RESPONSE`
4. The edge computes RTT using:

`RTT = ACK receive time - last send time`

5. The edge logs the result as JSON:

`{"src":"edge","type":"rtt","mode":"df|flooding","rtt_ms":...}`

This makes it possible to compare performance under Directed Forwarding vs flooding.

### 7) Directed Forwarding / Flooding Fallback
The edge now supports adaptive send behavior.

By default:
- the edge prefers **Directed Forwarding**
- it marks outgoing messages with the directed send tag
- it records the current send mode as `"df"`

If repeated client send timeouts occur:
- the edge increments a DF failure counter
- once the configured threshold is reached, the edge switches to **flooding**
- the current send mode becomes `"flooding"`

After a successful send completion:
- the DF failure count is cleared
- the edge preference is reset back to Directed Forwarding

### 8) Event Handler
The network module uses callback-based event handlers so that BLE mesh stack logic remains in `ble_mesh_config_edge.c`, while higher-level application behavior stays in `main.c`.

The main handlers are:

- `prov_complete_handler`
  - Invoked when the node provisioning process completes

- `config_complete_handler`
  - Invoked when the node is fully configured
  - Stores the edge’s own address
  - Marks the node as connected

- `recv_message_handler`
  - Invoked when the edge receives a message
  - Logs whether the message arrived via Directed Forwarding or Flooding
  - Increments and emits `pass_count`
  - Forwards payload to UART
  - Sends a response if the opcode requires it

- `recv_response_handler`
  - Invoked when the edge receives a response
  - Detects ACKs from the root and computes RTT
  - Clears confirmed important-message state as needed

- `timeout_handler`
  - Invoked when a response-expected message times out
  - Retransmits important messages when needed

- `broadcast_handler`
  - Invoked when a broadcast is received
  - Increments and emits `pass_count`
  - Forwards payload to UART

- `connectivity_handler`
  - Invoked when connectivity / heartbeat messages are received
  - Sends a response back as needed

### 9) Error Handling / Known Issues
Known issues / considerations include:

- **UART framing / partial message issue**
  - If only part of a framed UART message is read into the buffer, the parser may encounter incomplete message data

- **Directed Forwarding instability**
  - DF paths may become temporarily unreliable, which is why the edge now falls back to flooding after repeated timeouts

- **Root reset / persistent state mismatch**
  - If the root resets or clears persistent mesh state while edges still retain old state, reprovisioning mismatches may occur

- **Power / battery issues**
  - Low supply or unstable power may affect radio behavior, provisioning, or mesh message delivery

- **UART port ownership**
  - Multiple host-side readers or an incorrect serial port can interfere with normal operation


OPTIONAL:
potencial error and warning and current fix.

## Testing and Troubleshooting

### Suggested tests
- Send UART GPS JSON into the edge and verify:
  - the edge parses it
  - the edge sends GPS to the root
  - the edge emits a `gps_sent` JSON log line

- Send a message from the edge to the root and verify:
  - the root returns an ACK
  - the edge computes and logs RTT

- Force repeated timeouts and verify:
  - DF timeout count increases
  - the edge switches to flooding
  - subsequent RTT logs show `mode:"flooding"`

- Receive mesh traffic on the edge and verify:
  - `pass_count` increments
  - the JSON `pass_count` log is emitted
  - payload is forwarded over UART

## References
[ESP_BLE_MESH](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/esp-ble-mesh/ble-mesh-index.html)

[ESP_BLE_MESH Github Project Examples](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/esp_ble_mesh/README.md)