/* board.h - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/uart.h"
#include "driver/gpio.h"
//#include "led_strip_encoder.h" TODO: Find migrated extension or find a new library
#include <arpa/inet.h>
#include "../Secret/NetworkConfig.h"

// Dev mode send uart signal to usb-uart port
// #define UART_NUM_H2 UART_NUM_0 // defult log port
// #define TX_PIN_H2 24 // dpin connected with usb-uart
// #define RX_PIN_H2 23 // dpin connected with usb-uart

#define UART_NUM_H2 UART_NUM_1 // UART_NUM_0 is used for usb monitor already
#define TX_PIN_H2 GPIO_NUM_4
#define RX_PIN_H2 GPIO_NUM_5

#define UART_NUM    UART_NUM_H2
#define TXD_PIN     TX_PIN_H2
#define RXD_PIN     RX_PIN_H2
#define RTS_PIN     UART_PIN_NO_CHANGE // not using
#define CTS_PIN     UART_PIN_NO_CHANGE // not using
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

#define BUTTON_IO_NUM           9
#define BUTTON_ACTIVE_LEVEL     0
#define ESCAPE_BYTE 0xFA
#define UART_START 0xFF
#define UART_END 0xFE

enum State {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    WORKING,
};

/**
 * @brief Starts the timer.
 * 
 * Initializes and starts the timer to begin tracking elapsed time.
 */
void startTimer();

/**
 * @brief Gets the time elapsed since the timer was started.
 * 
 * @return double The time elapsed in seconds.
 * 
 * Retrieves the amount of time that has passed since the timer was started.
 */
double getTimeElapsed();

/**
 * @brief Checks if a timeout has occurred.
 * 
 * @return bool True if a timeout has occurred, false otherwise.
 * 
 * Determines whether the timer has reached the timeout threshold.
 */
bool getTimeout();

/**
 * @brief Sets the timeout status.
 * 
 * @param boolean The new timeout status to set.
 * 
 * Updates the timeout status based on the given boolean value.
 */
void setTimeout(bool boolean);

/**
 * @brief Sets the state of the LED.
 * 
 * @param state The new state to set for the LED.
 * 
 * Changes the LED state to the specified value.
 */
void setLEDState(enum State state);

/**
 * @brief Handles the timeout logic for the connection.
 *
 * This function checks the current timeout status and takes appropriate actions.
 * If the current timeout is false, it starts a timer and sets the timeout status to true.
 * If the timeout has already occurred and persists for more than 20 seconds, 
 * it resets the edge module.
 */
void handleConnectionTimeout();

/**
 * @brief Initialize the board.
 */
void board_init(void);

/**
 * @brief Operate the board's LED with specified RGB values.
 * 
 * @param r Red value (0-255).
 * @param g Green value (0-255).
 * @param b Blue value (0-255).
 */
void board_led_operation(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Encoded bytes and write to a UART port.
 * 
 * @param uart_num UART port number.
 * @param data Pointer to the data to be written.
 * @param length Length of the data.
 * @return Status of the write operation.
 */
int uart_write_encoded_bytes(uart_port_t uart_num, uint8_t* data, size_t length);

/**
 * @brief Decode bytes from the provided data.
 * 
 * @param data Pointer to the encoded data.
 * @param length Length of the encoded data.
 * @param decoded_data Pointer to the buffer for the decoded data.
 * @return Status of the decode operation.
 */
int uart_decoded_bytes(uint8_t* data, size_t length, uint8_t* decoded_data);

/**
 * @brief Send data to a specific node address over UART.
 * 
 * @param node_addr Node address to send the data to.
 * @param data Pointer to the data to be sent.
 * @param length Length of the data.
 * @return Status of the send operation.
 */
int uart_sendData(uint16_t node_addr, uint8_t* data, size_t length);

/**
 * @brief Send a message to a specific node address over UART.
 * 
 * @param node_addr Node address to send the message to.
 * @param msg Pointer to the message to be sent.
 * @return Status of the send operation.
 */
int uart_sendMsg(uint16_t node_addr, char* msg);

/**
 * @brief Send a message to the root node after provisioning
 */
void initialDummySend();

void edge_uart_send_json_line(const char *json_line);

#if defined(CONFIG_BLE_MESH_ESP32H2_DEV)
#define LED_R GPIO_NUM_8
#define LED_G GPIO_NUM_8
#define LED_B GPIO_NUM_8
#endif

#endif
