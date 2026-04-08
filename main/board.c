/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "iot_button.h"
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include "board.h"
#include "ble_mesh_config_edge.h"
#include "cJSON.h"

#if LOCAL_EDGE_DEVICE
    #include "local_edge_device.c"
#endif

#define TAG_B "BOARD"
#define TAG_W "Debug"

extern void send_message(uint16_t dst_address, uint16_t length, uint8_t *data_ptr, bool require_response);
extern void printNetworkInfo();
extern void create_data_send_event();
extern void stop_data_send_event();
extern void sendRobotRequest();
extern void reset_edge();
extern void send_important_message(uint16_t dst_address, uint16_t length, uint8_t *data_ptr);

static void uart_rx_task(void *arg);

clock_t start_time;
bool timeout = false;

void startTimer() {
    start_time = clock();
}

void setTimeout(bool boolean) {
    timeout = boolean;
}

double getTimeElapsed() {
    clock_t end_time = clock();
    return ((double) (end_time - start_time)) / CLOCKS_PER_SEC;
}

bool getTimeout() {
    return timeout;
}

void setLEDState(enum State nodeState) {
    if(nodeState == DISCONNECTED) {
        board_led_operation(50, 0, 0); // Red LED Color
    }
    else if (nodeState == CONNECTING) {
        board_led_operation(0, 0, 50); // Blue LED Color
    }
    else if (nodeState ==  CONNECTED) {
        board_led_operation(0, 50, 0); // Green LED Color
    }
    else if (nodeState == WORKING) {
        board_led_operation(50, 50, 0); // Yellow LED Color
    }
    else {
        board_led_operation(0, 0, 0); // No Color == No State
    }
}

void handleConnectionTimeout() {
    bool currentTimeout = getTimeout();
    // ESP_LOGI(TAG_M, "Current timeout value: %s", currentTimeout ? "true" : "false");

    if(!currentTimeout) {
        // ESP_LOGI(TAG_M, "Keep the first timeout time...");
        startTimer();
        setTimeout(true);
    }
    else if(getTimeElapsed() > 20.0) {
        // ESP_LOGI(TAG_M, "Edge not able to connect to root, Resetting the Edge Module");
        reset_edge();
    }
}

void board_led_operation(uint8_t r, uint8_t g, uint8_t b)
{
    //rmt_led_set(r,g,b); TODO: Find migrated extension or find a new library
}

static void board_led_init(void)
{
    //rmt_encoder_init(); TODO: Find migrated extension or find a new library
}

// ====================== repetive code, better clean up ======================
void board_dispatch_network_command(char *ble_cmd, uint16_t node_addr, uint8_t *data_buffer, size_t data_length)
{
    uint8_t command_msg[MAX_MSG_LEN + BLE_CMD_LEN + BLE_ADDR_LEN];
    memset(command_msg, 0, MAX_MSG_LEN + BLE_CMD_LEN + BLE_ADDR_LEN);
    uint8_t *msg_itr = command_msg;
    uint16_t node_addr_network_order = htons(node_addr);

    if (data_length > MAX_MSG_LEN)
    {
        ESP_LOGE(TAG_L, "Local Edge Device Trying to Send %d bytes message that's more than MAX_MSG_LEN-%d", (int)data_length, (int)MAX_MSG_LEN);
        return;
    }

    memcpy(msg_itr, ble_cmd, BLE_CMD_LEN);
    msg_itr += BLE_CMD_LEN;
    memcpy(msg_itr, &node_addr_network_order, BLE_ADDR_LEN);
    msg_itr += BLE_ADDR_LEN;

    if (data_buffer != NULL)
    {
        memcpy(msg_itr, data_buffer, data_length);
        msg_itr += data_length;
    }

    ESP_LOGI(TAG_L, "data_buffer: '%.*s'", data_length, data_buffer);
    execute_network_command((char *)command_msg, msg_itr - command_msg);
}

void board_ble_send_to_root(uint8_t *data_buffer, size_t data_length)
{
    char ble_cmd[7] = "SEND-";
    ESP_LOGI(TAG_L, "data_buffer: '%.*s'", data_length, data_buffer);
    board_dispatch_network_command(ble_cmd, 0, data_buffer, data_length);
}

// ====================== repetive code, better clean up ======================

void edge_uart_send_json_line(const char *json_line)
{
    uart_write_bytes(UART_NUM, json_line, strlen(json_line));
    uart_write_bytes(UART_NUM, "\n", 1);
}

static void button_tap_cb(void* arg)
{
    ESP_LOGW(TAG_W, "button tapped ------------------------- ");
    
    double lat_d = (38.5434667768 - 38.5395022575) * ((double) esp_random() / UINT32_MAX)
                   + 38.5395022575;
    double lon_d = (-121.7786203497 + 121.7716779140) * ((double) esp_random() / UINT32_MAX)
                   - 121.7786203497;
    int32_t lat_i = (int32_t)(lat_d * 1e7);
    int32_t lon_i = (int32_t)(lon_d * 1e7);

    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char iso_time[32];
    strftime(iso_time, sizeof(iso_time), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    gps_data_t gps = {0};
    strncpy(gps.gps_time, iso_time, sizeof(gps.gps_time) - 1);

    gps.fixType     = 3;
    gps.gnssFixOK   = 1;
    gps.diffSoln    = 0;
    gps.numSV       = 10;

    gps.lat         = lat_i;
    gps.lon         = lon_i;

    gps.button_state = 1; 

    ESP_LOGI(TAG_W, "Button GPS -> time:%s lat:%d lon:%d", gps.gps_time, gps.lat, gps.lon);
    send_gps_data(PROV_OWN_ADDR, &gps);
    char logbuf[256];
    snprintf(logbuf, sizeof(logbuf),
         "{\"src\":\"edge\",\"type\":\"gps_sent\","
         "\"time\":\"%s\",\"lat\":%" PRId32 ",\"lon\":%" PRId32 "}",
         gps.gps_time, gps.lat, gps.lon);
    edge_uart_send_json_line(logbuf);

    // if (control < 2) {
    //     ESP_LOGE(TAG_W, "=== Normal Message === [%d]", control);
    //     send_message(PROV_OWN_ADDR, message_2_length, (uint8_t*) message_2, false);
    //     control += 1;
    // } else if (control == 2) {
    //     ESP_LOGE(TAG_W, "=== Important Message === [%d]", control);
    //     send_important_message(PROV_OWN_ADDR, message_length, (uint8_t*) message);
    //     control += 1;
    // } else if (control < 5) {
    //     ESP_LOGE(TAG_W, "=== Normal Message === [%d]", control);
    //     send_message(PROV_OWN_ADDR, message_2_length, (uint8_t*) message_2, false);
    //     control += 1;
    // } else {
    //     control = 0;
    //     ESP_LOGE(TAG_W, "=== Reset Control === [%d]", control);
    // }
}

static void button_liong_press_cb(void *arg)
{
    ESP_LOGW(TAG_W, "button long pressed ------------------------- ");
    ESP_LOGW(TAG_W, "toggling data sending ------");
    static int control = 0;

    if (control == 0) {
        ESP_LOGW(TAG_W, "clear data sending ------");
        stop_data_send_event();
        control = 1;
    } else if (control == 1) {
        ESP_LOGW(TAG_W, "create data sending ------");
        create_data_send_event();
        control = 2;
    } else {
        ESP_LOGW(TAG_W, "clear data sending ------");
        stop_data_send_event();
        control = 0;
    }
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
        iot_button_set_serial_cb(btn_handle, 3, 5000, button_liong_press_cb, "SERIAL");
    }
}

static void uart_init() {  // Uart ===========================================================
    const int uart_num = UART_NUM;
    const int uart_buffer_size = UART_BUF_SIZE * 2;
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl= UART_HW_FLOWCTRL_DISABLE, // = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = UART_SCLK_DEFAULT, // = 122,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                        uart_buffer_size, 0, NULL, 0)); // not using queue
                                        // uart_buffer_size, 20, &uart_queue, 0));
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins                      (TX,      RX,      RTS,     CTS)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN));

    ESP_LOGI(TAG_B, "Uart init done");
}

// escape char
int uart_write_encoded_bytes(uart_port_t uart_num, uint8_t* data, size_t length) {
    uint8_t esacpe_byte = ESCAPE_BYTE;

    int byte_wrote = 0;
    for (uint8_t* byte_itr = data; byte_itr < data + length; ++byte_itr) {
        if (byte_itr[0] < esacpe_byte) {
            uart_write_bytes(UART_NUM, byte_itr, 1);
            byte_wrote += 1;
            continue;
        }

        // nned 2 byte encoded
        uint8_t encoded = byte_itr[0] ^ esacpe_byte; // bitwise Xor
        uart_write_bytes(UART_NUM, &esacpe_byte, 1);
        uart_write_bytes(UART_NUM, &encoded, 1);
        byte_wrote += 2;
    }

    return byte_wrote;
}

// Able to wrote back to the same buffer, since decoded data is always shorter
int uart_decoded_bytes(uint8_t* data, size_t length, uint8_t* decoded_data) {
    int decoed_len = 0;
    uint8_t* decode_itr = decoded_data;

    for (uint8_t* byte_itr = data; byte_itr < data + length; ++byte_itr) {
        if (byte_itr[0] != ESCAPE_BYTE) {
            // not a ESCAPE_BYTE
            decode_itr[0] = byte_itr[0];
            decode_itr += 1;
            decoed_len += 1;
            continue;
        }

        // ESCAPE_BYTE, decode 2 byte into 1
        byte_itr += 1; // move to next to get encoded byte
        uint8_t encoded = byte_itr[0];
        
        uint8_t decoded = encoded ^ ESCAPE_BYTE; // bitwise Xor
        decode_itr[0] = decoded;
        decode_itr += 1;
        decoed_len += 1;
    }
    
    return decoed_len;
}


// TB Finish, need to encode the send data for escape bytes
// do we need to regulate the message length?
int uart_sendData(uint16_t node_addr, uint8_t* data, size_t length)
{
#if LOCAL_EDGE_DEVICE
    // enabled local_edge_device, pass message to local_edge_device
    local_edge_device_network_message_handler(node_addr, data, length);
    return length;
#else
    // not enabled local_edge_device, pass message to uart with uart encoding
    uint8_t uart_start = UART_START;
    uint8_t uart_end = UART_END;
    int txBytes = 0;

    uint16_t node_addr_big_endian = htons(node_addr); 
    txBytes += uart_write_bytes(UART_NUM, &uart_start, 1); // 0xFF
    txBytes += uart_write_encoded_bytes(UART_NUM, (uint8_t*) &node_addr_big_endian, 2);
    txBytes += uart_write_encoded_bytes(UART_NUM, data, length);
    txBytes += uart_write_bytes(UART_NUM, &uart_end, 1);  // 0xFE

    ESP_LOGI("[UART]", "Wrote %d bytes Data on uart-tx", txBytes);
    return txBytes;
#endif
}

// TB Finish, need to encode the send data for escape bytes
int uart_sendMsg(uint16_t node_addr, char* msg)
{
    size_t length = strlen(msg);
    uint8_t uart_start = UART_START;
    uint8_t uart_end = UART_END;
    int txBytes = 0;

    uint16_t node_addr_big_endian = htons(node_addr); 
    txBytes += uart_write_bytes(UART_NUM, &uart_start, 1); // 0xFF
    txBytes += uart_write_encoded_bytes(UART_NUM, (uint8_t*) &node_addr_big_endian, 2);
    txBytes += uart_write_encoded_bytes(UART_NUM, (uint8_t*) msg, length);
    txBytes += uart_write_bytes(UART_NUM, &uart_end, 1);  // 0xFE

    ESP_LOGI("[UART]", "Wrote %d bytes Msg on uart-tx", txBytes);
    return txBytes;
}

static void uart_rx_task(void *arg)
{
    uint8_t buf[256];
    uint8_t decoded[256];

    while (1) {

        int len = uart_read_bytes(UART_NUM, buf, sizeof(buf), 10 / portTICK_PERIOD_MS);

        if (len > 0) {
            int decoded_len = uart_decoded_bytes(buf, len, decoded);
            decoded[decoded_len] = '\0';

            cJSON *root = cJSON_Parse((char*)decoded);
            if (root) {
                gps_data_t gps = {0};

                cJSON *jt = cJSON_GetObjectItem(root, "gps_time");
                if (cJSON_IsString(jt)) {
                    strncpy(gps.gps_time, jt->valuestring, sizeof(gps.gps_time) - 1);
                }

                gps.fixType    = cJSON_GetObjectItem(root, "fixType")->valueint;
                gps.gnssFixOK  = cJSON_GetObjectItem(root, "gnssFixOK")->valueint;
                gps.diffSoln   = cJSON_GetObjectItem(root, "diffSoln")->valueint;
                gps.numSV      = cJSON_GetObjectItem(root, "numSV")->valueint;
                gps.lat        = cJSON_GetObjectItem(root, "lat")->valueint;
                gps.lon        = cJSON_GetObjectItem(root, "lon")->valueint;

                gps.button_state = 0;

                send_gps_data(PROV_OWN_ADDR, &gps);
                ESP_LOGI(TAG_W, "Sent GPS (UART_RX) -> time:%s lat:%d lon:%d", gps.gps_time, gps.lat, gps.lon);
                char logbuf[256];
                snprintf(logbuf, sizeof(logbuf),
                    "{\"src\":\"edge\",\"type\":\"gps_sent\","
                    "\"time\":\"%s\",\"lat\":%" PRId32 ",\"lon\":%" PRId32 "}",
                    gps.gps_time, gps.lat, gps.lon);
                edge_uart_send_json_line(logbuf);
                cJSON_Delete(root);
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void initialDummySend()
{
    char message[5] = "the j";
    ESP_LOGE(TAG_W, "the j [%d]", 0);
    send_message(PROV_OWN_ADDR, strlen(message), (uint8_t*) message, false);
}

void board_init(void)
{
    uart_init();
    board_led_init();
    board_button_init();

#if LOCAL_EDGE_DEVICE
    // enabled local_edge_device, initialize the local device
    local_edge_device_init();
#endif
    
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
}
