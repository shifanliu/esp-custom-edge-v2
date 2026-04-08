#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_timer.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"

#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"

#include "../Secret/NetworkConfig.h"

#ifndef _BLE_EDGE_H_
#define _BLE_EDGE_H_

#pragma once

#include <stdbool.h>

extern bool edge_prefer_flooding;
extern int edge_df_fail_count;

typedef struct {
    uint16_t node_addr;
    uint16_t path_origin;
    uint16_t path_target;
    uint16_t origin_dependents[10]; //TODO: The size of this should be tied to directed relay paths in config
    uint16_t num_dependents_origin;
    uint16_t target_dependents[10]; //TODO: The size of this should be tied to directed relay paths in config
    uint16_t num_dependents_target;
} __attribute__((packed)) df_path_t;

// typedef struct {
//     int32_t latitude;       
//     int32_t longitude;      
//     int32_t utc_time;       
//     uint8_t gps_flag;       
//     uint16_t num_satellites;
//     uint8_t button_state;   
// } __attribute__((packed)) gps_data_t;
typedef struct gps_data {
    char     gps_time[32]; // Unix timestamp or seconds
    int32_t  fixType;
    int32_t  gnssFixOK;    // 0=unhealthy, 1=healthy
    int32_t  diffSoln;
    int32_t  numSV;        // number of satellites
    int32_t  lat;          // 32-bit signed, scaled by 1e7 if needed
    int32_t  lon;          // 32-bit signed, scaled by 1e7 if needed
    uint8_t  button_state; // 0=not pressed, 1=pressed
} __attribute__((packed)) gps_data_t;

#define MAX_DF_ENTRIES 10
extern df_path_t df_paths[MAX_DF_ENTRIES];
extern int df_path_count;
extern uint64_t last_send_timestamp;

/**
 * @brief Loop message connection for handling incoming and outgoing messages.
 */
void loop_message_connection();

/**
 * @brief Stop the ESP timer.
 */
void stop_esp_timer();

/**
 * @brief Stop the periodic timer.
 */
void stop_periodic_timer();

/**
 * @brief Get the current state of the node.
 * 
 * @return Current state of the node.
 */
enum State getNodeState();

/**
 * @brief Set the state of the node.
 * 
 * @param state New state of the node.
 */
void setNodeState(enum State state);

/**
 * @brief set the message ttl for current module.
 */
void set_message_ttl(uint8_t new_ttl);

/**
 * @brief Send Message (bytes) to another node in network
 *
 * @param dst_address  Dstination node's unicast address
 * @param length Length of message (bytes)
 * @param data_ptr pointer to data buffer that holds message
 * @param require_response flag that indicate if this message expecting response, timeout will get triger if response not recived
 */
void send_message(uint16_t dst_address, uint16_t length, uint8_t *data_ptr, bool require_response);

/**
 * @brief Send GPS data to another node in the network
 *
 * This function packages a gps_data_t structure and sends it
 * to the specified destination node over the BLE Mesh network.
 *
 * @param dst_address  Destination node's unicast address
 * @param gps Pointer to gps_data_t structure containing latitude,
 *            longitude, UTC time, GPS flag, number of satellites,
 *            and button state
 */
void send_gps_data(uint16_t dst_address, gps_data_t *gps);

/**
 * @brief Broadcast Message (bytes) to all node in network
 *
 * @param length Length of message (bytes)
 * @param data_ptr pointer to data buffer that holds message
 */
void broadcast_message(uint16_t length, uint8_t *data_ptr);

/**
 * @brief Send Response (bytes) to an recived message
 *
 * @param ctx message context of incoming message, will be used for sending response to that message to src node
 * @param length Length of message (bytes)
 * @param data_ptr pointer to data buffer that holds message
 * @param message_opcode opcode of incoming message, used to determine corresponding response opcode
 */
void send_response(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *data_ptr, uint32_t message_opcode);

/**
 * @brief Send an Important Message (bytes) to an node
 * 
 *  This function send and tracks an important message, it will retransmit the important message with higher ttl after timeout for 2 times.
 *
 * @param dst_address  Dstination node's unicast address
 * @param length Length of message (bytes)
 * @param data_ptr pointer to data buffer that holds message
 */
void send_important_message(uint16_t dst_address, uint16_t length, uint8_t *data_ptr);

/**
 * @brief Get the on-tracking Important Message's index
 *
 * @param opcode Message or Response opcode of an important message
 */
int8_t get_important_message_index(uint32_t opcode);

/**
 * @brief Retransmit an Important Message (bytes) to an node
 * 
 *  This function retransmit the important message with higher ttl after timeout for 2 times.
 *
 * @param ctx_ptr poninter to message context of timeout important message
 * @param opcode Message opcode of an important message
 * @param index Index  of on-tracking Important Message
 */
void retransmit_important_message(esp_ble_mesh_msg_ctx_t* ctx_ptr, uint32_t opcode, int8_t index);

/**
 * @brief Clear the tracking on an Important Message
 * 
 *  This function clears the tracking on 1) successful transmission wwith response 2) faild all transmission
 * 
 * @param index Index  of on-tracking Important Message
 */
void clear_important_message(int8_t index);

/**
 * @brief Reset the module and Erase persistent memeory if persistent memeory is enabled.
 * 
 *  This function does not restart the module!
 */
void reset_esp32();

/**
 * @brief Restart the module and Erase persistent memeory if persistent memeory is enabled.
 */
void restart_edge();

/**
 * @brief Initialize Root module and attach event handler callback functions
 * 
 * @param prov_complete_handler Callback function triggered on edge node provisioning completion
 * @param config_complete_handler Callback function triggered on edge node configuration completion
 * @param recv_message_handler Callback function triggered on reciving incoming message
 * @param recv_response_handler Callback function triggered on receiving incoming response to previously sent message
 * @param timeout_handler Callback function triggered on timeout on previously sent message without expected response
 * @param broadcast_handler_cb Callback function triggered on reciving incoming broadcase message
 * @param connectivity_handler_cb Callback function triggered on reciving incoming connectivity message (heartbeat connection check)
 */
esp_err_t esp_module_edge_init(
    void (*prov_complete_handler)(uint16_t node_index, const esp_ble_mesh_octet16_t uuid, uint16_t addr, uint8_t element_num, uint16_t net_idx),
    void (*config_complete_handler)(uint16_t addr),
    void (*recv_message_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr, uint32_t opcode),
    void (*recv_response_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr, uint32_t opcode),
    void (*timeout_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint32_t opcode),
    void (*broadcast_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr),
    void (*connectivity_handler)(esp_ble_mesh_msg_ctx_t *ctx, uint16_t length, uint8_t *msg_ptr)
);

#endif /* _BLE_EDGE_H_ */