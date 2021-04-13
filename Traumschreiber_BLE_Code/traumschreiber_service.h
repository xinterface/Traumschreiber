/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINTRAUMNGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
 
#ifndef TRAUM_SERVICE_H__
#define TRAUM_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_TRAUM_BASE_UUID              {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_TRAUM_SERVICE                0x0EE6 // Just a random, but recognizable value

#define BLE_UUID_TRAUM_BASE_CHARACTERISTC_0_UUID     0xEE60 // Just a random, but recognizable value
#define BLE_UUID_TRAUM_BASE_CHARACTERISTC_1_UUID     0xEE61 // Just a random, but recognizable value
#define BLE_UUID_TRAUM_BASE_CHARACTERISTC_2_UUID     0xEE62 // Just a random, but recognizable value
#define BLE_UUID_TRAUM_CONF_CHARACTERISTC_UUID       0xECC0 // Just a random, but recognizable value
#define BLE_UUID_TRAUM_CODE_CHARACTERISTC_UUID       0xC0DE // Just a random, but recognizable value


#define BLE_TRAUM_BASE_BITS_PER_CHANNEL 14
#define BLE_TRAUM_SAMPLES_PER_PACKAGE 4
#define BLE_TRAUM_PACKAGE_HEADER_LENGTH 1
#define TRAUM_SERVICE_VALUE_LENGTH (BLE_TRAUM_BASE_BITS_PER_CHANNEL * 3 * BLE_TRAUM_SAMPLES_PER_PACKAGE + BLE_TRAUM_PACKAGE_HEADER_LENGTH)
#define BLE_TRAUM_ATT_MTU TRAUM_SERVICE_VALUE_LENGTH+3
#define BLE_TRAUM_GAP_DL TRAUM_SERVICE_VALUE_LENGTH+7

#define CONF_CHAR_VALUE_LENGTH  8
#define CODE_CHAR_VALUE_LENGTH  14


static const uint8_t traum_use_code_characteristic = 1;
extern int16_t traum_code_characteristic_transmission_pending;
extern uint8_t* traum_code_characteristic_transmission_pointer;
static uint8_t traum_use_only_one_characteristic = 1;
static uint8_t traum_bits_per_channel = BLE_TRAUM_BASE_BITS_PER_CHANNEL;

/**
 * @brief This structure contains various status information for our service. 
 * It only holds one entry now, but will be populated with more items as we go.
 * The name is based on the naming convention used in Nordic's SDKs. 
 * 'ble indicates that it is a Bluetooth Low Energy relevant structure and 
 * os is short for Our Service). 
 */
typedef struct
{
    uint16_t    conn_handle; 
	uint16_t    service_handle;     /**< Handle of Our Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t    char_base_handle_0;
	ble_gatts_char_handles_t    char_base_handle_1;
	ble_gatts_char_handles_t    char_base_handle_2;
	ble_gatts_char_handles_t    char_code_handle;
	ble_gatts_char_handles_t    char_conf_handle;
    uint8_t   use_large_packages;
}ble_traum_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_traum_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void traum_service_init(ble_traum_t * p_traum_service);


void traum_eeg_data_characteristic_update(ble_traum_t *p_traum_service);
void traum_battery_status_update(ble_traum_t *p_traum_service, uint8_t * data);
uint32_t traum_encoding_char_update(ble_traum_t *p_traum_service, uint8_t * data);



#endif  /* _ TRAUM_SERVICE_H__ */
