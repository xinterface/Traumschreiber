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

#include <stdint.h>
#include <string.h>
#include "traumschreiber_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "uicr_config.h"
//#include "SEGGER_RTT.h"
#include "ad_spi.h"

//semaphore for char_update-function, so full packet sends won't be interrupted
bool ble_traum_char_update_semaphore = true;


// ALREADY_DONE_FOR_YOU: Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_traum_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  	ble_traum_t * p_traum_service =(ble_traum_t *) p_context;  
		///	NRF_LOG_INFO(" BLE EVENT HANDLER: %i", p_ble_evt->header.evt_id);

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			p_traum_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                        spi_ble_connect(p_traum_service);
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			p_traum_service->conn_handle = BLE_CONN_HANDLE_INVALID;
                        spi_ble_disconnect();
			break;
                case BLE_GATTS_EVT_WRITE:
  //                      NRF_LOG_INFO(" BLE WRITE EVENT %i,%02x", p_ble_evt->evt.gatts_evt.params.write.len, p_ble_evt->evt.gatts_evt.params.write.data[0]);
                        if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_traum_service->char_conf_handle.value_handle) {
                            spi_config_update(p_ble_evt->evt.gatts_evt.params.write.data);//(uint8_t)*
                        } else {
                            spi_ble_notify((uint16_t)*p_ble_evt->evt.gatts_evt.params.write.data);
                        }
                        break;
                case BLE_GATTS_EVT_HVN_TX_COMPLETE:                   //when data was send
                    //NRF_LOG_INFO(" HVN_TX_COMPLETE c:%i", p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
                    //NRF_LOG_INFO(" HVN: %i == %i", p_ble_evt->evt.gatts_evt.params.hvc.handle, p_traum_service->char_base_handle.value_handle);
                    
                    //this check does not work, because the p_ble_evt does not contain the handle information
                    //it is kept (commented out) because it would be a useful check and might be available in future sdk's
                    //until then spi_ble_sent() keeps track of connection/notification status as replacement
                    //if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_traum_service->char_base_handle.value_handle) {
                        spi_ble_sent(p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
                    //    traum_eeg_data_characteristic_update(p_traum_service);//try to send new data
                    //}
                    break;
                //case BLE_EVT_TX_COMPLETE //would be needed if indication was used (instead of BLE_GATTS_EVT_HVN_TX_COMPLETE)
	default:
		// No implementation needed.
		break;
	}
	
}


void traum_gpio_set(uint32_t value, uint32_t mask)
{
    uint32_t pin_number = 1;
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
  
    nrf_gpio_port_out_set(reg, value & mask);
}


/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t traum_char_add(ble_traum_t * p_traum_service, ble_gatts_char_handles_t * char_handle, uint8_t * init_value, uint8_t value_length, uint16_t ble_uuid, uint8_t p_notify, uint8_t p_indicate)
{
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
	uint32_t            err_code;
	ble_uuid_t          char_uuid;
	//ble_uuid128_t       base_uuid = BLE_UUID_TRAUM_BASE_UUID;
	//char_uuid.uuid      = BLE_UUID_TRAUM_CHARACTERISTC_UUID;
	//err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	//APP_ERROR_CHECK(err_code); 
	
	BLE_UUID_BLE_ASSIGN(char_uuid, ble_uuid);

    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 0;



    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
	char_md.p_cccd_md           = &cccd_md;
	char_md.char_props.notify   = p_notify;
	char_md.char_props.indicate = p_indicate;

   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));  
	attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
	
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;


    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
	attr_char_value.max_len     = value_length;
	attr_char_value.init_len    = sizeof(init_value);
	//uint8_t value[4]            = {0,0,0,0};
	attr_char_value.p_value     = init_value;
	

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
	err_code = sd_ble_gatts_characteristic_add(p_traum_service->service_handle,
									   &char_md,
									   &attr_char_value,
									   char_handle);
	APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for adding our new config characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t traum_conf_char_add(ble_traum_t * p_traum_service, ble_gatts_char_handles_t * char_handle, uint8_t * init_value, uint8_t value_length, uint16_t ble_uuid)
{
   
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
	uint32_t            err_code;
	ble_uuid_t          char_uuid;
	//ble_uuid128_t       base_uuid = BLE_UUID_TRAUM_BASE_UUID;
	//char_uuid.uuid      = BLE_UUID_TRAUM_CHARACTERISTC_UUID;
	//err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	//APP_ERROR_CHECK(err_code); 
	
	BLE_UUID_BLE_ASSIGN(char_uuid, ble_uuid);

    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;

       
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
	char_md.p_cccd_md           = &cccd_md;
	char_md.char_props.notify   = 0;
	char_md.char_props.indicate   = 0; //has no effect...

   // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));  
	attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
	
   // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;


    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
	attr_char_value.max_len     = value_length;
	attr_char_value.init_len    = value_length;
	//uint8_t value[2]            = {0};
	attr_char_value.p_value     = init_value;
	
    // OUR_JOB: Step 2.E, Add our new characteristic to the service
	err_code = sd_ble_gatts_characteristic_add(p_traum_service->service_handle,
									   &char_md,
									   &attr_char_value,
									   char_handle);
    APP_ERROR_CHECK(err_code);

        
    
    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void traum_service_init(ble_traum_t * p_traum_service)
{
    
    // STEP 3: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table     
        uint32_t		err_code;
	ble_uuid_t		service_uuid;
	ble_uuid128_t		base_uuid = BLE_UUID_TRAUM_BASE_UUID;
	
	
	// STEP 4: Add our service
        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_TRAUM_SERVICE);

	
	// OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
	p_traum_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                    &service_uuid,
                                    &p_traum_service->service_handle);
	APP_ERROR_CHECK(err_code);
	
        

	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service.
        uint8_t value0[TRAUM_SERVICE_VALUE_LENGTH]  = {0x00};
        traum_char_add(p_traum_service, &p_traum_service->char_base_handle_0, value0, TRAUM_SERVICE_VALUE_LENGTH, BLE_UUID_TRAUM_BASE_CHARACTERISTC_0_UUID, 1, 0);
        uint8_t value1[TRAUM_SERVICE_VALUE_LENGTH]  = {0x00};
        traum_char_add(p_traum_service, &p_traum_service->char_base_handle_1, value1, TRAUM_SERVICE_VALUE_LENGTH, BLE_UUID_TRAUM_BASE_CHARACTERISTC_1_UUID, 1, 0);
        uint8_t value2[TRAUM_SERVICE_VALUE_LENGTH]  = {0x00};
        traum_char_add(p_traum_service, &p_traum_service->char_base_handle_2, value2, TRAUM_SERVICE_VALUE_LENGTH, BLE_UUID_TRAUM_BASE_CHARACTERISTC_2_UUID, 1, 0);
        //add characteristic for encoding parameters
        uint8_t value_encoding[CODE_CHAR_VALUE_LENGTH]  = {0x00};
        traum_char_add(p_traum_service, &p_traum_service->char_code_handle, value_encoding, CODE_CHAR_VALUE_LENGTH, BLE_UUID_TRAUM_CODE_CHARACTERISTC_UUID, 1, 0);
        
        //add characteristic for config
        uint8_t value_conf[CONF_CHAR_VALUE_LENGTH] = {0x00};
        traum_conf_char_add(p_traum_service, &p_traum_service->char_conf_handle, value_conf, CONF_CHAR_VALUE_LENGTH, BLE_UUID_TRAUM_CONF_CHARACTERISTC_UUID);
        
        
	
    // Print messages to Segger Real Time Terminal
    // UNCOMMENT THE FOUR LINES BELOW AFTER INITIALIZING THE SERVICE OR THE EXAMPLE WILL NOT COMPILE.
    //SEGGER_RTT_WriteString(0, "Executing our_service_init().\n"); // Print message to RTT to the application flow
    //SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_OUR_SERVICE
    //SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
    //SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_traum_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
}


// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void traum_eeg_data_characteristic_update(ble_traum_t *p_traum_service)
{
    //checking semaphore
    //this is necessary becausethere might be strange behavior if a second char_update is called during full packet transmission. so better be safe.
    //with the while loop the second call is unnecessary anyways, because if there would be new data, it would be handelt automatically
    if (ble_traum_char_update_semaphore) {
        ble_traum_char_update_semaphore = false; //disable access

        //check if new data is present. while loop breaks if char_id < 0
        int8_t char_id = spi_new_data();
        while ((p_traum_service->conn_handle != BLE_CONN_HANDLE_INVALID) && (char_id >= 0))
	{
            uint32_t      err_code;

            uint16_t      len = TRAUM_SERVICE_VALUE_LENGTH;
            uint16_t      value_handle;

            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));

            if (char_id == 0) {
                value_handle = p_traum_service->char_base_handle_0.value_handle;
            } else if (char_id == 1) {
                value_handle = p_traum_service->char_base_handle_1.value_handle;
            } else if (char_id == 2) {
                value_handle = p_traum_service->char_base_handle_2.value_handle;
            } else {
                break;
            }

            hvx_params.handle = value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &len;
            hvx_params.p_data = spi_get_data_pointer(); //returns a (unit8_t*) to next data chunk

            err_code = sd_ble_gatts_hvx(p_traum_service->conn_handle, &hvx_params);
            //NRF_LOG_INFO("EEG err: %i 0x%04x", err_code, err_code);
            if (err_code == NRF_SUCCESS || err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING) { //##quick fix
                spi_data_sent(); //inform read-pointer that package was sent successfully
            } else if (err_code == NRF_ERROR_RESOURCES) {
                //NRF_LOG_INFO("BLE send buffer full");
                break;
            } else if (err_code == NRF_ERROR_INVALID_STATE) {
                NRF_LOG_INFO("BLE notifications not enabled");
                break;
            } else  {
                NRF_LOG_INFO("EEG err: %i 0x%04x", err_code, err_code);
                break;
            }

            //check if new data is present. while loop breaks if char_id < 0
            char_id = spi_new_data();

//            NRF_LOG_INFO(" R: %08x -> %08x", (uint32_t*)hvx_params.p_data, *(uint32_t*)hvx_params.p_data);
            //NRF_LOG_INFO(" R up : %08x", (uint32_t*)hvx_params.p_data);
            //NRF_LOG_INFO(" R upp: %08x", *(uint32_t*)hvx_params.p_data);
            //NRF_LOG_INFO(" R up2: %08x", *(uint32_t*)hvx_params.p_data[4]);
	}
        //NRF_LOG_INFO("upd end");
    
        ble_traum_char_update_semaphore = true; //enable access
    }
}


// Function to be called when updating config characteristic value
void traum_battery_status_update(ble_traum_t *p_traum_service, uint8_t * data)
{
    NRF_LOG_INFO("TBU 0.");
    NRF_LOG_FLUSH();
    if (p_traum_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
    NRF_LOG_INFO("TBU 1.");
    NRF_LOG_FLUSH();

            uint32_t       err_code;

            uint16_t               len = CONF_CHAR_VALUE_LENGTH;
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_traum_service->char_conf_handle.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &len;
            hvx_params.p_data = data;

    NRF_LOG_INFO("TBU 2.");
    NRF_LOG_FLUSH();
            err_code = sd_ble_gatts_hvx(p_traum_service->conn_handle, &hvx_params);
    NRF_LOG_INFO("TBU 3.");
    NRF_LOG_FLUSH();
    }
}


// Function to be called when updating encoding characteristic value
void traum_encoding_char_update(ble_traum_t *p_traum_service, uint8_t * data)
{
    //NRF_LOG_INFO("TEU 0.");
    //NRF_LOG_FLUSH();
    if (p_traum_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
    //NRF_LOG_INFO("TEU 1.");
    //NRF_LOG_FLUSH();

            uint32_t       err_code;

            uint16_t               len = CODE_CHAR_VALUE_LENGTH;
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_traum_service->char_code_handle.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &len;
            hvx_params.p_data = data;

    //NRF_LOG_INFO("TEU 2.");
    //NRF_LOG_FLUSH();
            err_code = sd_ble_gatts_hvx(p_traum_service->conn_handle, &hvx_params);
    //NRF_LOG_INFO("TEU 3.");
    //NRF_LOG_FLUSH();
    }
}
