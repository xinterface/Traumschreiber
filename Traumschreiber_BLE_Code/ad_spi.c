/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
#include "ad_spi.h"

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"
#include "nrf_drv_timer.h"
#include "traumschreiber_service.h"


// timer
APP_TIMER_DEF(m_spi_char_timer_id);
#define SPI_CHAR_TIMER_INTERVAL     APP_TIMER_TICKS(1000) // 1000 ms intervals

// data generation timer
APP_TIMER_DEF(m_spi_dg_timer_id);
#define SPI_DG_TIMER_INTERVAL     APP_TIMER_TICKS(4) // 1000 ms intervals

//ble service handle. needs to be given to traum_eeg_update_characteristic.
ble_traum_t * spi_traum_service;
bool spi_ble_connected_flag = false;
bool spi_ble_notification_flag = false;

uint8_t spi_bletrans_fullpackage_flag = 0; //it stores the packetnumber of the full-transmission-packages (if not active it has value 0)



// Debug output. called every 1000ms.
static void spi_timer_timeout_handler(void * p_context)
{
   // NRF_LOG_INFO("one sec");
//    NRF_LOG_INFO("skip counter: %i", packetSkipCounter);
//    NRF_LOG_INFO("r,p: %i/%i,c: %i\t\ts,p: %i/%i,c: %i", srb_read_position, srb_write_position, srb_capacity_used, stb_read_position, stb_write_position, stb_capacity_used);
    packetSkipCounter = 0;
    //nrf_gpio_pin_toggle(17);
}


// Debug function. generates data and emulates spi_event_handler()
static void spi_data_generation_timeout_handler(void * p_context)
{
    //only process data if there is a BLE-connection and someone listens for the data
    if (spi_ble_connected_flag && spi_ble_notification_flag) {
        
        for(int8_t i = 0;i < 8; i++) {
            //spi_data_gen_buf[i*4+3] += i + 1;
        }
    
        //if there is a spi_read_buffer-overflow, drop oldest package (and therefor make space for new one)
        if (srb_capacity_used + srb_packet_size > srb_buffer_length) {
            int pre = srb_read_position;
            srb_read_position = (srb_read_position + srb_packet_size) % srb_buffer_length;
            //it's 'srb_packet_size' because we need to skip a full data set to keep integrity of data
            srb_capacity_used -= srb_packet_size;
            //NRF_LOG_INFO(" W skip: %i -> %i", pre, srb_read_position);
            packetSkipCounter += 1;
        }

        //copy new data to spi_read_buffer and update buffer-pointers
        memcpy(&spi_read_buf[srb_write_position], &spi_data_gen_buf, srb_packet_size);
        NRF_LOG_INFO("counter: %i, read:%i%i%i%i", spi_data_gen_buf[1],spi_read_buf[srb_write_position+0],spi_read_buf[srb_write_position+1],spi_read_buf[srb_write_position+2],spi_read_buf[srb_write_position+3]);

        srb_write_position = (srb_write_position + srb_packet_size) % srb_buffer_length;
        srb_capacity_used += srb_packet_size;

        
        //call BLE send function to try to send the received data
        traum_eeg_data_characteristic_update(spi_traum_service);
    }
            
}


//initiates SPI-read
void spi_transfer_ADread(void)
{
    if (spi_ble_connected_flag && spi_ble_notification_flag) { //only read data from AD if BLE connection exists
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    }
    //nrf_gpio_pin_set(19); //LED3
}


/**
 * @brief GPIO user event handler.
 * @param event
 * triggered when conversion of AD is complete and data can be accessed vie SPI
 * calls spi_transfer_ADread to initiate SPI-read
 */
void spi_trigger_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //skipping events so log buffer does not overflow (helps with debugging, not of use later on)
    if (triggerSkipCounter < triggerSkipCounterMax) {
        triggerSkipCounter = triggerSkipCounter + 1;
        return;
    }
    triggerSkipCounter = 0;

    //nrf_drv_gpiote_out_toggle(PIN_OUT);
    //nrf_gpio_pin_clear(18); //LED2

       // Reset rx buffer
       memset(m_rx_buf, 0, m_rlength);
    
    spi_transfer_ADread();

    //nrf_gpio_pin_set(18); //LED2
}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    //nrf_drv_gpiote_out_clear(17);
    //nrf_gpio_pin_clear(17); //LED1
    ret_code_t err_code;

    if (!nrf_drv_gpiote_is_init()) //check if gpiote is already initialized
    {
        err_code = nrf_drv_gpiote_init(); //if not, do so
        APP_ERROR_CHECK(err_code);
    }

    //seting trigger for \DRDY-signal from AD

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  //  in_config.pull = NRF_GPIO_PIN_PULLUP;
    
    //if (!SPI_DATA_GEN_FLAG) { //if we are not using the data generator, set spi trigger
    //commented out the if because the 'enable' below should do the trick
        err_code = nrf_drv_gpiote_in_init(SPI_0_DRDY, &in_config, spi_trigger_pin_handler);
        APP_ERROR_CHECK(err_code);
    //}

    //the if is commented out, because we use the trigger either way
    //if (!SPI_DATA_GEN_FLAG) { //if we are not using the data generator, enable spi trigger
        nrf_drv_gpiote_in_event_enable(SPI_0_DRDY, true);
    //}
}


/**
 * @brief SPI user event handler.
 * @param event
 * called when SPI-transfer is completed
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    //only process data if there is a BLE-connection and someone listens for the data
    if (spi_ble_connected_flag && spi_ble_notification_flag) {
        //nrf_gpio_pin_clear(20); //LED4
        //nrf_gpio_pin_clear(19); //LED3
        //nrf_gpio_pin_clear(18); //LED2
    
        //if there is a spi_read_buffer-overflow, drop oldest package (and therefor make space for new one)
        if (srb_capacity_used + srb_packet_size > srb_buffer_length) {
            int pre = srb_read_position;
            srb_read_position = (srb_read_position + srb_packet_size) % srb_buffer_length;
            //it's 'srb_packet_size' because we need to skip a full data set to keep integrity of data
            srb_capacity_used -= srb_packet_size;
            //NRF_LOG_INFO(" W skip: %i -> %i", pre, srb_read_position);
            packetSkipCounter += 1;
        }


        if (spi_data_gen_enabled) {
            for(int8_t i = 0;i < 8; i++) {
                spi_data_gen_buf[i] = (spi_data_gen_buf[i] + 0x010101) & 0xFFFFFF;
            }

            //copy new data to spi_read_buffer from generation buffer
            if (spi_data_gen_use_half) {
                memcpy(&spi_read_buf[srb_write_position], &m_rx_buf, srb_packet_size/2);
                memcpy(&spi_read_buf[srb_write_position+srb_packet_size/2], &spi_data_gen_buf, srb_packet_size/2);
            } else {
                memcpy(&spi_read_buf[srb_write_position], &spi_data_gen_buf, srb_packet_size);
            }
        } else {
        
            //copy new data to spi_read_buffer from ad read buffer
            memcpy(&spi_read_buf[srb_write_position], &m_rx_buf, srb_packet_size);
        }
        // and update buffer-pointers
        srb_write_position = (srb_write_position + srb_packet_size) % srb_buffer_length;
        srb_capacity_used += srb_packet_size;

        //call BLE send function to try to send the received data
        traum_eeg_data_characteristic_update(spi_traum_service);
            

        //nrf_gpio_pin_set(20); //LED4
        //nrf_gpio_pin_set(18); //LED2
    }
}


//keeping track whether BLE is connected or not.
void spi_ble_connect(ble_traum_t * p_traum_service)
{
    spi_traum_service = p_traum_service;
    spi_ble_connected_flag = true;
    NRF_LOG_INFO(" spi_ble_connect called");
}
void spi_ble_disconnect()
{
    spi_traum_service = NULL;
    spi_ble_connected_flag = false;
    NRF_LOG_INFO(" spi_ble_disconnect called");
}

//keeping track whether BLE stack has notifications enabled.
void spi_ble_notify(uint16_t notify)
{
    //uint32_t cccd_value; //should be 1 or 2 (but actually isn't. though it was 0 or 1 once...
    //// Pupulate ble_gatts_value_t structure to hold received data and metadata.
    //ble_gatts_value_t cccd_data;
    //cccd_data.len = sizeof(uint32_t);
    //cccd_data.offset = 0;
    //cccd_data.p_value = (uint8_t*)&cccd_value;
    //sd_ble_gatts_value_get(p_traum_service->conn_handle, p_traum_service->char_base_handle.cccd_handle, &cccd_data);
    
    if (notify == 1) {
        spi_ble_notification_flag = true;
    } else {
        spi_ble_notification_flag = false;
    }

//    NRF_LOG_INFO(" spi_ble_notify called");
}


/**
 * @brief Function to see if there is new data available to send.
 */
bool spi_new_data(void)
{
    //if there is data available in spi_read_buffer or full packets are currently being transmitted
    if (srb_capacity_used >= srb_packet_size || spi_bletrans_fullpackage_flag != 0) {
        //and if there is space in spi_send_buffer
        if (stb_capacity_used + stb_packet_size >= stb_buffer_length) {
            NRF_LOG_INFO(" stb full (c:%i)", stb_capacity_used);
            return false;
        }
        return true;
    } else {
        //NRF_LOG_INFO(" no new data to send");
        return false;
    }
}

/**
 * @brief Function to fetch the current pointer to the oldest spi data.
 * should probably be called spi_get_(send)_data
 */
uint8_t* spi_read_data(void)
{    
    //resetting the send buffer
    memset(&spi_send_buf[stb_write_position], 0, stb_packet_size);


    if (srb_read_position + srb_packet_size > srb_buffer_length) {
    //this can never occur! but it's a good security check!
        NRF_LOG_INFO(" spi_read_data overflow");
    } else {
        //old solution
        //memcpy(&spi_send_buf[stb_write_position], &spi_read_buf[srb_read_position], stb_packet_size);
        
            int8_t offset = 0;
            int8_t index = 1;
            int8_t a;

            //for each channel, fetch current value and calculate difference to the last one
            for(int i = 0; i < SPI_READ_CHANNEL_NUMBER-2;i++) { //only 6 channels!
                offset++; //skipp first byte per channel
                for(a = 0;a<3;a++){
                    spi_send_buf[stb_write_position+index] = spi_read_buf[srb_read_position+offset];
                    index++;
                    offset++;
                }
            }
        

        //fill in header
        //4 bits for packet ID, 1 bit error, 3 bits value length (with offset spi_min_bits_per_channel)
        spi_send_buf[stb_write_position] = (uint8_t)((recieved_packets_counter << 4) | (0 << 3) | (7));
        recieved_packets_counter = (recieved_packets_counter + 1) % 16;

    }
//    NRF_LOG_INFO(" S: %08x -> %08x  (%i) (c:%i)", (uint32_t*) tmp_buf, *(uint32_t*) tmp_buf, srb_read_position, srb_capacity_used);
NRF_LOG_INFO(" R: %08x %08x", *(uint32_t*) &spi_read_buf[srb_read_position], *(uint32_t*) &spi_read_buf[srb_read_position+4]);
NRF_LOG_INFO(" S: %08x %08x", *(uint32_t*) &spi_send_buf[stb_write_position], *(uint32_t*) &spi_send_buf[stb_write_position+4]);
    //    

    return &spi_send_buf[stb_write_position];
}

    //    NRF_LOG_INFO(" S d  : %08x", *(uint32_t*) spi_read_buf[srb_read_position]);
    //    NRF_LOG_INFO(" S p  : %08x", (uint32_t*) spi_read_buf[srb_read_position]);
    //    NRF_LOG_INFO(" S: %08x (%08x) -> %08x  (%i)", (uint32_t*) spi_read_buf[srb_read_position], (uint32_t*) (spi_read_buf + srb_read_position), *(uint32_t*) spi_read_buf[srb_read_position], srb_read_position);
    //    NRF_LOG_INFO(" S: %08x -> %08x  (%i) (c:%i)", (uint32_t*) ret, *(uint32_t*) ret, srb_read_position, srb_capacity_used);
  
//update buffer read pointer and capacity
void spi_data_sent()
{
    //the if handels that during full packet transmission the read buffer is only used once
    //    the if will only be triggered once during that transmission (after last packet)
    //    for all other differential packets it will be normally triggered
    if (spi_bletrans_fullpackage_flag == 0) {
        srb_read_position = (srb_read_position + srb_packet_size) % srb_buffer_length;
        srb_capacity_used -= srb_packet_size;
    }
    stb_write_position = (stb_write_position + stb_packet_size) % stb_buffer_length;
    stb_capacity_used += stb_packet_size;
//    NRF_LOG_INFO(" R pos: %i (c:%i)", srb_read_position, srb_capacity_used);
    //NRF_LOG_INFO(" R pos: %i", srb_read_position);
}

//update ble-buffer read pointer and capacity
void spi_ble_sent(uint8_t count)
{
    if (stb_capacity_used >= 0) {
        stb_read_position = (stb_read_position + stb_packet_size * count) % stb_buffer_length;
        stb_capacity_used -= stb_packet_size * count;
//      NRF_LOG_INFO(" R pos: %i (c:%i)", srb_read_position, srb_capacity_used);
        //NRF_LOG_INFO(" R pos: %i", srb_read_position);
    }
}



void spi_config_update(uint8_t value)
{

    NRF_LOG_INFO("config update");
    NRF_LOG_FLUSH();

    //stopping data generation timer
    app_timer_stop(m_spi_dg_timer_id);
    //disabling data_ready-interrupt
    nrf_drv_gpiote_in_event_disable(SPI_0_DRDY);

    
    NRF_LOG_INFO("spi conf 01");
    NRF_LOG_FLUSH();
    //writing gain level to channel registers
    //uint8_t tx_buf[] = ADREG_CHANNEL_CONFIG; //len = 2
    uint8_t cc = ADREG_CHANNEL_CONFIG_GAIN_MASK & (value << 0);
    uint8_t tx_buf[] = {0x00, cc, 0x01, cc, 0x02, cc, 0x03, cc, 0x04, cc, 0x05, cc, 0x06, cc, 0x07, cc,}; //len 16
    uint8_t tx_buf_len = 16;
    //tx_buf[1] = tx_buf[1] | (ADREG_CHANNEL_CONFIG_GAIN_MASK & (value << 0)); //
    
    NRF_LOG_INFO("spi conf 02");
    NRF_LOG_FLUSH();
   // for(uint8_t channel = 0;channel < 8;channel += 1) {
   //     tx_buf[0] = channel; //
        uint32_t err_code = nrf_drv_spi_transfer(&spi, tx_buf, tx_buf_len, m_rx_buf, tx_buf_len);
        
        NRF_LOG_INFO("spi conf 02.1: %04x", err_code);
        NRF_LOG_FLUSH();
        APP_ERROR_CHECK(err_code);
    //    nrf_delay_ms(50);
    //}

    NRF_LOG_INFO("spi conf 03");
    NRF_LOG_FLUSH();
    
    spi_data_gen_enabled = value & 0x20;
    spi_data_gen_use_half = value & 0x10;
    //starting data generation timer if set
    if (spi_data_gen_enabled) {
        NRF_LOG_INFO("SPI data generation start up.");
        //uint8_t err_code = app_timer_start(m_spi_dg_timer_id, SPI_DG_TIMER_INTERVAL, NULL);
        //APP_ERROR_CHECK(err_code);
        nrf_drv_gpiote_in_event_enable(SPI_0_DRDY, true); //do this in any case, because generation is now ad triggered
    } else { //els enable data_ready-interrupt
        nrf_drv_gpiote_in_event_enable(SPI_0_DRDY, true);
    }

    NRF_LOG_INFO("SPI config updated.");
    NRF_LOG_FLUSH();

}




void spi_init(void)
{

    NRF_LOG_INFO("timer init");
    NRF_LOG_FLUSH();

    
    // Initialize timer module.
   // ret_code_t err_code = app_timer_init();
   // APP_ERROR_CHECK(err_code);

    APP_ERROR_CHECK(app_timer_create(&m_spi_char_timer_id, APP_TIMER_MODE_REPEATED, spi_timer_timeout_handler));
    ret_code_t err_code = app_timer_start(m_spi_char_timer_id, SPI_CHAR_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);


      
    NRF_LOG_INFO("SPI init for write General User Register.");
    NRF_LOG_FLUSH();
    
    //setting Settings in AD
    //SPI Interface setup
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_0_CS;
    spi_config.miso_pin = SPI_0_SDO;
    spi_config.mosi_pin = SPI_0_SDI;
    spi_config.sck_pin  = SPI_0_SCLK;
    spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M8;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));


    //writing in SRC Config to set AD ORD to 250Hz
    uint8_t tx_buf_src[] = ADREG_DECIMATION_RATE_N_MAD; //len=4
    uint8_t tx_buf_len = 4;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf_src, tx_buf_len, m_rx_buf, tx_buf_len));

    //writing in SRC Update to update AD ORD to 250Hz
    uint8_t tx_buf_src_u[] = ADREG_SRC_MODE_UPDATE_SET; //len=2
    tx_buf_len = 2;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf_src_u, tx_buf_len, m_rx_buf, tx_buf_len));
    nrf_delay_ms(20); //this should be 'at least two MCLK periods'
    uint8_t tx_buf_src_c[] = ADREG_SRC_MODE_UPDATE_CLEAR; //len=2
    tx_buf_len = 2;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf_src_c, tx_buf_len, m_rx_buf, tx_buf_len));

    //add GUC1 change
    //writing in General User Config 1 to enable internal reference
    uint8_t tx_buf1[] = ADREG_GENERAL_USER_CONFIG_1; //len = 2
    tx_buf_len = 2;
    NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf1); 
    NRF_LOG_FLUSH(); 
    tx_buf1[1] = tx_buf1[1] | ADREG_GENERAL_USER_CONFIG_1_BYTE_OR; //##commented out while testing    
    NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf1);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf1, tx_buf_len, m_rx_buf, tx_buf_len));

    //writing in General User Config to enable AD for data output via SPI
    uint8_t tx_buf[] = ADREG_GENERAL_USER_CONFIG_3; //len = 2
    tx_buf_len = 2;
    NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf); 
    NRF_LOG_FLUSH(); 
    tx_buf[1] = tx_buf[1] | ADREG_GENERAL_USER_CONFIG_3_BYTE_OR; //##commented out while testing    
    NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf, tx_buf_len, m_rx_buf, tx_buf_len));
   
    NRF_LOG_FLUSH();
    nrf_delay_ms(200);
    nrf_drv_spi_uninit(&spi) ; //need to uninititialize because otherwise reinitialisation crashes
    NRF_LOG_INFO("SPI init2.");
    NRF_LOG_FLUSH();
    
    //initializing SPI for cyclic read operation
 //   nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_0_CS;
    spi_config.miso_pin = SPI_0_SDO;
    spi_config.mosi_pin = SPI_0_SDI;
    spi_config.sck_pin  = SPI_0_SCLK;
    spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M8;
    //spi_config.irq_priority = 2; //0 and 1 are reserved
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    
  
    NRF_LOG_INFO("SPI init fin.");
    NRF_LOG_FLUSH();

    //init GPIO and setting trigger for \DRDY-signal
    gpio_init();

    
    //define data generation timer
    APP_ERROR_CHECK(app_timer_create(&m_spi_dg_timer_id, APP_TIMER_MODE_REPEATED, spi_data_generation_timeout_handler));
    //starting data generation timer
    if (spi_data_gen_enabled) {
        NRF_LOG_INFO("SPI data generation start up.");
        //err_code = app_timer_start(m_spi_dg_timer_id, SPI_DG_TIMER_INTERVAL, NULL);
        //APP_ERROR_CHECK(err_code);
    }

    NRF_LOG_INFO("SPI example started.");
    NRF_LOG_FLUSH();

}
