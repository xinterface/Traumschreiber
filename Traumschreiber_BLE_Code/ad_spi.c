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

//ble service handle. needs to be given to traum_eeg_update_characteristic.
ble_traum_t * spi_traum_service;
bool spi_ble_connected_flag = false;
bool spi_ble_notification_flag = false;

//uint8_t spi_bletrans_fullpackage_flag = 0; //it stores the packetnumber of the full-transmission-packages (if not active it has value 0)



// Debug output. called every 1000ms.
static void spi_timer_timeout_handler(void * p_context)
{
   // NRF_LOG_INFO("one sec");
//    NRF_LOG_INFO("skip counter: %i", packetSkipCounter);
//    NRF_LOG_INFO("r,p: %i/%i,c: %i\t\ts,p: %i/%i,c: %i", srb_read_position, srb_write_position, srb_capacity_used, stb_read_position, stb_write_position, stb_capacity_used);
    packetSkipCounter = 0;
    //nrf_gpio_pin_toggle(17);
}



//initiates SPI-read
void spi_transfer_ADread(uint8_t ad_id)
{
    if (spi_ble_connected_flag && spi_ble_notification_flag) { //only read data from AD if BLE connection exists

       // Reset rx buffer
       memset(m_rx_buf[ad_id], 0, m_rlength);

       APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi[ad_id], m_tx_buf, m_length, m_rx_buf[ad_id], m_length));
    }
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

    uint8_t ad_id;
    if (pin == SPI_0_DRDY) {
        ad_id = 0;
    } else if (pin == SPI_1_DRDY) {
        ad_id = 1;
    } else if (pin == SPI_2_DRDY) {
        ad_id = 2;
    } else {
       NRF_LOG_INFO("false pin trigger");
       return;
    }
    
    spi_transfer_ADread(ad_id);
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

        uint8_t ad_id = *(uint8_t*)p_context;

        for(int i = 0; i < SPI_READ_CHANNEL_NUMBER;i++) {
            //convert 24Bit SPI value to 32Bit
            //val = (int32_t)(0x007FFFFF && (uint32_t)spi_read_buf[i*SPI_READ_PER_CHANNEL] + 0x000001FF * ((uint32_t)spi_read_buf[i*SPI_READ_PER_CHANNEL] && 0x00800000));
            //it is bytes 1,2,3. pos 0 is header.
            spi_channel_values[ad_id*SPI_READ_CHANNEL_NUMBER+i] = (m_rx_buf[ad_id][i*SPI_READ_PER_CHANNEL+1] << 24 | m_rx_buf[ad_id][i*SPI_READ_PER_CHANNEL+2] << 16 | m_rx_buf[ad_id][i*SPI_READ_PER_CHANNEL+3] << 8) >> 8; //more elegant than the line above
        
            //spi_new_diff_values[i] = val - spi_last_abs_values[i]; //change to last recieved value
            //spi_last_abs_values[i] = val;
        }
        ad_recieved[ad_id] = true;

        //see if all channels were recieved, if yes try to send data
        if (ad_recieved[0] && ad_recieved[1] && ad_recieved[2]) {

            //if there is space in spi_send_buffer
            if (stb_write_capacity + stb_packet_size_w > stb_buffer_length) {
                NRF_LOG_INFO(" stb full (c:%i)", stb_write_capacity);
                packetSkipCounter += 1;
                return;
            } else {

                //copy new data to spi_read_buffer from ad read buffer
                //memcpy(&spi_read_buf[srb_write_position], &spi_channel_values, srb_packet_size);

                if (spi_data_gen_enabled) {
                    for(int8_t i = 0;i < 8; i++) {
                        spi_data_gen_buf[i] = (spi_data_gen_buf[i] + 0x010101) & 0xFFFFFF;
                    }

                    //copy new data to spi_read_buffer from generation buffer
                    //it seems that there is a little/big endian mixup here (ad is other order than ble chip), but since for counters that does not matter for values <256, this is ignored
                    if (spi_data_gen_use_half) {
                        memcpy(&spi_channel_values, &spi_data_gen_buf, 12);
                    } else {
                        memcpy(&spi_channel_values, &spi_data_gen_buf, 32);
                    }
                }

                //filter

                //encode
                spi_encode_data();

                //call BLE send function to try to send the received data
                traum_eeg_data_characteristic_update(spi_traum_service);

            } //end IF here to wait for next round of packets if buffer is full

            ad_recieved[0] = false;
            ad_recieved[1] = false;
            ad_recieved[2] = false;
        }

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

    //resetting buffers
    stb_write_position  = 0;
    stb_read_position   = 0;
    stb_write_capacity  = 0;
    stb_read_capacity   = 0;

    NRF_LOG_INFO(" spi_ble_disconnect called");
}

//keeping track whether BLE stack has notifications enabled.
void spi_ble_notify(uint16_t notify)
{
    if (notify == 1) {
        spi_ble_notification_flag = true;
    } else {
        spi_ble_notification_flag = false;
    }
}


/**
 * @brief Function to fetch the current pointer to the oldest spi data.
 * should probably be called spi_get_(send)_data
 */
void spi_encode_data(void)
{    
    //resetting the send buffer
    memset(&spi_send_buf[stb_write_position], 0, stb_packet_size_w);
    
    uint8_t * spi_read_buf = (uint8_t*)spi_channel_values;
    
    int8_t offset = 0;
    int8_t index = 1;
    int8_t a;

    //for each channel, fetch current value and calculate difference to the last one
    for(int i = 0; i < SPI_READ_CHANNEL_NUMBER-2;i++) { //only 6 channels!
        offset++; //skipp first byte per channel
        for(a = 0;a<3;a++){
            spi_send_buf[stb_write_position+index] = spi_read_buf[offset];
            index++;
            offset++;
        }
    }
    

    //fill in header
    //4 bits for packet ID, 4 bits # packets dropped
    //dropped the 1 bit error Bit
    packetSkipCounter = packetSkipCounter > 15 ? 0xF : packetSkipCounter;
    spi_send_buf[stb_write_position] = (uint8_t)((recieved_packets_counter << 4) | (packetSkipCounter % 16));
    recieved_packets_counter = (recieved_packets_counter + 1) % 16;
    packetSkipCounter = 0;

    //update ringbuffer characteristics
    stb_write_position = (stb_write_position + stb_packet_size_w) % stb_buffer_length;
    stb_write_capacity += stb_packet_size_w;
    stb_read_capacity += stb_packet_size_w;

//    NRF_LOG_INFO(" S: %08x -> %08x  (%i) (c:%i)", (uint32_t*) tmp_buf, *(uint32_t*) tmp_buf, srb_read_position, srb_capacity_used);
//NRF_LOG_INFO(" R: %08x %08x", *(uint32_t*) &spi_read_buf[srb_read_position], *(uint32_t*) &spi_read_buf[srb_read_position+4]);
//NRF_LOG_INFO(" S: %08x %08x", *(uint32_t*) &spi_send_buf[stb_write_position], *(uint32_t*) &spi_send_buf[stb_write_position+4]);
    //    
}


/**
 * @brief Function to see if there is new data available to send.
 */
bool spi_new_data(void)
{
    //if there is data available in spi_send_buffer
    if (stb_read_capacity >= stb_packet_size_r) {
        return true;
    } else {
        //NRF_LOG_INFO(" no new data to send");
        return false;
    }
}

/**
 * @brief Function to fetch the current pointer to the oldest spi data.
 */
uint8_t* spi_get_data_pointer(void)
{    
    return &spi_send_buf[stb_read_position];
}

    //    NRF_LOG_INFO(" S d  : %08x", *(uint32_t*) spi_read_buf[srb_read_position]);
    //    NRF_LOG_INFO(" S p  : %08x", (uint32_t*) spi_read_buf[srb_read_position]);
    //    NRF_LOG_INFO(" S: %08x (%08x) -> %08x  (%i)", (uint32_t*) spi_read_buf[srb_read_position], (uint32_t*) (spi_read_buf + srb_read_position), *(uint32_t*) spi_read_buf[srb_read_position], srb_read_position);
    //    NRF_LOG_INFO(" S: %08x -> %08x  (%i) (c:%i)", (uint32_t*) ret, *(uint32_t*) ret, srb_read_position, srb_capacity_used);
  
//update buffer read pointer and capacity
void spi_data_sent()
{
    stb_read_position = (stb_read_position + stb_packet_size_r) % stb_buffer_length;
    stb_read_capacity -= stb_packet_size_r;
}

//update ble-buffer read pointer and capacity
void spi_ble_sent(uint8_t count)
{
    //do only if there is a BLE-connection and someone listens for the data
    if (spi_ble_connected_flag && spi_ble_notification_flag) {

        stb_write_capacity -= stb_packet_size_r * count;
      //NRF_LOG_INFO(" T pos: %i (c:%i)", stb_read_position, stb_capacity_used);
        
        //call BLE send function to try to send another packet
        traum_eeg_data_characteristic_update(spi_traum_service);
    }
}



uint16_t spi_read_battery_status() {
        
        int ad_id = 0;
        
    NRF_LOG_INFO("SPI BS 0.");
    NRF_LOG_FLUSH();

        //writing in General User Config 2 to enable SAR read
        uint8_t tx_buf2[] = ADREG_GENERAL_USER_CONFIG_2; //len = 2
        uint8_t tx_buf_len = 2;
        tx_buf2[1] = tx_buf2[1] | ADREG_GENERAL_USER_CONFIG_2_BYTE_OR;
        //NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf1);
        uint32_t err_code = 0x11;
        while (err_code == 0x11) {
        err_code = nrf_drv_spi_transfer(&spi[ad_id], tx_buf2, tx_buf_len, m_rx_buf[ad_id], tx_buf_len);
        
        NRF_LOG_INFO("spi bs 0.1: %04x", err_code);
        NRF_LOG_FLUSH();
    nrf_delay_ms(1000);
        }
        APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SPI BS 1.");
    NRF_LOG_FLUSH();
        //writing in GLOBAL DIAGNOSTICS MUX REGISTER to set SAR to read out battery status
        uint8_t tx_buf_sar[] = ADREG_SAR_MUX; //len=2
        tx_buf_len = 2;
        err_code = nrf_drv_spi_transfer(&spi[ad_id], tx_buf_sar, tx_buf_len, m_rx_buf[ad_id], tx_buf_len);

        NRF_LOG_INFO("spi bs 1.1: %04x", err_code);
        NRF_LOG_FLUSH();
        APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SPI BS 2.");
    NRF_LOG_FLUSH();
       // read battery status
       memset(m_rx_buf[ad_id], 0, tx_buf_len);
       nrf_drv_spi_transfer(&spi[ad_id], m_tx_buf, tx_buf_len, m_rx_buf[ad_id], tx_buf_len);

    NRF_LOG_INFO("SPI BS 3.");
    NRF_LOG_FLUSH();
       return (uint16_t)*m_rx_buf[ad_id];

}

void spi_config_update(const uint8_t* value_p) //it's 'const' because there is a warning otherwise
{
    uint8_t value = value_p[0];
    union data_union {
      uint8_t send_data[4];
      struct{uint8_t a; uint8_t b; uint16_t status;} battery;
    } data;
    memset(&data.send_data, 0, 4);
    //NRF_LOG_INFO("config update");
    //NRF_LOG_FLUSH();

    //writing gain level to channel registers
    uint8_t cc = ADREG_CHANNEL_CONFIG_GAIN_MASK & (value << 0);
    uint8_t tx_buf[] = {0x00, cc, 0x01, cc, 0x02, cc, 0x03, cc, 0x04, cc, 0x05, cc, 0x06, cc, 0x07, cc,}; //len 16
    uint8_t tx_buf_len = 16;

    for(int i=1;i<AD_NUMBER;i++) {
        uint32_t err_code = nrf_drv_spi_transfer(&spi[i], tx_buf, tx_buf_len, m_rx_buf[i], tx_buf_len);
        
        NRF_LOG_INFO("spi conf 02.%i: %04x", i, err_code);
        NRF_LOG_FLUSH();
        APP_ERROR_CHECK(err_code);
    }

    spi_data_gen_enabled = value & 0x20;
    spi_data_gen_use_half = value & 0x10;

    NRF_LOG_INFO("SPI config updated.");
    NRF_LOG_FLUSH();     


    if (value & 0x08) {
        //nrf_delay_ms(2000);
        //NRF_LOG_INFO("SPI config updated2.");
        //NRF_LOG_FLUSH();   

        //read out battery status
        //data.battery.status = spi_read_battery_status();
        data.battery.status = 0x201;
        //send and reset battery bit while doing so
        data.send_data[0] = value & 0xF7;
        traum_battery_status_update(spi_traum_service, data.send_data);

        NRF_LOG_INFO("SPI config, battery send.");
        NRF_LOG_FLUSH();
    }

}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    if (!nrf_drv_gpiote_is_init()) //check if gpiote is already initialized
    {
        err_code = nrf_drv_gpiote_init(); //if not, do so
        APP_ERROR_CHECK(err_code);
    }

    
    //seting trigger for \DRDY-signal from AD

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  //  in_config.pull = NRF_GPIO_PIN_PULLUP;
    
    //set spi triggers
    nrf_drv_gpiote_in_init(SPI_0_DRDY, &in_config, spi_trigger_pin_handler);
    nrf_drv_gpiote_in_init(SPI_1_DRDY, &in_config, spi_trigger_pin_handler);
    nrf_drv_gpiote_in_init(SPI_2_DRDY, &in_config, spi_trigger_pin_handler);

    //enable spi triggers
    nrf_drv_gpiote_in_event_enable(SPI_0_DRDY, true);
    nrf_drv_gpiote_in_event_enable(SPI_1_DRDY, true);
    nrf_drv_gpiote_in_event_enable(SPI_2_DRDY, true);
}


void spi_init(void)
{

    NRF_LOG_INFO("timer init");
    NRF_LOG_FLUSH();

    
    // Initialize timer module.
    APP_ERROR_CHECK(app_timer_create(&m_spi_char_timer_id, APP_TIMER_MODE_REPEATED, spi_timer_timeout_handler));
    ret_code_t err_code = app_timer_start(m_spi_char_timer_id, SPI_CHAR_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);


      
    //NRF_LOG_INFO("SPI init for write General User Register.");
    //NRF_LOG_FLUSH();
    
    for(int i=0;i<AD_NUMBER;i++) {

        NRF_LOG_INFO("Initialising SPI%i", i);

        //setting Settings in AD
        //SPI Interface setup
        nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
        if (i == 0) {
            spi_config.ss_pin   = SPI_0_CS;
            spi_config.miso_pin = SPI_0_SDO;
            spi_config.mosi_pin = SPI_0_SDI;
            spi_config.sck_pin  = SPI_0_SCLK;
        } else if (i == 1) {
            spi_config.ss_pin   = SPI_1_CS;
            spi_config.miso_pin = SPI_1_SDO;
            spi_config.mosi_pin = SPI_1_SDI;
            spi_config.sck_pin  = SPI_1_SCLK;
        } else if (i == 2) {
            spi_config.ss_pin   = SPI_2_CS;
            spi_config.miso_pin = SPI_2_SDO;
            spi_config.mosi_pin = SPI_2_SDI;
            spi_config.sck_pin  = SPI_2_SCLK;
        }
        spi_config.frequency = SPI_FREQUENCY_FREQUENCY_M8;
        APP_ERROR_CHECK(nrf_drv_spi_init(&spi[i], &spi_config, NULL, NULL));


        //writing in SRC Config to set AD ORD to 250Hz
        uint8_t tx_buf_src[] = ADREG_DECIMATION_RATE_N_MAD; //len=4
        uint8_t tx_buf_len = 4;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi[i], tx_buf_src, tx_buf_len, m_rx_buf[i], tx_buf_len));

        //writing in SRC Update to update AD ORD to 250Hz
        uint8_t tx_buf_src_u[] = ADREG_SRC_MODE_UPDATE_SET; //len=2
        tx_buf_len = 2;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi[i], tx_buf_src_u, tx_buf_len, m_rx_buf[i], tx_buf_len));
        nrf_delay_ms(20); //this should be 'at least two MCLK periods'
        uint8_t tx_buf_src_c[] = ADREG_SRC_MODE_UPDATE_CLEAR; //len=2
        tx_buf_len = 2;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi[i], tx_buf_src_c, tx_buf_len, m_rx_buf[i], tx_buf_len));

        //add GUC1 change
        //writing in General User Config 1 to enable internal reference
        uint8_t tx_buf1[] = ADREG_GENERAL_USER_CONFIG_1; //len = 2
        tx_buf_len = 2;
        //NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf1); 
        //NRF_LOG_FLUSH(); 
        tx_buf1[1] = tx_buf1[1] | ADREG_GENERAL_USER_CONFIG_1_BYTE_OR; //##commented out while testing    
        //NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf1);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi[i], tx_buf1, tx_buf_len, m_rx_buf[i], tx_buf_len));

        //writing in General User Config to enable AD for data output via SPI
        uint8_t tx_buf[] = ADREG_GENERAL_USER_CONFIG_3; //len = 2
        tx_buf_len = 2;
        //NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf); 
        //NRF_LOG_FLUSH(); 
        tx_buf[1] = tx_buf[1] | ADREG_GENERAL_USER_CONFIG_3_BYTE_OR; //##commented out while testing    
        //NRF_LOG_INFO(" R_tx: %04x", *(uint16_t*) tx_buf);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi[i], tx_buf, tx_buf_len, m_rx_buf[i], tx_buf_len));
   
        nrf_delay_ms(200);
        nrf_drv_spi_uninit(&spi[i]) ; //need to uninititialize because otherwise reinitialisation crashes
        //NRF_LOG_INFO("SPI init2.");
        //NRF_LOG_FLUSH();
    
        //initializing SPI for cyclic read operation
        //the pointer to spi_instances is needed to distinguish, which AD send the trigger
        APP_ERROR_CHECK(nrf_drv_spi_init(&spi[i], &spi_config, spi_event_handler, &spi_instances_id[i]));
    }

    //init GPIO and setting trigger for \DRDY-signal
    gpio_init();
  
    NRF_LOG_INFO("SPI init fin.");
    NRF_LOG_FLUSH();

}
