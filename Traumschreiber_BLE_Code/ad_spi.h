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

 #ifndef AD_SPI_H_
#define AD_SPI_H_


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

#define ARM_MATH_CM4
#include "arm_math.h"




#define SPI_0_DRDY 17 // connected to \DRDY-signal from AD0
#define SPI_0_SCLK  4 // connected to SCLK-signal from AD0
#define SPI_0_CS    6 // connected to CS-signal from AD0
#define SPI_0_SDI  15 // connected to SDI-signal from AD0
#define SPI_0_SDO  16 // connected to SDO-signal from AD0

#define SPI_1_DRDY 14 // connected to \DRDY-signal from AD1
#define SPI_1_SCLK  19 // connected to SCLK-signal from AD1
#define SPI_1_CS    8 // connected to CS-signal from AD1
#define SPI_1_SDI  7 // connected to SDI-signal from AD1
#define SPI_1_SDO  5 // connected to SDO-signal from AD1

#define SPI_2_DRDY 25 // connected to \DRDY-signal from AD2
#define SPI_2_SCLK  22 // connected to SCLK-signal from AD2
#define SPI_2_CS    24 // connected to CS-signal from AD2
#define SPI_2_SDI  28 // connected to SDI-signal from AD2
#define SPI_2_SDO  29 // connected to SDO-signal from AD2

#define AD_NUMBER      3
static const uint8_t spi_instances[] = {0,1,2}; /**< SPI instance indices. */
static uint8_t spi_instances_id[] = {0,1,2}; /**< SPI instance indices for ini-funcion callback context (needs to be !const). */
static const nrf_drv_spi_t spi[] = {NRF_DRV_SPI_INSTANCE(0), NRF_DRV_SPI_INSTANCE(1), NRF_DRV_SPI_INSTANCE(2)};  /**< SPI instance. */

//#define SPI_INSTANCE  0 /**< SPI instance index. */
//static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */


//SPI Constants and buffers
#define SPI_READ_SIGNAL   {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //default read signal for 32 Bytes
#define SPI_READ_CHANNEL_NUMBER   8
#define SPI_CHANNEL_NUMBER_TOTAL SPI_READ_CHANNEL_NUMBER*AD_NUMBER
#define SPI_READ_PER_CHANNEL      4
#define SPI_READ_LENGTH   SPI_READ_CHANNEL_NUMBER*SPI_READ_PER_CHANNEL
static uint8_t       m_tx_buf[] = SPI_READ_SIGNAL;           /**< TX buffer. */
static uint8_t       m_rx_buf[AD_NUMBER][SPI_READ_LENGTH + 1];    /**< RX buffer. */
static const uint8_t m_length  =  sizeof(m_tx_buf);        /**< Transfer length. */
static const uint8_t m_rlength =  sizeof(m_rx_buf[0]);        /**< RX Buffer length. */

#define ADREG_GENERAL_USER_CONFIG_1 {0x11, 0x24} //write signal for register with standart settings
#define ADREG_GENERAL_USER_CONFIG_2 {0x12, 0x09} //write signal for register with standart settings
#define ADREG_GENERAL_USER_CONFIG_3 {0x13, 0x80} //write signal for register with standart settings
#define ADREG_GENERAL_USER_CONFIG_3_BYTE_OR 0x10 //Byte for OR operation with GUC3-Register to set Bit#4
#define ADREG_GENERAL_USER_CONFIG_1_BYTE_OR 0x10 //Byte for OR operation with GUC1-Register to set Bit#4
#define ADREG_GENERAL_USER_CONFIG_2_BYTE_OR 0x20 //Byte for OR operation with GUC2-Register to set Bit#5

#define ADREG_DECIMATION_RATE_N     {0x60, 0x00, 0x61, 0x80} //write signal for register with standart settings
#define ADREG_DECIMATION_RATE_N_500 {0x60, 0x08, 0x61, 0x00} //write signal for register, ODR = 250Hz (should, but actually seems to be 500Hz)
#define ADREG_DECIMATION_RATE_N_250 {0x60, 0x0F, 0x61, 0xFF} //write signal for register, ODR = 250Hz (should not work, but seems to do 250Hz)

#define ADREG_SRC_MODE_UPDATE_CLEAR {0x64, 0x00} //write signal for register with standart settings
#define ADREG_SRC_MODE_UPDATE_SET   {0x64, 0x01} //write signal for register with update instruction

#define ADREG_CHANNEL_CONFIG {0x00, 0x00} //write signal for channel 0 with standart settings
#define ADREG_CHANNEL_CONFIG_GAIN_MASK 0xC0 //mask to code gain into channel config

#define ADREG_SAR_MUX {0x16, 0x00} //write signal to set SAR to read out battery status


static uint8_t triggerSkipCounter[AD_NUMBER] = {0}; //counter used to skip trigger events for downsampling
static uint8_t triggerSkipCounterMax = 2; // how many events are skipped (each 'triggerSkipCounterMax + 1' package is taken)

static uint16_t packetSkipCounter = 0;


extern bool spi_ble_connected_flag;
extern uint8_t spi_ble_notification_flag;
extern uint8_t spi_ble_notification_threshold;
extern bool  ad_recieved[];
extern uint8_t  ad_converted[];


//debug
static uint16_t  recieved_packets_counter      = 0;
static uint16_t  recieved_packets_counter_max  = 16;
static uint16_t  send_packets_counter          = 0;
static uint16_t  collected_packets_counter     = 0;
static uint32_t  buffer_usage_counter          = 0;
static uint8_t debug_flag = 0;


//filtering
//notch (50Hz)
#define IIR_ORDER     6
static float32_t m_biquad_state[SPI_CHANNEL_NUMBER_TOTAL][IIR_ORDER];
static float32_t m_biquad_coeffs_48_o4[5*2] = {9.6508098e-01, -1.5620271e+00, 9.6508098e-01, 1.5685816e+00, -9.6424139e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.6110035e+00, -9.6592170e-01};
static float32_t m_biquad_coeffs_48_o6[5*3] = {9.5097190e-01, -1.5391909e+00, 9.5097190e-01, 1.5788558e+00, -9.5095676e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.5718505e+00, -9.7445124e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.6235807e+00, -9.7592056e-01};
static float32_t m_biquad_coeffs_46_o4[5*2] = {9.3137884e-01, -1.5089085e+00, 9.3137884e-01, 1.5195662e+00, -9.2813122e-01, 1.0000000e+00, -1.6200802e+00, 1.0000000e+00, 1.6056036e+00, -9.3464386e-01};
static float32_t m_biquad_coeffs_46_o6[5*3] = {9.0431875e-01, -1.4650689e+00, 9.0431875e-01, 1.5424813e+00, -9.0420359e-01, 1.0000000e+00, -1.6200802e+00, 1.0000000e+00, 1.5252273e+00, -9.4815099e-01, 1.0000000e+00, -1.6200802e+00, 1.0000000e+00, 1.6297319e+00, -9.5389223e-01};
static arm_biquad_cascade_df2T_instance_f32 iir_instance[SPI_CHANNEL_NUMBER_TOTAL];
static uint8_t  spi_iir_filter_enabled = 1;
//low pass
#define IIRLP_ORDER         6
static float32_t m_lowpass_state[SPI_CHANNEL_NUMBER_TOTAL][IIRLP_ORDER];
static float32_t m_lowpass_coeffs_o4[5*2] = {3.3628151e-03, 6.7256303e-03, 3.3628151e-03, 1.1295059e+00, -3.3775738e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 1.4013136e+00, -6.5967937e-01};
static float32_t m_lowpass_coeffs_o6[5*3] = {8.5753645e-04, 1.7150729e-03, 8.5753645e-04, 8.7762954e-01, -2.0393320e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 9.8240579e-01, -3.4766539e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 1.2385063e+00, -6.9898443e-01};
static arm_biquad_cascade_df2T_instance_f32 lowpass_instance[SPI_CHANNEL_NUMBER_TOTAL];
static uint8_t  spi_lowpass_filter_enabled = 1;
//high pass
#define IIRHP_ORDER         4
static float32_t m_highpass_state[SPI_CHANNEL_NUMBER_TOTAL][IIRHP_ORDER];
static float32_t m_highpass_coeffs_1_0_o4[5*2] = {9.8371517e-01, -1.9674303e+00, 9.8371517e-01, 1.9768914e+00, -9.7704745e-01, 1.0000000e+00, -2.0000000e+00, 1.0000000e+00, 1.9902712e+00, -9.9042840e-01};
static float32_t m_highpass_coeffs_1_7_o4[5*2] = {9.7247345e-01, -1.9449469e+00, 9.7247345e-01, 1.9608460e+00, -9.6129352e-01, 1.0000000e+00, -2.0000000e+00, 1.0000000e+00, 1.9833308e+00, -9.8378341e-01};
static float32_t m_highpass_coeffs_0_8_o2[5*1] = {9.9291659e-01, -1.9858332e+00, 9.9291659e-01, 1.9857830e+00, -9.8588336e-01};
static float32_t m_highpass_coeffs_1_7_o2[5*1] = {9.8500771e-01, -1.9700154e+00, 9.8500771e-01, 1.9697906e+00, -9.7024021e-01};
static arm_biquad_cascade_df2T_instance_f32 highpass_instance[SPI_CHANNEL_NUMBER_TOTAL];
//first order high pass
#define FO_HP_RC         1
#define FO_HP_RC_0_5     0.3183099
#define FO_HP_RC_1_3     0.1224269
#define FO_HP_DT         0.002
static float32_t fo_hp_alpha = FO_HP_RC / (FO_HP_RC + FO_HP_DT);
static const float32_t fo_hp_alpha_0_5 = FO_HP_RC_0_5 / (FO_HP_RC_0_5 + FO_HP_DT);
static const float32_t fo_hp_alpha_1_3 = FO_HP_RC_1_3 / (FO_HP_RC_1_3 + FO_HP_DT);
static float32_t fo_hp_last_y[SPI_CHANNEL_NUMBER_TOTAL] = {0};
static float32_t fo_hp_last_x[SPI_CHANNEL_NUMBER_TOTAL] = {0};
static uint8_t  spi_highpass_filter_enabled = 0;
static uint8_t  spi_fo_hp_filter_enabled = 0;
static uint8_t  spi_running_average_enabled = 1;


//encoding
#define SPI_BLE_USE_NEW_ENCODING_FLAG   1
static float32_t  spi_channel_values[SPI_CHANNEL_NUMBER_TOTAL];
static float32_t  spi_filtered_values[SPI_CHANNEL_NUMBER_TOTAL];
static int32_t  spi_encoded_values[SPI_CHANNEL_NUMBER_TOTAL] = {0};
static float32_t  spi_estimated_variance[SPI_CHANNEL_NUMBER_TOTAL] = {0x100};
static float32_t  spi_estimated_average[SPI_CHANNEL_NUMBER_TOTAL] = {0x1};
static int16_t  spi_max_difval        = 511; //2**spi_max_bits_per_channel
static int16_t  spi_min_difval        = -512;
static uint32_t  spi_ble_difval_mask  = 0x03FF;
static uint8_t  spi_encode_min_shift = 0x0;
static uint8_t  spi_encode_max_shift = 0xF;
static uint32_t  spi_encode_shift[SPI_CHANNEL_NUMBER_TOTAL]     = {0x00}; //how many bits the difval is shifted before encoding (aka, how many bits are dropped)
#define SPI_FACTOR_SAFE_ENCODING_DEFAULT   8
static uint8_t spi_enc_factor_safe_encoding = SPI_FACTOR_SAFE_ENCODING_DEFAULT;
static uint8_t spi_ble_send_devision = 0;
static float32_t spi_enc_estimate_factor_9 = 0.999;
static float32_t spi_enc_estimate_factor_1 = 0.001;
static float32_t spi_enc_estimate_factor_5 = 1.004;
static uint8_t spi_enc_warmup = 1;

static uint8_t  spi_code_send_buf[CODE_CHAR_VALUE_LENGTH] = {0x44};    /**< TX buffer. */


//BLE send ring buffer
#define SPI_BLE_BUFFER_WTRITE_LENGTH    TRAUM_SERVICE_VALUE_LENGTH+(SPI_BLE_USE_NEW_ENCODING_FLAG*10)
#define SPI_BLE_BUFFER_LENGTH      TRAUM_SERVICE_VALUE_LENGTH*48
static uint8_t  spi_send_buf[SPI_BLE_BUFFER_LENGTH];    /**< TX buffer. */
static const uint16_t  stb_buffer_length   = SPI_BLE_BUFFER_LENGTH; //needed because somehow constants can't be used in calculations...
static uint16_t        stb_write_position  = 0;
static const uint16_t  stb_packet_size_w   = SPI_BLE_BUFFER_WTRITE_LENGTH; //needed because somehow constants can't be used in calculations...
static uint16_t        stb_read_position   = 0;
static const uint16_t  stb_packet_size_r   = TRAUM_SERVICE_VALUE_LENGTH; //needed because somehow constants can't be used in calculations...
static uint16_t        stb_write_capacity  = 0; //used
static uint16_t        stb_read_capacity   = 0; //used
static uint8_t         stb_characteristic  = 0; //which characteristic to send on next package
static uint16_t        stb_read_capacity_safety  = 20*TRAUM_SERVICE_VALUE_LENGTH; //used. actual buffer size seems to be 5/char=15, plus some extra


//static uint16_t traumschreiber_battery_status = 0;
union data_union {
    uint8_t send_data[CONF_CHAR_VALUE_LENGTH];
    struct{uint8_t byte_0_3[4]; uint8_t byte_4; uint8_t byte_5; uint16_t battery_status;} reg;
} static spi_config_register;
extern bool battery_read;
static const uint8_t spi_config_register_default[CONF_CHAR_VALUE_LENGTH] = {0x08, 0x00, 0x01, 0x11, 0x0F, 0x80, 0x00, 0x00};


//Debug Data Generation
#define SPI_DATA_GEN_FLAG   0
static uint8_t  spi_data_gen_enabled = SPI_DATA_GEN_FLAG;
static uint8_t  spi_data_gen_use_half = 0;
#define SPI_DATA_GEN_BASE_32   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
static const uint16_t spi_data_gen_add = 0x000201;
static int32_t  spi_data_gen_buf[8] = {spi_data_gen_add,0x10,0x100,0x1000,-spi_data_gen_add,-0x10,-0x100,-0x1000};


// timer event handler
static void spi_timer_timeout_handler(void * p_context);
static void spi_data_generation_timeout_handler(void * p_context);

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context);

void spi_transfer_read(uint8_t ad_id);

/**
 * @brief GPIO user event handler.
 * @param event
 * triggered when conversion of AD is complete and data can be accessed vie SPI
 * initiates SPI-read
 */
void spi_trigger_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void);

/**
 * @brief SPI user event handler.
 * @param event
 * called when SPI-transfer is completed
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);
void spi_data_conversion(uint8_t ad_id);
//void spi_filter_data(void);
void spi_encode_data(void);
void spi_adapt_encoding(void);

void spi_ble_connect(ble_traum_t * p_traum_service);
void spi_ble_disconnect();
void spi_ble_notify(uint16_t notify, ble_traum_t * p_traum_service);


int8_t spi_new_data(void);
uint8_t* spi_get_data_pointer(void);
void spi_data_sent(void);
void spi_ble_sent(uint8_t count);

void spi_read_battery_status();
void spi_send_battery_status();
void spi_config_update(const uint8_t * value_p);
void filter_init(uint8_t highpass, uint8_t lowpass, uint8_t notch);
void spi_init(void);


#endif  /* _ AD_SPI_H__ */
