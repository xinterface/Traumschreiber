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


//filtering
//notch
#define IIR_ORDER     8
#define IIR_NUMSTAGES (IIR_ORDER/2)
static float32_t m_biquad_state[SPI_CHANNEL_NUMBER_TOTAL][IIR_ORDER];
//static float32_t m_biquad_coeffs[5*IIR_NUMSTAGES] = {0.9314, -0.5764, 0.9314, 0.5313, -0.9306, 1.0000, -0.6188, 1.0000, 0.6624, -0.9321};
//static float32_t m_biquad_coeffs_167[5*IIR_NUMSTAGES] = {0.9007, 0.4943, 0.9007, -0.4241, -0.8993, 1.0000, 0.5488, 1.0000, -0.6163, -0.9022};
//static float32_t m_biquad_coeffs_167[5*IIR_NUMSTAGES] = {0.8988, 0.5571, 0.8988, -0.4899, -0.8972, 1.0000, 0.6198, 1.0000, -0.6838, -0.9005};
//static float32_t m_biquad_coeffs_500[5*IIR_NUMSTAGES] = {0.9314, -1.5089, 0.9314, 1.5196, -0.9281, 1.0000, -1.6201, 1.0000, 1.6056, -0.9346};
static float32_t m_biquad_coeffs_500_45_o4[5*2] = {9.1496915e-01, -1.4833783e+00, 9.1496915e-01, 1.4952379e+00, -9.0997660e-01, 1.0000000e+00, -1.6212331e+00, 1.0000000e+00, 1.6035019e+00, -9.2000347e-01};
static float32_t m_biquad_coeffs_500_45_o6[5*3] = {8.8183820e-01, -1.4296653e+00, 8.8183820e-01, 1.5252712e+00, -8.8161862e-01, 1.0000000e+00, -1.6212331e+00, 1.0000000e+00, 1.5017873e+00, -9.3475473e-01, 1.0000000e+00, -1.6212331e+00, 1.0000000e+00, 1.6330429e+00, -9.4362485e-01};
static float32_t m_biquad_coeffs_500_47_o4[5*2] = {9.4808078e-01, -1.5351176e+00, 9.4808078e-01, 1.5440156e+00, -9.4622338e-01, 1.0000000e+00, -1.6191845e+00, 1.0000000e+00, 1.6081027e+00, -9.4994372e-01};
static float32_t m_biquad_coeffs_500_47_o6[5*3] = {9.2735755e-01, -1.5015630e+00, 9.2735755e-01, 1.5603334e+00, -9.2730778e-01, 1.0000000e+00, -1.6191845e+00, 1.0000000e+00, 1.5485861e+00, -9.6138775e-01, 1.0000000e+00, -1.6191845e+00, 1.0000000e+00, 1.6265789e+00, -9.6465474e-01};
static float32_t m_biquad_coeffs_500_48_o4[5*2] = {9.6508098e-01, -1.5620271e+00, 9.6508098e-01, 1.5685816e+00, -9.6424139e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.6110035e+00, -9.6592170e-01};
static float32_t m_biquad_coeffs_500_48_o6[5*3] = {9.5097190e-01, -1.5391909e+00, 9.5097190e-01, 1.5788558e+00, -9.5095676e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.5718505e+00, -9.7445124e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.6235807e+00, -9.7592056e-01};
static float32_t m_biquad_coeffs_500_48_o8[5*4] = {9.3642753e-01, -1.5156503e+00, 9.3642753e-01, 1.5701379e+00, -9.5402128e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.5931994e+00, -9.5519781e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.5747515e+00, -9.8035067e-01, 1.0000000e+00, -1.6185452e+00, 1.0000000e+00, 1.6297992e+00, -9.8155707e-01};
static arm_biquad_cascade_df2T_instance_f32 iir_instance[SPI_CHANNEL_NUMBER_TOTAL];
static uint8_t  spi_iir_filter_enabled = 1;
//low pass
#define IIRLP_ORDER_MAX     12
#define IIRLP_ORDER         8
#define IIRLP_NUMSTAGES (IIRLP_ORDER/2)
static float32_t m_lowpass_state[SPI_CHANNEL_NUMBER_TOTAL][IIRLP_ORDER_MAX];
static float32_t m_lowpass_coeffs_o6[5*3] = {2.3161805e-03, 4.6323611e-03, 2.3161805e-03, 6.8339882e-01, -1.3033381e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 7.7352463e-01, -2.7940087e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 1.0025213e+00, -6.5815877e-01};
static float32_t m_lowpass_coeffs_o8[5*4] = {3.0956367e-04, 6.1912735e-04, 3.0956367e-04, 6.7885773e-01, -1.2282289e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 7.2742898e-01, -2.0315919e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 8.3824922e-01, -3.8645459e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 1.0465667e+00, -7.3100932e-01};
static float32_t m_lowpass_coeffs_o10[5*5] = {4.1302788e-05, 8.2605576e-05, 4.1302788e-05, 6.7676860e-01, -1.1936750e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 7.0725202e-01, -1.6978673e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 7.7352463e-01, -2.7940087e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 8.8806046e-01, -4.6884181e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 1.0752206e+00, -7.7840262e-01};
static float32_t m_lowpass_coeffs_o12[5*6] = {5.5059638e-06, 1.1011928e-05, 5.5059638e-06, 6.7563712e-01, -1.1749605e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 6.9658381e-01, -1.5214163e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 7.4096222e-01, -2.2554301e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 8.1433155e-01, -3.4689505e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 9.2671735e-01, -5.3277987e-01, 1.0000000e+00, 2.0000000e+00, 1.0000000e+00, 1.0953200e+00, -8.1164669e-01};
static arm_biquad_cascade_df2T_instance_f32 lowpass_instance[SPI_CHANNEL_NUMBER_TOTAL];
static uint8_t  spi_lowpass_filter_enabled = 1;

//encoding
#define SPI_BLE_USE_NEW_ENCODING_FLAG   1
static int32_t  spi_channel_values[SPI_CHANNEL_NUMBER_TOTAL];
static int32_t  spi_filtered_values[SPI_CHANNEL_NUMBER_TOTAL];
static int32_t  spi_encoded_values[SPI_CHANNEL_NUMBER_TOTAL] = {0};
static float32_t  spi_estimated_variance[SPI_CHANNEL_NUMBER_TOTAL] = {0x100};
static int16_t  spi_max_difval        = 511; //2**spi_max_bits_per_channel
static int16_t  spi_min_difval        = -512;
static uint32_t  spi_ble_difval_mask  = 0x03FF;
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
    uint8_t send_data[4];
    struct{uint8_t byte_0; uint8_t byte_1; uint16_t battery_status;} reg;
} static spi_config_register;
extern bool battery_read;


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
void spi_filter_data(void);
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
void filter_init(uint8_t notch, uint8_t lowpass);
void spi_init(void);


#endif  /* _ AD_SPI_H__ */
