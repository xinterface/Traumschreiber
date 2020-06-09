
#ifndef UNTITLED2_QUEUE_H
#define UNTITLED2_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#include "ad_spi.h"
#include "traumschreiber_service.h"
//#include "fastDCT32.h"


#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 64
#define BYTE 8
#define BLE_BYTES_PER_PACKET 20
#define MAX_BLE_PACKET_SIZE BLE_BYTES_PER_PACKET * BYTE
#define MAX_DCT_COEFFICIENT 63
#define CHUNK_SIZE_DCT 24
#define DCT_MAX_QUANTIZATION_LEVEL 32

#define READY_FLAG_BUFFER_SIZE 24

static uint32_t values[SPI_READ_CHANNEL_NUMBER][BUFFER_SIZE];
static uint8_t  startingPositions[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8_t  queuedElements[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8_t  counter[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static uint8_t encodedReady[READY_FLAG_BUFFER_SIZE];
static uint8_t encodedReadyPtr = 0;
static uint8_t encodedReadyNumber = 0;

//Huffman codebook
//values are ordered from -63 to 63
static int16_t huffmanCodeBook[127] =
        {
                0b010001000111, //-63
                0b01010000110,  //-62
                0b01100001101,  //-61
                0b01100001100,0b01000101000,0b01010010000,0b01010000000,
                0b01000100010,0b0111001011,0b1101110110,0b1101101011,0b0111001010,0b1101101010,0b0110010011,0b1101100100,
                0b0110010010,0b0111000011,0b0101001001,0b0110111010,0b0101010001,0b0101000110,0b0101000111,0b0100000101,
                0b0100010110,0b011111111,0b110111100,0b110110100,0b011100000,0b011111100,0b011001000,0b011001101,
                0b011000110,0b0100010011,0b0100010101,0b110110011,0b110111010,0b110111001,0b110111000,0b011011100,
                0b011010011,0b010110010,0b010110001,0b010100101,0b010110011,0b010010110,0b010000111,0b11011000,
                0b11010010,0b01101111,0b01100101,0b01100010,0b01010011,0b01000010,0b01000000,0b0111110,0b0110110,
                0b0101011,0b0100011,0b011101,0b010111,0b11001,0b1111,0b101,0b00,0b100,0b1110,0b11000,0b010011,
                0b011110,0b0100100,0b0101101,0b0110101,0b1101010,0b1101011,0b01001010,0b01010101,0b01100000,
                0b01100111,0b01110001,0b11010000,0b11011011,0b11011111,0b010000110,0b010010111,0b010100010,
                0b011000111,0b011111101,0b011001100,0b011010001,0b011100111,0b011100110,0b0101000001,0b110100111,
                0b0100010000,0b0101010000,0b010101001,0b011010010,0b011100100,0b011000010,0b011111110,0b110100011,
                0b110100110,0b0100010111,0b0100000110,0b0100000100,0b0100010010,0b110111101,0b0101000010,0b0101100000,
                0b0101100001,0b0110000111,0b0111000010,0b0110100001,0b1101000101,0b0110111011,0b1101000100,0b1101110111
                ,0b01000001111,0b01010000001,0b1101100101,0b01101000000,0b01000101001,0b01000001110,0b01010010001,
                0b01010000111, //61
                0b01101000001,//62
                0b010001000110//63
        };

static uint8_t huffmanCodeBookLength[127] =
        {
                12,11,11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,9,9,9,9,9,9,10,10,9,9,9,9,9,
                9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,7,7,7,7,6,6,5,4,3,2,3,4,5,6,6,7,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,
                10,9,10,10,9,9,9,9,9,9,9,10,10,10,10,9,10,10,10,10,10,10,10,10,10,10,11,11,10,11,11,11,11,11,11,12,
        };

static uint16_t mask[13] =
        {
            0b0,
            0b1,
            0b11,
            0b111,
            0b1111,
            0b11111,
            0b111111,
            0b1111111,
            0b11111111,
            0b111111111,
            0b1111111111,
            0b11111111111,
            0b111111111111,
        };


static uint8_t encoded[SPI_READ_CHANNEL_NUMBER][20];

void AddToQueue(uint8_t channel, uint32_t value);
void UpdateQueue();
void Encode(uint8_t channel);

int8_t Quantization(int8_t* writeTo, double* dct, uint8_t len);
bool EncodedDataAvailable();
void EncodedData(uint8_t* writeTo);
uint8_t AddCode(uint8_t* writeTo, uint8_t pos, int16_t code, uint8_t codeLength);

#endif //UNTITLED2_QUEUE_H
