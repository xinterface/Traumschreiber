#include "Queue.h"
#include "fast-dct-fft.h"


/***
 * Add value to the queue
 * @param channel
 * @param value
 */
void AddToQueue(uint8_t channel, uint32_t value){

    values[channel][(startingPositions[channel] + queuedElements[channel]) % BUFFER_SIZE] = value;
    queuedElements[channel] ++;

}

/**
 * Check if enough elements are queued to compress them
 */
void UpdateQueue(){

    uint8_t channel;


    for(channel = 0; channel < SPI_READ_CHANNEL_NUMBER; channel++){

        //The channel offset makes sure, that only one channel gets encoded each cycle
        if(queuedElements[channel] >= CHUNK_SIZE_DCT + channel){

            Encode(channel);

            counter[channel] ++;
            if(counter[channel] > 16)
                counter[channel] = 0;

            //Update Queue pointer
            queuedElements[channel] = queuedElements[channel] - CHUNK_SIZE_DCT;
            startingPositions[channel] = (startingPositions[channel] + CHUNK_SIZE_DCT) % BUFFER_SIZE;

            //Update Ready Queue
            encodedReady[(encodedReadyPtr + encodedReadyNumber) % READY_FLAG_BUFFER_SIZE] = channel;
            encodedReadyNumber ++;

        }
    }
}



void binprintf(uint8_t v)
{
    unsigned int mask=1<<((sizeof(uint8_t)<<3)-1);
    while(mask) {
        printf("%d", (v&mask ? 1 : 0));
        mask >>= 1;
    }
}


bool EncodedDataAvailable(){
    return encodedReadyNumber > 0;
}

void EncodedData(uint8_t* writeTo){

    if(encodedReadyNumber < 1)
        return;

    memcpy(writeTo, encoded[encodedReady[(encodedReadyPtr) % READY_FLAG_BUFFER_SIZE]], 20);

    encodedReadyNumber --;
    encodedReadyPtr ++;

    if(encodedReadyPtr >= READY_FLAG_BUFFER_SIZE)
        encodedReadyPtr = 0;
    return;

}


/***
 * Encodes values of one channel
 *
 * 5 Bit Channel Number
 * 4 Bit Counter
 * 24 Bit mean
 * 6 Bit Quantization Level
 *
 * 7 Bit DCT Coefficient 2
 * 7 Bit DCT Coefficient 3
 * 7 Bit DCT Coefficient 4
 * 7 Bit DCT Coefficient 5
 * 7 Bit DCT Coefficient 6
 * 7 Bit DCT Coefficient 7
 * 7 Bit DCT Coefficient 8
 * 7 Bit DCT Coefficient 9
 * 7 Bit DCT Coefficient 10
 *
 * The following bits are Huffman encoded and variable length
 *
 * Note: the DCT Coefficient 1 is always 0
 *
 * @param channel
 */
void Encode(uint8_t channel){

    int i;
    for(i = 0; i < 20; i++){
        encoded[channel][i] = 0;
    }

    //calculate mean
    double sum = values[channel][(startingPositions[channel]) % BUFFER_SIZE];
    for(i = 1; i < CHUNK_SIZE_DCT; i++){
        sum += values[channel][(startingPositions[channel] + i) % BUFFER_SIZE];
    }

    uint32_t mean = round(sum / CHUNK_SIZE_DCT);

    //center values
    double centeredValues[CHUNK_SIZE_DCT];
    for(i = 0; i < CHUNK_SIZE_DCT; i++){
        centeredValues[i] = (double)values[channel][(startingPositions[channel] + i) % BUFFER_SIZE] - mean;
    }

    if(FastDctFft_transform(centeredValues, CHUNK_SIZE_DCT)){
        //TODO if FastDctFft_transform fails
    }


    int8_t quantizedDCT[CHUNK_SIZE_DCT];
    uint8_t quantizationLevel = Quantization(quantizedDCT, centeredValues, CHUNK_SIZE_DCT);

    encoded[channel][0] = channel << 3;
    encoded[channel][0] |= counter[channel] >> 1;
    encoded[channel][1] = counter[channel] << 7;

    size_t pos = 9;

    pos = AddCode(encoded[channel], pos, mean >> 16,8);
    pos = AddCode(encoded[channel], pos, mean >> 8,8);
    pos = AddCode(encoded[channel], pos, mean,8);
    pos = AddCode(encoded[channel], pos, quantizationLevel, 6);

    //Skipping the first DCT Coefficient since it is always Zero due to the centering
    for(i = 1; i < 10; i++){
        pos = AddCode(encoded[channel], pos, quantizedDCT[i], 7);
    }
    for(i = 10; i < CHUNK_SIZE_DCT; i++){
        pos = AddCode(encoded[channel], pos, huffmanCodeBook[quantizedDCT[i] + 63], huffmanCodeBookLength[quantizedDCT[i] + 63]);
        if(pos > MAX_BLE_PACKET_SIZE){
            return;
        }
    }
}

/***
 * Quantizizes the values to fit in 7 Bit
 * @param writeTo
 * @param dct
 * @param len
 * @return
 */
int8_t Quantization(int8_t* writeTo, double* dct, uint8_t len){

    //find maximum dct coefficient to determine quantization level
    double max = 0;
    size_t i;
    for(i = 0; i < len; i++){
        double tmp = fabs(dct[i]);
        if(tmp > max)
            max = tmp;
    }

    int8_t quantizationLevel = 0;

    if(max > MAX_DCT_COEFFICIENT ){
        quantizationLevel = ceil(log2(ceil(1/(MAX_DCT_COEFFICIENT/max))));
        double scale = 1/pow(2,quantizationLevel);
        for(i = 0; i < CHUNK_SIZE_DCT; i++){
            writeTo[i] = round(dct[i] * scale);
        }
    }

    else if(max != 0){
        quantizationLevel = floor(log2(floor(MAX_DCT_COEFFICIENT / max)));
        double scale = pow(2,quantizationLevel);

        if(quantizationLevel > DCT_MAX_QUANTIZATION_LEVEL)
            quantizationLevel = DCT_MAX_QUANTIZATION_LEVEL;

        for(i = 0; i < CHUNK_SIZE_DCT; i++){
            writeTo[i] = round(dct[i] * scale);
        }
        quantizationLevel = -quantizationLevel;
    }

    if(quantizationLevel > MAX_DCT_COEFFICIENT - 1){
        quantizationLevel = MAX_DCT_COEFFICIENT - 1;
    }
    if(quantizationLevel < -MAX_DCT_COEFFICIENT){
        quantizationLevel = -MAX_DCT_COEFFICIENT;
    }
    return quantizationLevel;
}

uint8_t AddCode(uint8_t* writeTo, uint8_t pos, int16_t code, uint8_t codeLength){

    uint8_t index = pos / BYTE;
    uint8_t start = pos % BYTE;
    uint8_t bitsLeft = BYTE - start;
    if(pos + codeLength >= MAX_BLE_PACKET_SIZE)
        return UINT8_MAX;
    code &= mask[codeLength];
    if(codeLength > bitsLeft){

        writeTo[index] |= code >> (codeLength - bitsLeft);
        if(codeLength - bitsLeft > 8){
            writeTo[index + 1] = code >>    (codeLength - bitsLeft - BYTE);
            writeTo[index + 2] = code <<    (BYTE + BYTE - codeLength + bitsLeft);
        }
        else{
            writeTo[index + 1] |= code << (BYTE - codeLength + bitsLeft);
        }

    }
    else {
        writeTo[index] |= code << (bitsLeft - codeLength);
    }
    return pos + codeLength;
}

