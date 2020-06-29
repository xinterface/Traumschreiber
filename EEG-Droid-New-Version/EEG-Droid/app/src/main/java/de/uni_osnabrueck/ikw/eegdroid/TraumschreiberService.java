package de.uni_osnabrueck.ikw.eegdroid;


import android.bluetooth.BluetoothGattCharacteristic;
import android.util.Log;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.UUID;
import java.lang.Math;

/**
 * Created by jan on 01.01.18.
 */



public class TraumschreiberService {

    // original code
    // public final static String VENDOR_PREFIX = "74:72:61:75:6D:";
    // development-board
    // public final static String VENDOR_PREFIX = "EB:07:88:DF:29:";
    // Traumschreiber
    // public final static String VENDOR_PREFIX = "F3:C7:2B:4C:A1:";
    public final static String VENDOR_PREFIX = "F3:";
    // Names chosen according to the python tflow_edge Traumschreiber.py
    public final static UUID BIOSIGNALS_UUID = UUID.fromString("faa7b588-19e5-f590-0545-c99f193c5c3e");
    public final static UUID LEDS_UUID = UUID.fromString("fcbea85a-4d87-18a2-2141-0d8d2437c0a4");

    //public final static UUID UUID_HEART_RATE_MEASUREMENT =
    //       UUID.fromString(SampleGattAttributes.HEART_RATE_MEASUREMENT);

    public static boolean isTraumschreiberAddress(String bluetoothDeviceAddress) {
        return bluetoothDeviceAddress.startsWith(VENDOR_PREFIX);
    }

    String mTraumschreiberDeviceAddress;

    public TraumschreiberService(String traumschreiberDeviceAddress) {
        this.mTraumschreiberDeviceAddress = traumschreiberDeviceAddress;
    }

//    /***
//     * decompress takes a bytearray data_bytes and converts it to integers, according to the way the Traumschreiber transmits the data via bluetooth
//     * @param data_bytes
//     * @return int[] data_ints of the datapoint values as integers
//     */
//    public static int[] decompress(byte[] data_bytes) {
//        int bytelengthDatapoint = 2;
//        int[] data_ints = new int[data_bytes.length / bytelengthDatapoint];
//        Log.d("Decompressing", "decompress: "+String.format("%02X %02X ", data_bytes[0], data_bytes[1]));
//
//
//        //https://stackoverflow.com/questions/9581530/converting-from-byte-to-int-in-java
//        //Example: rno[0]&0x000000ff)<<24|(rno[1]&0x000000ff)<<16|
//        for(int i = 0; i < data_bytes.length /bytelengthDatapoint; i++) {
//            int new_int = (data_bytes[i*bytelengthDatapoint + 1]) << 8 | (data_bytes[i*bytelengthDatapoint + 0])&0xff;
//            //new_int = new_int << 8;
//            data_ints[i] = new_int;
//        }
//
//        return data_ints;
//    }

    public static int[] decompress(byte[] data_bytes) {
//        int packet_id = data_bytes[0] >> 4;
        int[] data_ints = new int[6];
        for (int channel = 0; channel < 6; channel++) {
            if(data_bytes.length >= (channel + 1) * 3) {
                data_ints[channel] = data_bytes[channel * 3] << 16 | data_bytes[channel * 3 + 1] << 8 | data_bytes[channel * 3 + 2];
            }
        }
        return data_ints;
    }

    private static int[] decode(String data) {
        int channel = Integer.parseInt(data.substring(0,4), 2);
        int counter = Integer.parseInt(data.substring(5,9), 2);
        int mean = Integer.parseInt(data.substring(10,33), 2);
        int quantizationLevel = Integer.parseInt(data.substring(34,39), 2);
        int[] dctEncoded = new int[24];
        for (int i = 0; i < 9; i++) {
            dctEncoded[i] = Integer.parseInt(data.substring(34 + (i * 7), 40 + (i * 7)), 2);
        }
        String huffmann = data.substring(106, data.length());
        int[] huffmannArray = huffmannDecoding(huffmann);
        for (int i = 0; i < huffmannArray.length; i++) {
            if(9+i < 24)
                dctEncoded[9+i] = huffmannArray[i];
        }
        int[] dctDecoded = iDCT(dctEncoded, quantizationLevel);
        int quantization = 1;
        for (int i = 0; i < quantizationLevel; i++) {
            quantization *= 2;
        }
        for (int i = 0; i < dctDecoded.length; i++) {
            dctDecoded[i] *= quantization;
            dctDecoded[i] += mean;
        }
        return dctDecoded;
    }

    private static int[] iDCT(int[] y, int l) {
        int[] x = new int[24];
        for(int i=0; i<24; i++) {
            int ergebnis = 0;
            int n = i+1;
            for (int k = 1; k <= 24; k++) {
                int zwischenergebnis = y[k-1];
                int sigma = 0;
                if(k == l) sigma = 1;
                zwischenergebnis *= (1/Math.sqrt(1+sigma));
                zwischenergebnis *= Math.cos((Math.PI/48)*(k-1)*(2*n-1));
                ergebnis += zwischenergebnis;
            }
            ergebnis = (int)(((double) ergebnis) * Math.sqrt(1./12.));
            x[n-1] = ergebnis;
        }
        return x;
    }

    private static int[] huffmannDecoding(String encoded) {
        int[] huffmanCodeBook=
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
        int[] mask =
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
            0b111111111111
        };
        LinkedList<Integer> result = new LinkedList<>();
        int lowerIndex = 0;
        int upperIndex = 1;
        while (upperIndex < encoded.length()) {
            for (int i = 0; i < huffmanCodeBook.length; i++) {
                if(Integer.parseInt(encoded.substring(lowerIndex, upperIndex), 2) == huffmanCodeBook[i]) {
                    result.add(huffmanCodeBook[i]);
                    lowerIndex = upperIndex;
                    upperIndex++;
                }
            }
            upperIndex++;
        }
        int[] result_array = new int[result.size()];
        for (int i = 0; i < result_array.length; i++) {
            result_array[i] = result.pop();
        }
        return result_array;
    }

}

