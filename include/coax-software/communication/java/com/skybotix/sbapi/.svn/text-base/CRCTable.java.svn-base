package com.skybotix.sbapi;

public class CRCTable {
    private final static byte WIDTH = 8;
    private final static byte TOPBIT = (byte)(1 << (WIDTH-1));
    private final static byte POLYNOMIAL  = (byte)0xD8;
    private byte[] crcTable = new byte[256];

    CRCTable() {
        short dividend,bit;
        byte  remainder;


        /*
         * Compute the remainder of each possible dividend.
         */
        for (dividend = 0; dividend < 256; ++dividend) {
            /*
             * Start with the dividend followed by zeros.
             */
            remainder = (byte)(dividend << (WIDTH - 8));

            /*
             * Perform modulo-2 division, a bit at a time.
             */
            for (bit = 8; bit > 0; --bit) {
                /*
                 * Try to divide the current data bit.
                 */			
                if ((remainder & TOPBIT) != 0) {
                    remainder = (byte)((remainder << 1) ^ POLYNOMIAL);
                } else {
                    remainder = (byte)((remainder << 1));
                }
            }

            /*
             * Store the result into the table.
             */
            crcTable[dividend] = remainder;
        }
    }

    byte compute(byte[] message, int nBytes) {
        byte data,value;
        short remainder = 0;

        /*
         * Divide the message by the polynomial, a byte at a time.
         */
        for (value = 0; value < nBytes; ++value)
        {
            data = (byte)(message[value] ^ (remainder >> (WIDTH - 8)));
            remainder = (short)(crcTable[data] ^ (remainder << 8));
        }

        /*
         * The final remainder is the CRC.
         */
        return (byte)(remainder);

    }   /* crcFast() */

}

