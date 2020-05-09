// STM32F100 and SI4032 RTTY transmitter
// released under GPL v.2 by anonymous developer
// enjoy and have a nice day
// ver 1.5a



// Note that the Golay Code is incompatible with the GPL License :
// * COPYRIGHT NOTICE: This computer program is free for non-commercial purposes.

#include <stdio.h>
#include <stdint.h>
#include <string.h>

volatile char tx_on = 0;
char tx_on_delay = 10;
char tx_off_delay = 10;
char tx_enable = 1;
char current_mfsk_byte = 0;
char txbuff[50]; // 22 bytes * 23/12 + 2 + 4 = 50
int packet_length = 0;

/* Horus binary packet */
struct __attribute__ ((packed)) FBinaryPacket 
{
uint8_t   PayloadID;
uint16_t  Counter;
uint8_t   Hours;
uint8_t   Minutes;
uint8_t   Seconds;
float   Latitude;
float   Longitude;
uint16_t  Altitude;
uint8_t   Speed; // Speed in Knots (1-255 knots)
uint8_t   Sats;
int8_t    Temp; // -64 to +64; SI4032 internal chip temp.
uint8_t   BattVoltage; // 0 = 0v, 255 = 5.0V, linear steps in-between.
uint16_t  Checksum; // CRC16-CCITT Checksum.
};
FBinaryPacket FSK;

void start_sending() {
//	radio_enable_tx(); // carrier on
	tx_on_delay = 10;  // preamble before packet
	tx_on = 1;	   // enable timer interrupt TX
	tx_enable = 0;	   // lock radio for main thread
}

void stop_sending() {
	current_mfsk_byte = 0;  // reset next sentence to start
	tx_off_delay = 10;      // postamble before next action
	tx_on = 0;		// disable timer interrupt TX
//	radio_disable_tx();	// carrier off
}

void fill_FSK() {
  FSK.PayloadID = HORUS_4FSK_ID;
  FSK.Hours = GPS.Hours;
  FSK.Minutes = GPS.Minutes;
  FSK.Seconds = GPS.Seconds;
  FSK.Longitude = GPS.Longitude;
  FSK.Latitude = GPS.Latitude;
  FSK.Altitude = (uint16_t)GPS.Altitude;
  if (GPS.Altitude < 0)
    FSK.Altitude = 0; // unsigned altitude
  FSK.Sats = GPS.Satellites;
  FSK.Speed = (uint8_t)(9 * GPS.Speed / 2500); // Knots?
  FSK.Temp = (int8_t)GPS.InternalTemperature;
  FSK.BattVoltage = (uint8_t)(255.0 * GPS.BatteryVoltage / 5.0); // Fails if Volts > 5.0
}

int get_mfsk(char _char); // extract 2 bits from TX string
void LoraFSKshift(byte shift); // bitbang carrier frequency

// Symbol Timing Interrupt
void TIM2_IRQHandler(void) {
	static int mfsk_symbol = 0;
		if ( tx_on ) {
			if ( !tx_on_delay ) {
				// 4FSK Symbol Selection Logic
				mfsk_symbol = get_mfsk(txbuff[current_mfsk_byte]);

				if ( mfsk_symbol == -1 ) {
					// Reached the end of the current character, increment the current-byte pointer.
					if ( current_mfsk_byte++ >= packet_length ) {
						stop_sending();
					} else {
						// We've now advanced to the next byte, grab the first symbol from it.
						mfsk_symbol = get_mfsk(txbuff[current_mfsk_byte]);
					}
				}
				LoraFSKshift(mfsk_symbol & 0x03);
			} else {
				// Preamble for horus_demod to lock on:
				mfsk_symbol = (mfsk_symbol + 1) & 0x03;
				LoraFSKshift(mfsk_symbol);
				tx_on_delay--;
			}
		} else {
			// tx_off_delay counts down at the interrupt rate. When it hits zero, we unlock the radio for main loop.
			if ( tx_off_delay ) {
				tx_off_delay--;
				// Postamble for horus_demod to lock on:
				mfsk_symbol = (mfsk_symbol + 1) & 0x03;
				LoraFSKshift(mfsk_symbol);
			} else if ( !tx_enable )
				tx_enable = 1;
		}
}

// ESP32 specific timer interrupt setup
hw_timer_t * timer = NULL;
void initTimer() {
	timer = timerBegin(0, 8000, true); // 80Mhz to 10 kHz
	timerAttachInterrupt(timer, &TIM2_IRQHandler, true);
	timerAlarmWrite(timer, 100, true); // 10 kHz to 100 Hz
	timerAlarmEnable(timer);
}

// Init settings for timer loop
void setupFSK() {
	tx_on = 0;	// not transmitting
	tx_enable = 1;  // available to TX

	initTimer();	// start interrupts
}

// is sending ?
int checkFSK() {
	return tx_enable;
}

// UKHAS checksum calculator
uint16_t array_CRC16_checksum(char *string, int len) {
  uint16_t crc = 0xffff;
  char i;
  int ptr = 0;
  while (ptr < len) {
    ptr++;
    crc = crc ^ (*(string++) << 8);
    for (i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (uint16_t) ((crc << 1) ^ 0x1021);
      else
        crc <<= 1;
    }
  }
  return crc;
}

int horus_l2_encode_tx_packet(unsigned char *out, unsigned char *in, int num);
void send_mfsk_packet() {
	fill_FSK();
	FSK.Checksum = (uint16_t)array_CRC16_checksum( (char*)&FSK,sizeof(FBinaryPacket) - 2);

	memset(txbuff, 0x1b, 4);  // preamble, 32bits (unnecessary)
	int coded_len = horus_l2_encode_tx_packet( (uint8_t*)txbuff + 4, (uint8_t*)&FSK, sizeof(FBinaryPacket) );

	packet_length = coded_len + 4; // packet + preamble
	start_sending();
}

int get_mfsk(char current_char) {
	// Step through a byte, and return 4FSK symbols.
	static uint8_t nr_nibble = 0;
	char _c = current_char;

	if (nr_nibble == 4){
		// Reached the end of the byte.
		nr_nibble = 0;
		// Return -1 to indicate we have finished with this byte.
		return -1;
	} else {
		// Shift it left to the nibble we are up to.
		for (int _i=0;_i<nr_nibble;_i++){
			_c = _c << 2;
		}
		// Get the current symbol (the 2-bits we need).
		uint8_t symbol = ((uint8_t)_c & 0xC0) >> 6;

		nr_nibble++;
		return (int)symbol;
	}
}


/*---------------------------------------------------------------------------*\

  FILE........: horus_l2.c
  AUTHOR......: David Rowe
  DATE CREATED: Dec 2015

  Horus telemetry layer 2 processing.  Takes an array of 8 bit payload
  data, generates parity bits for a (23,12) Golay code, interleaves
  data and parity bits, pre-pends a Unique Word for modem sync.
  Caller is responsible for providing storage for output packet.

\*---------------------------------------------------------------------------*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define RUN_TIME_TABLES
#define INTERLEAVER
#define SCRAMBLER

static char uw[] = {'$','$'};

/* Function Prototypes ------------------------------------------------*/

int32_t get_syndrome(int32_t pattern);
unsigned short gen_crc16(unsigned char* data_p, unsigned char length);
void interleave(unsigned char *inout, int nbytes, int dir);
void scramble(unsigned char *inout, int nbytes);

/* Functions ----------------------------------------------------------*/

/*
   We are using a Golay (23,12) code which has a codeword 23 bits
   long.  The tx packet format is:

      | Unique Word | payload data bits | parity bits |

   This function works out how much storage the caller of
   horus_l2_encode_tx_packet() will need to store the tx packet
 */

int horus_l2_get_num_tx_data_bytes(int num_payload_data_bytes) {
    int num_payload_data_bits, num_golay_codewords;
    int num_tx_data_bits, num_tx_data_bytes;
    
    num_payload_data_bits = num_payload_data_bytes*8;
    num_golay_codewords = num_payload_data_bits/12;
    if (num_payload_data_bits % 12) /* round up to 12 bits, may mean some unused bits */
        num_golay_codewords++;

    num_tx_data_bits = sizeof(uw)*8 + num_payload_data_bits + num_golay_codewords*11;
    num_tx_data_bytes = num_tx_data_bits/8;
    if (num_tx_data_bits % 8) /* round up to nearest byte, may mean some unused bits */
        num_tx_data_bytes++;
    
    return num_tx_data_bytes;
}


/*
  Takes an array of payload data bytes, prepends a unique word and appends
  parity bits.

  The encoder will run on the payload on a small 8-bit uC.  As we are
  memory constrained so we do a lot of burrowing for bits out of
  packed arrays, and don't use a LUT for Golay encoding.  Hopefully it
  will run fast enough.  This was quite difficult to get going,
  suspect there is a better way to write this.  Oh well, have to start
  somewhere.
 */

int horus_l2_encode_tx_packet(unsigned char *output_tx_data,
                              unsigned char *input_payload_data,
                              int            num_payload_data_bytes)
{
    int            num_tx_data_bytes, num_payload_data_bits;
    unsigned char *pout = output_tx_data;
    int            ninbit, ningolay, nparitybits;
    int32_t        ingolay, paritybyte, inbit, golayparity;
    int            ninbyte, shift, golayparitybit, i;

    num_tx_data_bytes = horus_l2_get_num_tx_data_bytes(num_payload_data_bytes);
    memcpy(pout, uw, sizeof(uw)); pout += sizeof(uw);
    memcpy(pout, input_payload_data, num_payload_data_bytes); pout += num_payload_data_bytes;

    /* Read input bits one at a time.  Fill input Golay codeword.  Find output Golay codeword.
       Write this to parity bits.  Write parity bytes when we have 8 parity bits.  Bits are
       written MSB first. */

    num_payload_data_bits = num_payload_data_bytes*8;
    ninbit = 0;
    ingolay = 0;
    ningolay = 0;
    paritybyte = 0;
    nparitybits = 0;

    while (ninbit < num_payload_data_bits) {

        /* extract input data bit */
        ninbyte = ninbit/8;
        shift = 7 - (ninbit % 8);
        inbit = (input_payload_data[ninbyte] >> shift) & 0x1;
        ninbit++;

        /* build up input golay codeword */
        ingolay = ingolay | inbit;
        ningolay++;

        /* when we get 12 bits do a Golay encode */
        if (ningolay % 12) {
            ingolay <<= 1;
        }
        else {
            golayparity = get_syndrome(ingolay<<11);
            ingolay = 0;

            /* write parity bits to output data */
            for (i=0; i<11; i++) {
                golayparitybit = (golayparity >> (10-i)) & 0x1;
                paritybyte = paritybyte | golayparitybit;
                nparitybits++;
                if (nparitybits % 8) {
                   paritybyte <<= 1;
                }
                else {
                    /* OK we have a full byte ready */
                    *pout = paritybyte;
                    pout++;
                    paritybyte = 0;
                }
            }
        }
    } /* while(.... */


    /* Complete final Golay encode, we may have partially finished ingolay, paritybyte */
    if (ningolay % 12) {
        ingolay >>= 1;
        golayparity = get_syndrome(ingolay<<12);

        /* write parity bits to output data */
        for (i=0; i<11; i++) {
            golayparitybit = (golayparity >> (10 - i)) & 0x1;
            paritybyte = paritybyte | golayparitybit;
            nparitybits++;
            if (nparitybits % 8) {
                paritybyte <<= 1;
            }
            else {
                /* OK we have a full byte ready */
                *pout++ = (unsigned char)paritybyte;
                paritybyte = 0;
            }
        }
    }
 
    /* and final, partially complete, parity byte */
    if (nparitybits % 8) {
        paritybyte <<= 7 - (nparitybits % 8);  // use MS bits first
        *pout++ = (unsigned char)paritybyte;
    }
    assert(pout == (output_tx_data + num_tx_data_bytes));

    /* optional interleaver - we dont interleave UW */
    #ifdef INTERLEAVER
    interleave(&output_tx_data[sizeof(uw)], num_tx_data_bytes-2, 0);
    #endif

    /* optional scrambler to prevent long strings of the same symbol
       which upsets the modem - we dont scramble UW */

    #ifdef SCRAMBLER
    scramble(&output_tx_data[sizeof(uw)], num_tx_data_bytes-2);
    #endif

    return num_tx_data_bytes;
}

#ifdef INTERLEAVER
void interleave(unsigned char *inout, int nbytes, int dir)
{
    uint16_t nbits = (uint16_t)nbytes*8;
    uint32_t i, j, n, ibit, ibyte, ishift, jbyte, jshift;
    uint32_t b;
    unsigned char out[nbytes];


    memset(out, 0, nbytes);
           
    b = 337; // prime number chosen for THIS configuration

    for(n=0; n<nbits; n++) {
        /*  "On the Analysis and Design of Good Algebraic Interleavers", Xie et al,eq (5) */
        i = n;
        j = (b*i) % nbits;
        
        if (dir) {
            uint16_t tmp = j;
            j = i;
            i = tmp;
        }

        /* read bit i and write to bit j postion */
        ibyte = i/8;
        ishift = i%8;
        ibit = (inout[ibyte] >> ishift) & 0x1;

        jbyte = j/8;
        jshift = j%8;

        /* write jbit to ibit position */
        out[jbyte] |= ibit << jshift; // replace with i-th bit
        //out[ibyte] |= ibit << ishift; // replace with i-th bit
    }
 
    memcpy(inout, out, nbytes);
}
#endif


#ifdef SCRAMBLER
/* 16 bit DVB additive scrambler as per Wikpedia example */
void scramble(unsigned char *inout, int nbytes)
{
    int nbits = nbytes*8;
    int i, ibit, ibits, ibyte, ishift, mask;
    uint16_t scrambler = 0x4a80;  /* init additive scrambler at start of every frame */
    uint16_t scrambler_out;

    /* in place modification of each bit */
    for(i=0; i<nbits; i++) {

        scrambler_out = ((scrambler & 0x2) >> 1) ^ (scrambler & 0x1);

        /* modify i-th bit by xor-ing with scrambler output sequence */
        ibyte = i/8;
        ishift = i%8;
        ibit = (inout[ibyte] >> ishift) & 0x1;
        ibits = ibit ^ scrambler_out;                  // xor ibit with scrambler output

        mask = 1 << ishift;
        inout[ibyte] &= ~mask;                  // clear i-th bit
        inout[ibyte] |= ibits << ishift;         // set to scrambled value

        /* update scrambler */
        scrambler >>= 1;
        scrambler |= scrambler_out << 14;
    }
}
#endif

/*---------------------------------------------------------------------------*\

                                   GOLAY FUNCTIONS

\*---------------------------------------------------------------------------*/

/* File:    golay23.c
 * Title:   Encoder/decoder for a binary (23,12,7) Golay code
 * Author:  Robert Morelos-Zaragoza (robert@spectra.eng.hawaii.edu)
 * Date:    August 1994
 *
 * The binary (23,12,7) Golay code is an example of a perfect code, that is,
 * the number of syndromes equals the number of correctable error patterns.
 * The minimum distance is 7, so all error patterns of Hamming weight up to
 * 3 can be corrected. The total number of these error patterns is:
 *
 *       Number of errors         Number of patterns
 *       ----------------         ------------------
 *              0                         1
 *              1                        23
 *              2                       253
 *              3                      1771
 *                                     ----
 *    Total number of error patterns = 2048 = 2^{11} = number of syndromes
 *                                               --
 *                number of redundant bits -------^
 *
 * Because of its relatively low length (23), dimension (12) and number of
 * redundant bits (11), the binary (23,12,7) Golay code can be encoded and
 * decoded simply by using look-up tables. The program below uses a 16K
 * encoding table and an 8K decoding table.
 *
 * For more information, suggestions, or other ideas on implementing error
 * correcting codes, please contact me at (I'm temporarily in Japan, but
 * below is my U.S. address):
 *
 *                    Robert Morelos-Zaragoza
 *                    770 S. Post Oak Ln. #200
 *                      Houston, Texas 77056
 *
 *             email: robert@spectra.eng.hawaii.edu
 *
 *       Homework: Add an overall parity-check bit to get the (24,12,8)
 *                 extended Golay code.
 *
 * COPYRIGHT NOTICE: This computer program is free for non-commercial purposes.
 * You may implement this program for any non-commercial application. You may
 * also implement this program for commercial purposes, provided that you
 * obtain my written permission. Any modification of this program is covered
 * by this copyright.
 *
 * ==   Copyright (c) 1994  Robert Morelos-Zaragoza. All rights reserved.   ==
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#define X22             0x00400000   /* vector representation of X^{22} */
#define X11             0x00000800   /* vector representation of X^{11} */
#define MASK12          0xfffff800   /* auxiliary vector for testing */
#define GENPOL          0x00000c75   /* generator polinomial, g(x) */

/* Global variables:
 *
 * pattern = error pattern, or information, or received vector
 * encoding_table[] = encoding table
 * decoding_table[] = decoding table
 * data = information bits, i(x)
 * codeword = code bits = x^{11}i(x) + (x^{11}i(x) mod g(x))
 * numerr = number of errors = Hamming weight of error polynomial e(x)
 * position[] = error positions in the vector representation of e(x)
 * recd = representation of corrupted received polynomial r(x) = c(x) + e(x)
 * decerror = number of decoding errors
 * a[] = auxiliary array to generate correctable error patterns
 */

int32_t get_syndrome(int32_t pattern)
/*
 * Compute the syndrome corresponding to the given pattern, i.e., the
 * remainder after dividing the pattern (when considering it as the vector
 * representation of a polynomial) by the generator polynomial, GENPOL.
 * In the program this pattern has several meanings: (1) pattern = infomation
 * bits, when constructing the encoding table; (2) pattern = error pattern,
 * when constructing the decoding table; and (3) pattern = received vector, to
 * obtain its syndrome in decoding.
 */
{
    int32_t aux = X22;

    if (pattern >= X11)
       while (pattern & MASK12) {
           while (!(aux & pattern))
              aux = aux >> 1;
           pattern ^= (aux/X11) * GENPOL;
           }
    return(pattern);
}
