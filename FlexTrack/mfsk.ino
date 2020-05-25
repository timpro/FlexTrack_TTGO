// STM32F100 and SI4032 RTTY transmitter
// released under GPL v.2 by anonymous developer
// enjoy and have a nice day
// ver 1.5a

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "HRA128_384.h"

void LoraFSKshift(int8_t shift); // bitbang carrier frequency
void interleave(unsigned char *inout, int nbytes);
void scramble(unsigned char *inout, int nbytes);
int ldpc_encode_tx_packet(unsigned char *out, unsigned char *in);


volatile char tx_on = 0;
volatile char tx_enable = 1;
char tx_on_delay = 4;
char tx_off_delay = 4;
uint8_t current_mfsk_byte = 0;
char txbuff[60]; // 16 bytes + 32 parity + 4 preamble  = 52
int packet_length = 0;

/* Short binary packet */
// 4 byte preamble for high error rates ("ESC,ESC,$,$")
// All data 8 bit Gray coded before calculating Checksum
//	- to improve soft bit prediction
struct __attribute__ ((packed)) FBinaryPacket 
{
uint8_t   PayloadID;	// Legacy list
uint8_t   Counter;	// 8 bit counter
uint16_t  Biseconds;	// Time of day / 2
uint8_t   Latitude[3];	// (int)(float * 1.0e7) / (1<<8)
uint8_t   Longitude[3];	// ( better than 10m precision )
uint16_t  Altitude;	// 0 - 65 km
uint8_t   Voltage;	// scaled 5.0v in 255 range
uint8_t   User;		// Temp / Sats
	// Temperature	6 bits MSB => (+30 to -32)
	// Satellites	2 bits LSB => 0,4,8,12 is good enough
uint16_t  Checksum;	// CRC16-CCITT Checksum.
} FSK;	// 16 data bytes, for (128,384) LDPC FEC
	// => 52 bytes at 100Hz 4fsk => 2 seconds

void start_sending() {
//	radio_enable_tx(); // carrier on
	tx_on_delay = 4;  // preamble before packet
	tx_on = 1;	   // enable timer interrupt TX
	tx_enable = 0;	   // lock radio for main thread
}

void stop_sending() {
	current_mfsk_byte = 0;  // reset next sentence to start
	tx_off_delay = 4;      // postamble before next action
	tx_on = 0;		// disable timer interrupt TX
//	radio_disable_tx();	// carrier off
}

// pack position +/- 180.xx into 24 bits
// using Ublox 32bit binary format, truncated
int32_t float_int32(float pos) {
	return (int32_t)(pos * 1.0e7f);
}

void fill_FSK() {
	int32_t position, user, sats, temp, volts;
  // static uint8_t count = 0;

	FSK.PayloadID = HORUS_4FSK_ID;
  FSK.Counter=SentenceCounter;
	FSK.Biseconds = 30 * 60 * GPS.Hours
			+ 30 * GPS.Minutes
			+ GPS.Seconds >> 1;

	position = float_int32(GPS.Longitude);
	FSK.Longitude[0] = 0xFF & (position >> 8);
	FSK.Longitude[1] = 0xFF & (position >>16);
	FSK.Longitude[2] = 0xFF & (position >>24);
	position = float_int32(GPS.Latitude);
	FSK.Latitude[0] = 0xFF & (position >> 8);
	FSK.Latitude[1] = 0xFF & (position >>16);
	FSK.Latitude[2] = 0xFF & (position >>24);
	if (GPS.Altitude > 0)
		FSK.Altitude = (uint16_t)GPS.Altitude;
	else
		FSK.Altitude = 0; // unsigned altitude

	// 1.5v to 3.5v, for RS41, 5.0V => 255
	volts = (uint16_t)(GPS.BatteryVoltage * 51.0f);
	if (volts > 255) volts = 255;
	FSK.Voltage = (uint8_t)volts;
	// 4 bits offset 5

	// Six bit temperature, +31C to -32C in 1C steps
	temp = (int16_t)GPS.InternalTemperature;
	if (temp > 31) temp = 31;
	if (temp < -32) temp = -32;
	user = (uint8_t)(temp << 2);
	// 6 bits offset 2

	// rough guide to GPS quality, (0,4,8,12 sats)
	sats = GPS.Satellites >> 2;
	if (sats < 0) sats = 0;
	if (sats > 3) sats = 3;
	user |= sats;
	// 2 bits offset 0

	FSK.User = user;
}

#if 1
int8_t get_4fsk(char c_in) {
	static uint8_t nr_bit = 0;
	int8_t _c;

	if (nr_bit > 7){
		nr_bit = 0;
		return -1; // request next input char
	} else {
		// Shift the bits we want into the MSB.
		_c = c_in << nr_bit;
		nr_bit += 2;
		return 3 & (_c >> 6); // MSB as LSB
	}
}
#else
int8_t get_2fsk(char c_in) {
	// Step through a byte, and return 2FSK symbols.
	static uint8_t nr_bit = 0;
	int8_t _c;

	if (nr_bit > 7){
		nr_bit = 0;
		return -1; // request next input char
	} else {
		// Shift the bit we want into the MSB.
		_c = c_in << nr_bit++;
		return 1 & (_c >> 7); // MSB as LSB
	}
}
#endif

// Symbol Timing Interrupt
void TIM2_IRQHandler(void) {
	static int8_t mfsk_symbol = 0;
		if ( tx_on ) {
			if ( !tx_on_delay ) {
				mfsk_symbol = get_4fsk(txbuff[current_mfsk_byte]);

				if ( mfsk_symbol == -1 ) {
					mfsk_symbol = 0;
					// Reached the end of the current character, increment the current-byte pointer.
					if ( current_mfsk_byte++ >= packet_length ) {
						stop_sending();
					} else {
						// We've now advanced to the next byte, grab the first symbol from it.
						mfsk_symbol = get_4fsk(txbuff[current_mfsk_byte]);
					}
				}
				LoraFSKshift(mfsk_symbol);
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
				mfsk_symbol = (mfsk_symbol - 1) & 0x03;
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
	timerAlarmWrite(timer, 400, true); // 10 kHz to 25 Hz
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

void arrayToGray(uint8_t *loc, uint8_t len)
{
	uint8_t i, n;
	uint8_t *ptr = loc;
	for (i = 0; i < len; i++) {
		n = *ptr;
		*ptr++ = n ^ (n>>1);
	}
}

void send_mfsk_packet() {
	fill_FSK();
	arrayToGray((uint8_t*)&FSK, sizeof(FSK) - 2);
	FSK.Checksum = (uint16_t)array_CRC16_checksum((char*)&FSK, sizeof(FSK) - 2);
	// Serial.println("Encoding 4fsk packet");
	packet_length = ldpc_encode_tx_packet((uint8_t*)txbuff, (uint8_t*)&FSK);
	// Serial.println("Sending 4fsk packet");
	start_sending();
}


/*---------------------------------------------------------------------------*\

  Adapted from Horus 4fsk code in RS41HUP, licenced as GPL2:

  FILE........: horus_l2.c
  AUTHOR......: David Rowe
  DATE CREATED: Dec 2015

  Horus telemetry layer 2 processing.  Takes an array of 8 bit payload
  data, generates parity bits for error correction, interleaves
  data and parity bits, scrambles and prepends a Unique Word.

\*---------------------------------------------------------------------------*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define INTERLEAVER
#define SCRAMBLER
#define CODEBYTES (DATABYTES + PARITYBYTES)
//  Take payload data bytes, prepend a unique word and append parity bits
int ldpc_encode_tx_packet(unsigned char *out_data, unsigned char *in_data) {
    unsigned int   i, last = 0;
    unsigned char *pout;
    const char uw_v2[] = { 0x96, 0x69, 0x69, 0x96 };

    pout = out_data;
    memcpy(pout, uw_v2, sizeof(uw_v2));
    pout += sizeof(uw_v2);
    memcpy(pout, in_data, DATA_BYTES);
    pout += DATA_BYTES;
    memset(pout, 0, PARITY_BYTES);

    // process parity bit offsets
    for (i = 0; i < NUMBERPARITYBITS; i++) {
        unsigned int shift, j;

	for(j = 0; j < MAX_ROW_WEIGHT; j++) {
		uint8_t tmp  = H_rows[i + j * NUMBERPARITYBITS];
		if (tmp) {
			tmp--;
			shift = 7 - (tmp & 7); // MSB
			last ^= in_data[tmp >> 3] >> shift;
		}
	}
	shift = 7 - (i & 7); // MSB
	pout[i >> 3] |= (last & 1) << shift;
    }

    pout = out_data + sizeof(uw_v2);
    interleave(pout, DATA_BYTES + PARITY_BYTES);
    scramble(pout, DATA_BYTES + PARITY_BYTES);

    return DATA_BYTES + PARITY_BYTES + sizeof(uw_v2);
}

// single directional for encoding
void interleave(unsigned char *inout, int nbytes)
{
    uint16_t nbits = (uint16_t)nbytes*8;
    uint32_t i, j, ibit, ibyte, ishift, jbyte, jshift;
    unsigned char out[nbytes];

    memset(out, 0, nbytes);
    for(i=0; i<nbits; i++) {
        /*  "On the Analysis and Design of Good Algebraic Interleavers", Xie et al,eq (5) */
        j = (COPRIME * i) % nbits;
        
        /* read bit i  */
        ibyte = i>>3;
        ishift = i&7;
        ibit = (inout[ibyte] >> ishift) & 0x1;

	/* write bit i  to bit j position */ 
        jbyte = j>>3;
        jshift = j&7;
        out[jbyte] |= ibit << jshift; // replace with i-th bit
    }
 
    memcpy(inout, out, nbytes);
}

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
        ibyte = i>>3;
        ishift = i&7;
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
