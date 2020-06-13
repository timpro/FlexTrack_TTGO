/*---------------------------------------------------*\
|                                                     |
| LoRa radio code, for downlink, uplink and repeating |
|                                                     |
| Messages can be timed using a GPS reference, to     |
| comply with the TDMA timing requirements.           |
|                                                     |
| Connections:                                        |
|                                                     |
|               Arduino  X - RFM98W DIO5              |
|               Arduino  X - RFM98W DIO0              |
|                                                     |
|               Arduino  X  - RFM98W NSS              |
|               Arduino 11 - RFM98W MOSI              |
|               Arduino 12 - RFM98W MISO              |
|               Arduino 13 - RFM98W CLK               |
|                                                     |
\*---------------------------------------------------*/

#ifdef LORA_NSS

#include <SPI.h>
#include <string.h>

// RFM98 registers
#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
//#define REG_PREAMBLE_MSB          0x18
//#define REG_PREAMBLE_LSB          0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31

#define REG_DETECTION_THRESHOLD     0x37
#define REG_TEMP                    0x3C
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41

// FSK stuff
#define REG_PA_RAMP                 0x0A
#define REG_PREAMBLE_FSK            0x26
#define REG_SYNC_CONF               0x27
#define REG_SYNC1                   0x28
#define REG_PACKET_CONFIG1          0x30
#define REG_PACKET_CONFIG2          0x31
#define REG_PAYLOAD_LENGTH_FSK      0x32
#define REG_FIFO_THRESH             0x35
#define REG_FDEV_MSB                0x04
#define REG_FDEV_LSB                0x05
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_BITRATE_MSB             0x02
#define REG_BITRATE_LSB             0x03
#define REG_IRQ_FLAGS2              0x3F
#define REG_PLL_HOP                 0x44

// MODES
#define RF98_MODE_RX_CONTINUOUS     0x85
#define RF98_MODE_TX                0x83
#define RF98_MODE_SLEEP             0x80
#define RF98_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              255

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2
#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04

// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F    // 50mW (chip max)
#define PA_LOW_BOOST                0x81    //  2mW
#define PA_MED_BOOST                0x8B    // 20mW
#define PA_MAX_UK                   0x88    // 10mW (UK max 434)
#define PA_HALF_UK                  0x85    //  5mW (UK max 868)
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// 20 dBm support NOT implemented
#define REG_PA_DAC                  0x4D
#define PA_DAC_20                   0x87

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00

typedef enum {lmIdle, lmListening, lmSending} tLoRaMode;

tLoRaMode LoRaMode;
byte currentMode = 0x81;
int TargetID;
int ImplicitOrExplicit;
int GroundCount;
int AirCount;
int BadCRCCount;
unsigned char Sentence[SENTENCE_LENGTH];
unsigned long LastLoRaTX=0;
int FSKCount=0;
int InFSKMode=0;
int SendingFSK=0;

void writeRegister(byte addr, byte value);
void select();
void unselect();

void SetupLoRa(void)
{
  setupRFM98(LORA_FREQUENCY, LORA_MODE);
  // setupFSK();
}

void setupRFM98(float Frequency, int Mode)
{
  int ErrorCoding;
  int Bandwidth;
  int SpreadingFactor;
  int LowDataRateOptimize;
  int PayloadLength;
  
  // initialize the pins
  #ifdef LORA_RESET
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(10);          // Module needs this before it's ready
  #endif
  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_DIO0, INPUT);

  // SPI.begin();
  SPI.begin(SCK,MISO,MOSI,LORA_NSS);
  
  // LoRa mode 
  setLoRaMode();

  // Frequency
  setFrequency(Frequency + LORA_OFFSET / 1000.0);

  // LoRa settings for various modes.  We support modes 2 (repeater mode), 1 (normally used for SSDV) and 0 (normal slow telemetry mode).
  
  if (Mode == 5)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_41K7;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0;
  }
  else if (Mode == 2)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_62K5;
    SpreadingFactor = SPREADING_8;
    LowDataRateOptimize = 0;		
  }
  else if (Mode == 1)
  {
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_6;
    LowDataRateOptimize = 0;    
  }
  else // if (Mode == 0)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0x08;		
  }
  
  PayloadLength = ImplicitOrExplicit == IMPLICIT_MODE ? 255 : 0;

  writeRegister(REG_MODEM_CONFIG, ImplicitOrExplicit | ErrorCoding | Bandwidth);
  writeRegister(REG_MODEM_CONFIG2, SpreadingFactor | CRC_ON);
  writeRegister(REG_MODEM_CONFIG3, 0x04 | LowDataRateOptimize);									// 0x04: AGC sets LNA gain
  
  // writeRegister(REG_DETECT_OPT, (SpreadingFactor == SPREADING_6) ? 0x05 : 0x03);					// 0x05 For SF6; 0x03 otherwise
  writeRegister(REG_DETECT_OPT, (readRegister(REG_DETECT_OPT) & 0xF8) | ((SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));  // 0x05 For SF6; 0x03 otherwise
  
  writeRegister(REG_DETECTION_THRESHOLD, (SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);		// 0x0C for SF6, 0x0A otherwise  
  
  writeRegister(REG_PAYLOAD_LENGTH, PayloadLength);
  writeRegister(REG_RX_NB_BYTES, PayloadLength);
  
  // Change the DIO mapping to 01 so we can listen for TxDone on the interrupt
  writeRegister(REG_DIO_MAPPING_1,0x40);
  writeRegister(REG_DIO_MAPPING_2,0x00);
  
  // Go to standby mode
  setMode(RF98_MODE_STANDBY);
  
  Serial.println("Setup Complete");
}

// expect tx worse than  1 ppm, so float is good enough
void setFrequency(float Frequency)
{
  unsigned long FrequencyValue;
    
  Serial.print("Frequency is ");
  Serial.println(Frequency);

  Frequency = Frequency * 16384.0f;
  FrequencyValue = (unsigned long)(Frequency);

  Serial.print("FrequencyValue is ");
  Serial.println(FrequencyValue);

  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);    // Set frequency
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
}

void setLoRaMode()
{
  Serial.println("Setting LoRa Mode");
  setMode(RF98_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);
}

/////////////////////////////////////
//    Method:   Change the mode
//////////////////////////////////////
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
//  Serial.printf("Set LoRa Mode %d\n", newMode);
  
  switch (newMode) 
  {
    case RF98_MODE_TX:
      writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
      writeRegister(REG_PA_CONFIG, POWERLEVEL);
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_SLEEP:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_STANDBY:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RF98_MODE_SLEEP)
  {
    delay(10);
  }
}


/////////////////////////////////////
//    Method:   Read Register
//////////////////////////////////////

byte readRegister(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();

//  printf ("Reg %d = %02X\n", addr, regval);
  return regval;
}

/////////////////////////////////////
//    Method:   Write Register
//////////////////////////////////////

void writeRegister(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(LORA_NSS, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(LORA_NSS, HIGH);
}

int FSKPacketSent(void)
{
  return ((readRegister(REG_IRQ_FLAGS2) & 0x48) != 0);
}


int LoRaIsFree(void)
{
  if (SendingFSK && FSKPacketSent()) {
    SendingFSK = 0;
    LoRaMode = lmIdle;
  }

  if ((LoRaMode != lmSending) || digitalRead(LORA_DIO0))
  {
    if (LoRaMode == lmSending)
    {
      // Clear that IRQ flag
      writeRegister( REG_IRQ_FLAGS, 0x08); 
      LoRaMode = lmIdle;
    }
    return 1;
  }
  
  return 0;
}


// TODO: fix this!
// read lora chip temperature
int lora_read_temperature() {
  byte temp;
  // Read ADC Value. Internal Temp (degC) = 15 - ADC Value
  temp = readRegister(REG_TEMP);

  return 15 - (signed char)(temp);
}

void SendLoRaPacket(unsigned char *buffer, int Length)
{
  int i;
  
  LastLoRaTX = millis();

  Serial.print("Sending "); Serial.print(Length);Serial.println(" bytes");

  setLoRaMode();
  setMode(RF98_MODE_STANDBY);

  writeRegister(REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 maps DIO0 to TxDone
  writeRegister(REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
  writeRegister(REG_FIFO_ADDR_PTR, 0x00); 
  if (ImplicitOrExplicit == EXPLICIT_MODE)
  {
    writeRegister(REG_PAYLOAD_LENGTH, Length);
  }
  select();
  // tell SPI which address you want to write to
  SPI.transfer(REG_FIFO | 0x80);

  // loop over the payload and put it on the buffer 
  for (i = 0; i < Length; i++)
  {
    SPI.transfer(buffer[i]);
  }
  unselect();

  // go into transmit mode
  setMode(RF98_MODE_TX);
  LoRaMode = lmSending;
  InFSKMode = 0;

  // read temperature while chip is awake
  GPS.InternalTemperature = lora_read_temperature();
}


int BuildLoRaPositionPacket(unsigned char *TxLine)
{
  struct TBinaryPacket BinaryPacket;

  SentenceCounter++;

  BinaryPacket.PayloadIDs = 0xC0 | (LORA_ID << 3) | LORA_ID;
  BinaryPacket.Counter = SentenceCounter;
  BinaryPacket.BiSeconds = GPS.SecondsInDay / 2L;
  BinaryPacket.Latitude = GPS.Latitude;
  BinaryPacket.Longitude = GPS.Longitude;
  BinaryPacket.Altitude = GPS.Altitude;

  memcpy(TxLine, &BinaryPacket, sizeof(BinaryPacket));
	
  return sizeof(struct TBinaryPacket);
}

void SwitchToFSKMode(void)
{
  unsigned long FrequencyValue;

  Serial.println("Setting FSK Mode");
  setMode(RF98_MODE_SLEEP);
  writeRegister(REG_OPMODE, 0x0);

  InFSKMode = 1;
  setMode(RF98_MODE_STANDBY);

  writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMIT
  writeRegister(REG_PA_CONFIG, POWERLEVEL);
  writeRegister(REG_PA_RAMP, 0x2A); // GFSK

  // Frequency
  FrequencyValue = (unsigned long)((LORA_FSK_FREQ + (LORA_OFFSET / 1000.0)) * 16384.0f);
  writeRegister(REG_FRF_MSB, (FrequencyValue >> 16) & 0xFF);   // Set frequency
  writeRegister(REG_FRF_MID, (FrequencyValue >> 8) & 0xFF);
  writeRegister(REG_FRF_LSB, FrequencyValue & 0xFF);

  // Modem config
  writeRegister(REG_PLL_HOP,	 0x80 + 0x2d);	// default value + fasthop
  writeRegister(REG_PACKET_CONFIG1,	0x10);	// fixed length, no whitening, CRC on, no addressing
  writeRegister(REG_PACKET_CONFIG2,	0x40);	// packet mode
  writeRegister(REG_BITRATE_MSB,	0xFA);	// 32MHz / 256 * 250  => 500 Hz
  writeRegister(REG_BITRATE_LSB,	0x00);	// - no finer adjustment needed
  writeRegister(REG_FDEV_LSB,		0x07);	// 7 * 120 Hz => 840 Hz
  writeRegister(REG_PREAMBLE_FSK,	0x0F);	// - rfm preamble is odd
  writeRegister(REG_PAYLOAD_LENGTH_FSK,   48);	// 16 data + 32 FEC
  writeRegister(REG_SYNC_CONF,		0x17);	// 8 sync (4 are preamble)
  writeRegister(REG_SYNC1,		0x55);  // preamble
  writeRegister(REG_SYNC1 + 1,		0x55);  // preamble
  writeRegister(REG_SYNC1 + 2,		0x55);  // preamble
  writeRegister(REG_SYNC1 + 3,		0x55);  // preamble
  writeRegister(REG_SYNC1 + 4,		0x96);	// horus sync
  writeRegister(REG_SYNC1 + 5,		0x69);	// horus sync
  writeRegister(REG_SYNC1 + 6,		0x69);	// horus sync
  writeRegister(REG_SYNC1 + 7,		0x96);	// horus sync
}

void SendLoRaFSK()
{
  if ( !InFSKMode )
    SwitchToFSKMode();

  setMode(RF98_MODE_TX);

  // Set channel state
  LoRaMode = lmSending;
  SendingFSK = 1;
  send_mfsk_packet();

  // read temperature while chip is awake
  GPS.InternalTemperature = lora_read_temperature();
}

void CheckLoRa(void)
{
  if (LoRaIsFree())
  {		
    Serial.println("LoRa is free");
    {
      int PacketLength;

      if (FSKCount++)
      {
        FSKCount = 0;
        Serial.println(F("LoRa: Tx 4FSK packet"));
        SendLoRaFSK();
      } else {
        if (LORA_BINARY)
            PacketLength = BuildLoRaPositionPacket(Sentence);
        else
            PacketLength = BuildSentence((char *)Sentence, LORA_PAYLOAD_ID);
        Serial.println(F("LoRa: Tx LoRa packet"));
        SendLoRaPacket(Sentence, PacketLength);		
      }
    }
  }
}

#endif
