#ifndef _MSS_CASCADE_H_
#define _MSS_CASCADE_H_

#define CHANNEL0_ON_DELAY   0x00
#define CHANNEL1_ON_DELAY   0x01
#define CHANNEL0_OFF_DELAY  0x02
#define CHANNEL1_OFF_DELAY  0x03

#define ADC_CHANNEL_SETPOINT_0  0
#define ADC_CHANNEL_SETPOINT_1  1
#define ADC_CHANNEL_DETECTOR_0  2
#define ADC_CHANNEL_DETECTOR_1  3
#define ADC_CHANNEL_REDGRN_A    4
#define ADC_CHANNEL_REDGRN_B    5


#define CONFIG_SIGNAL_TYPE_0    0
#define CONFIG_SIGNAL_TYPE_1    1
#define CONFIG_SIGNAL_TYPE_2    2
#define CONFIG_SIGNAL_A_DUAL    3
#define CONFIG_SIGNAL_B_DUAL    4
#define CONFIG_SWITCH_6         5
#define CONFIG_SWITCH_7         6
#define CONFIG_SWITCH_8         7
#define CONFIG_SUPPRESS_BLINKY  8
#define CONFIG_APPROACH_LIT     9



// *******************************************
// From mss-cascade.c

extern volatile uint8_t eventFlags;

#define EVENT_DO_BD_READ 0x01
#define EVENT_DO_ADC_RUN 0x02
#define EVENT_1HZ_BLINK  0x04

typedef enum
{
	SIGNAL_3LIGHT_CA = 0,
	SIGNAL_3LIGHT_CC = 1,
	SIGNAL_3WIRE_RG_CA = 2,
	SIGNAL_3WIRE_RG_CC = 3,
	SIGNAL_2WIRE_RG    = 4,
	SIGNAL_UNKNOWN     = 32
} SignalHeadType;

typedef enum
{
	ASPECT_OFF          = 0,
	ASPECT_GREEN        = 1,
	ASPECT_YELLOW       = 2,
	ASPECT_FL_YELLOW    = 3,
	ASPECT_RED          = 4,
	ASPECT_FL_GREEN     = 5,
	ASPECT_FL_RED       = 6,
	ASPECT_LUNAR        = 7
} SignalAspect;

void translateCodelineToIndications();


// *******************************************
// From codeline.c

typedef enum 
{
	INDICATION_STOP               = 0,
	INDICATION_APPROACH           = 1,
	INDICATION_APPROACH_DIVERGING = 2,
	INDICATION_ADVANCE_APPROACH   = 3,
	INDICATION_CLEAR              = 4
} CodelineStatus;

extern CodelineStatus codeLineA;
extern CodelineStatus codeLineB;

void processMSSCodeline();

// *******************************************
// From detectors.c

#define DETECT_DCC_A  0
#define DETECT_DCC_B  1
#define DETECT_IR     2
#define ERROR_IR      3

#define   TMD26711_ADDR   0x39
#define   INFO_ADDR       0x20
#define   PROXIMITY_THRESHOLD   0x300
#define   PPULSE_DEFAULT        8
#define   ON_DEBOUNCE_DEFAULT   1
#define   OFF_DEBOUNCE_DEFAULT  10

// Externed variables
extern uint8_t detectorStatus;
extern volatile uint16_t adcValue[6];
// Functions
void initializeADC();
void initializeDetectors();
void triggerIRDetector();
void processDetectors(void);


// *******************************************
// From indicators.c

#define ON  1
#define OFF 0
#define BLINK (eventFlags & EVENT_1HZ_BLINK)

void setOccupancyAOn();
void setOccupancyAOff();
void setOccupancyBOn();
void setOccupancyBOff();
void setOccupancyALEDOn();
void setOccupancyALEDOff();
void setOccupancyALED(uint8_t state);
void setOccupancyBLEDOn();
void setOccupancyBLEDOff();
void setOccupancyBLED(uint8_t state);
void setAuxLEDOn();
void setAuxLEDOff();
void setAuxLED(uint8_t state);
void setOccupancyIRLEDOn();
void setOccupancyIRLEDOff();
void setOccupancyIRLED(uint8_t state);

// *******************************************
// From i2c.c

#define   I2C_SDA   PE5
#define   I2C_SCL   PE4
#define   I2C_SDA_DDR   DDRE
#define   I2C_SDA_PORT  PORTE 
#define   I2C_SDA_PIN   PINE
#define   I2C_SCL_DDR   DDRE
#define   I2C_SCL_PORT  PORTE

uint8_t i2cWriteByte(uint8_t byte);
uint8_t i2cReadByte(uint8_t ack);
uint8_t writeByte(uint8_t addr, uint8_t cmd, uint16_t writeVal);
uint8_t readWord(uint8_t addr, uint8_t cmd, uint16_t* data);



#endif


