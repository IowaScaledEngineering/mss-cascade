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

// Externed variables
extern uint8_t detectorStatus;

// Functions
void initializeADC();
void initializeDetectors();
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

#endif


