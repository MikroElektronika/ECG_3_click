/*
Example for ECG_3 Click

    Date          : Oct 2018.
    Author        : Nemanja Medakovic

Test configuration KINETIS :
    
    MCU              : MK64
    Dev. Board       : HEXIWEAR
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes SPI interface and performs the all necessary configuration for device
  to work properly.
- Application Task - (code snippet) - Reads ECG Data every 8ms and sends this data to the serial plotter.

Additional Functions :

- void plotECG() - Sends ECG Data to the serial plotter.
- void logRTOR() - Sends Heart Rate and R - R Data to the uart terminal.

*/

#include "Click_ECG_3_types.h"
#include "Click_ECG_3_config.h"

uint32_t ecgData;
uint16_t rrData;
uint16_t hrData;
uint32_t measTimeCnt;
char text[ 50 ];

void plotECG()
{
    if (ecgData > 50000)
    {
        LongWordToStr( ecgData, text );
        mikrobus_logWrite( text, _LOG_TEXT );
        mikrobus_logWrite( ",", _LOG_TEXT );
        LongWordToStr( measTimeCnt, text );
        mikrobus_logWrite( text, _LOG_LINE );
        
        if (measTimeCnt == 0xFFFFFFFF)
        {
            measTimeCnt = 0;
        }
        else
        {
            measTimeCnt++;
        }
    }
    Delay_ms( 8 );
}

void logRTOR()
{
    if ((rrData != 0) && (hrData != 65535))
    {
        WordToStr( rrData, text );
        mikrobus_logWrite( "R - R Interval : ", _LOG_TEXT );
        mikrobus_logWrite( text, _LOG_TEXT );
        mikrobus_logWrite( " ms", _LOG_LINE );
    
        WordToStr( hrData, text );
        mikrobus_logWrite( "Heart Rate : ", _LOG_TEXT );
        mikrobus_logWrite( text, _LOG_TEXT );
        mikrobus_logWrite( " BPM", _LOG_LINE );
    }
    Delay_ms( 2000 );
}

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_PWM_PIN, _GPIO_OUTPUT );

    mikrobus_spiInit( _MIKROBUS1, &_ECG3_SPI_CFG[0] );

    mikrobus_logInit( _MIKROBUS2, 115200 );
    mikrobus_logWrite( "Initializing...", _LOG_LINE );

    Delay_ms( 100 );
}

void applicationInit()
{
    ecg3_spiDriverInit( (T_ECG3_P)&_MIKROBUS1_GPIO, (T_ECG3_P)&_MIKROBUS1_SPI );
    Delay_ms( 300 );
    
    ecg3_swReset();
    ecg3_fifoReset();
    Delay_ms( 100 );
    
    ecg3_init();
    measTimeCnt = 0;
    Delay_ms( 300 );
    
    mikrobus_logWrite( "ECG 3 is initialized", _LOG_LINE );
    mikrobus_logWrite( "", _LOG_LINE );
}

void applicationTask()
{
    ecg3_getECG( &ecgData );
    
    plotECG();
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
        applicationTask();
    }
}
