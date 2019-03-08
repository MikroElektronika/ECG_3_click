![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# ECG_3 Click

- **CIC Prefix**  : ECG3
- **Author**      : Nemanja Medakovic
- **Verison**     : 1.0.0
- **Date**        : Oct 2018.

---

### Software Support

We provide a library for the ECG_3 Click on our [LibStock](https://libstock.mikroe.com/projects/view/2599/ecg-3-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

Library can perform a control of the ECG 3 Click board. 
Offers a choice to check registers, write to the registers, read ECG and RTOR Data.
Also library performs a calculations necessary to get Heart Rate in BPM value, and R - R Data in ms value.
For more details check documentation.

Key functions :

- ``` uint8_t ecg3_writeReg( uint8_t regAddr, uint32_t dataIn ) ``` - Function writes data to the register.
- ``` uint8_t ecg3_readReg( uint8_t regAddr, uint32_t *dataOut ) ``` - Function reads data from the register.
- ``` void ecg3_getECG( uint32_t *outECG ) ``` - Function reads ECG data from FIFO register.
- ``` void ecg3_getRTOR( uint16_t *outHR, uint16_t *outRR ) ``` - Function reads Heart Rate and R - R data and calculates Heart Rate data to BPM, 
  and R - R data to ms.

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes SPI interface and performs the all necessary configuration for device
  to work properly.
- Application Task - (code snippet) - Reads ECG Data every 8ms and sends this data to the serial plotter.


```.c
void applicationTask()
{
    ecg3_getECG( &ecgData );
    
    plotECG();
}
```

Additional Functions :

- void plotECG() - Sends ECG Data to the serial plotter.
- void logRTOR() - Sends Heart Rate and R - R Data to the uart terminal.

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2599/ecg-3-click) page.

Other mikroE Libraries used in the example:

- Conversions
- SPI
- UART

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
