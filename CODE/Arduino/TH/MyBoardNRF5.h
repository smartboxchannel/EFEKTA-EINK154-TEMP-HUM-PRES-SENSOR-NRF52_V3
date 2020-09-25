// ######################  Temperature and humidity sensor with electronic ink display 1.54 | nRF52 ####################### //
//                                                                                                                          //
//        @filename   :   EFEKTA_THEINK154_1.8.ino                                                                          //
//        @brief en   :   Wireless, battery-operated temperature and humidity sensor (SHT20, SI7020)                        //
//                        with electronic ink display(good display). Works on nRF52.                                        //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры и влажности(sht20, si7020)                            //
//                        с дисплеем на электронных чернилах(good display). Работает на nRF52.                              //
//        @author     :   Andrew Lamchenko aka Berk                                                                         //
//                                                                                                                          //
//        Copyright (C) EFEKTALAB 2020                                                                                      //
//                                                                                                                          //
// ######################################################################################################################## //

#ifndef _MYBOARDNRF5_H_
#define _MYBOARDNRF5_H_

#include "gdef.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (8u)


//   LEDs

#if defined(HOLYIOT)
#define PIN_LED1              (8)
#define LED_BUILTIN           PIN_LED1
#elif defined(EBYTE)
#define PIN_LED1              (24)
#define PIN_LED2              (20)
#define PIN_LED3              (13)
#define BLUE_LED               (PIN_LED2)
#define GREEN_LED             (PIN_LED3)
#define RED_LED              (PIN_LED1)
#define LED_BUILTIN           (PIN_LED1)
#endif


  //   Buttons

#if defined(MINEW)
#define PIN_BUTTON             (11)
#elif defined(HOLYIOT)
#define PIN_BUTTON             (19)
#elif defined(EBYTE)
#define PIN_BUTTON             (7)
#endif

#ifdef DCDC_SDC
#define BATTERY_PIN             (2)
#endif

#if defined(MINEW)
#define RST_PIN         10
#define DC_PIN          4
#define CS_PIN          29
#define BUSY_PIN        3
#elif defined(EBYTE)
#define RST_PIN         (30)
#define DC_PIN          (4)
#define CS_PIN          (29)
#define BUSY_PIN        (12)
#elif defined(HOLYIOT)
#define RST_PIN         (17)
#define DC_PIN          (16)
#define CS_PIN          (5)
#define BUSY_PIN        (15)
#endif

    /*
       Analog ports

       If you change g_APinDescription, replace PIN_AIN0 with
       port numbers mapped by the g_APinDescription Array.
       You can add PIN_AIN0 to the g_APinDescription Array if
       you want provide analog ports MCU independed, you can add
       PIN_AIN0..PIN_AIN7 to your custom g_APinDescription Array
       defined in MyBoardNRF5.cpp
    */
static const uint8_t A0  = ADC_A0;
static const uint8_t A1  = ADC_A1;
static const uint8_t A2  = ADC_A2;
static const uint8_t A3  = ADC_A3;
static const uint8_t A4  = ADC_A4;
static const uint8_t A5  = ADC_A5;
static const uint8_t A6  = ADC_A6;
static const uint8_t A7  = ADC_A7;

/*
   Serial interfaces

   RX and TX are required.
   If you have no serial port, use unused pins
   CTS and RTS are optional.
*/
#if defined(MINEW)
#define PIN_SERIAL_RX       (27)
#define PIN_SERIAL_TX       (18)
#elif defined(HOLYIOT)
#define PIN_SERIAL_RX       (12)
#define PIN_SERIAL_TX       (11)
#elif defined(EBYTE)
#define PIN_SERIAL_RX       (3)
#define PIN_SERIAL_TX       (28)
#endif
/*
   SPI Interfaces

   This is optional

   If SPI is defined MISO, MOSI, SCK are required
   SS is optional and can be used in your sketch.
*/
#define SPI_INTERFACES_COUNT 1

#if defined(MINEW)
#define PIN_SPI_MISO         (2)
#define PIN_SPI_MOSI         (30)
#define PIN_SPI_SCK          (9)
#define PIN_SPI_SS           (29)
#elif defined(HOLYIOT)
#define PIN_SPI_MISO         (2)
#define PIN_SPI_MOSI         (3)
#define PIN_SPI_SCK          (4)
#define PIN_SPI_SS           (5)
#elif defined(EBYTE)
#define PIN_SPI_MISO         (22)
#define PIN_SPI_MOSI         (31)
#define PIN_SPI_SCK          (5)
#define PIN_SPI_SS           (29)
#endif

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
   Wire Interfaces

   This is optional
*/
#define WIRE_INTERFACES_COUNT 2

#if defined(MINEW)
#define PIN_WIRE_SDA         (12u)
#define PIN_WIRE_SCL         (17u)
#elif defined(HOLYIOT)
#define PIN_WIRE_SDA         (13u)
#define PIN_WIRE_SCL         (14u)
#elif defined(EBYTE)
#define PIN_WIRE_SDA         (15u)
#define PIN_WIRE_SCL         (17u)
#endif

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
