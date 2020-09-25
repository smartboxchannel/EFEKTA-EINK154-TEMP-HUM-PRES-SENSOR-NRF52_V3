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
//        Copyright (C) Waveshare     September 5 2017                                                                      //
//                                                                                                                          //
// ######################################################################################################################## //

#ifndef EPDIF_H
#define EPDIF_H

#include <arduino.h>

// Pin definition


class EpdIf {
public:
    EpdIf(void);
    ~EpdIf(void);

    static int  IfInit(void);
    static void DigitalWrite(int pin, int value); 
    static int  DigitalRead(int pin);
    static void DelayMs(unsigned int delaytime);
    static void SpiTransfer(unsigned char data);
};

#endif
