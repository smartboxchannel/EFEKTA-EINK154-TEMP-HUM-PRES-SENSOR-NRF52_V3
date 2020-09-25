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

#define LANG_RU
//#define LANG_EN
//#define LANG_ESP
//#define LANG_DE
//#define LANG_FR
//#define LANG_HI
//#define LANG_ZHO
//#define LANG_CN

#define MINEW // nRF52810, nRF52811
//#define EBYTE // nRF52840
//#define HOLYIOT // nRF52832


//#define MY_DEBUG
//#define DCDC_SDC
//#define DEVICE_PRESENT // not for nrf52810, nrf52811
#define MY_RESET_REASON_TEXT
//#define SHT20
#define SI7020
