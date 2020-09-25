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

extern const unsigned char IMAGE_DATA2[1573];
extern const unsigned char IMAGE_DATA3[1440];
extern const unsigned char IMAGE_DATA4[3240];
extern const unsigned char IMAGE_DATA5[1820];
extern const unsigned char IMAGE_DATA6[700];
extern const unsigned char IMAGE_DATA7[390];
extern const unsigned char IMAGE_DATA8[1440];
extern const unsigned char IMAGE_DATA9[246];
extern const unsigned char IMAGE_KN100[48];
extern const unsigned char IMAGE_KN80[48];
extern const unsigned char IMAGE_KN60[48];
extern const unsigned char IMAGE_KN45[48];
extern const unsigned char IMAGE_KN30[48];
extern const unsigned char IMAGE_KN15[48];
extern const unsigned char IMAGE_KN0[48];
extern const unsigned char IMAGE_B100[64];
extern const unsigned char IMAGE_B70[64];
extern const unsigned char IMAGE_B40[64];
extern const unsigned char IMAGE_B10[64];
extern const unsigned char LZERO[720];
extern const unsigned char LONE[720];
extern const unsigned char LTWO[720];
extern const unsigned char LTHREE[720];
extern const unsigned char LFOUR[720];
extern const unsigned char LFIVE[720];
extern const unsigned char LSIX[720];
extern const unsigned char LSEVEN[720];
extern const unsigned char LEIGHT[720];
extern const unsigned char LNINE[720];
extern const unsigned char LNONE[720];
extern const unsigned char LOGO[2088];
extern const unsigned char LOGO2[2088];
extern const unsigned char SZERO[150];
extern const unsigned char SONE[150];
extern const unsigned char STWO[150];
extern const unsigned char STHREE[150];
extern const unsigned char SFOUR[150];
extern const unsigned char SFIVE[150];
extern const unsigned char SSIX[150];
extern const unsigned char SSEVEN[150];
extern const unsigned char SEIGHT[150];
extern const unsigned char SNINE[150];
extern const unsigned char SNONE[150];
extern const unsigned char PERS[80];
extern const unsigned char CELS[80];

extern const unsigned char NCON_ENG[900];
extern const unsigned char TEMP_ENG[200];
extern const unsigned char HUM_ENG[170];
extern const unsigned char BUTT_ZERO_ENG[750];
extern const unsigned char BUTT_ONE_ENG[750];
extern const unsigned char BUTT_ONEPLUS_ENG[750];
extern const unsigned char BUTT_TWO_ENG[750];
extern const unsigned char BUTT_END_ENG[300];
extern const unsigned char REP_TIME_ENG[1800];
extern const unsigned char REP_BATT_ENG[1800];

extern const unsigned char NCON_RU[900];
extern const unsigned char TEMP_RU[200];
extern const unsigned char HUM_RU[170];
extern const unsigned char BUTT_ZERO_RU[750];
extern const unsigned char BUTT_ONE_RU[750];
extern const unsigned char BUTT_ONEPLUS_RU[750];
extern const unsigned char BUTT_TWO_RU[750];
extern const unsigned char BUTT_END_RU[300];
extern const unsigned char REP_TIME_RU[1800];
extern const unsigned char REP_BATT_RU[1800];

extern const unsigned char NCON_ESP[900];
extern const unsigned char TEMP_ESP[200];
extern const unsigned char HUM_ESP[170];
extern const unsigned char BUTT_ZERO_ESP[750];
extern const unsigned char BUTT_ONE_ESP[750];
extern const unsigned char BUTT_ONEPLUS_ESP[750];
extern const unsigned char BUTT_TWO_ESP[750];
extern const unsigned char BUTT_END_ESP[300];
extern const unsigned char REP_TIME_ESP[1800];
extern const unsigned char REP_BATT_ESP[1800];

extern const unsigned char NCON_DE[900];
extern const unsigned char TEMP_DE[200];
extern const unsigned char HUM_DE[170];
extern const unsigned char BUTT_ZERO_DE[750];
extern const unsigned char BUTT_ONE_DE[750];
extern const unsigned char BUTT_ONEPLUS_DE[750];
extern const unsigned char BUTT_TWO_DE[750];
extern const unsigned char BUTT_END_DE[300];
extern const unsigned char REP_TIME_DE[1800];
extern const unsigned char REP_BATT_DE[1800];

extern const unsigned char NCON_FR[900];
extern const unsigned char TEMP_FR[200];
extern const unsigned char HUM_FR[170];
extern const unsigned char BUTT_ZERO_FR[750];
extern const unsigned char BUTT_ONE_FR[750];
extern const unsigned char BUTT_ONEPLUS_FR[750];
extern const unsigned char BUTT_TWO_FR[750];
extern const unsigned char BUTT_END_FR[300];
extern const unsigned char REP_TIME_FR[1800];
extern const unsigned char REP_BATT_FR[1800];

extern const unsigned char NCON_HI[900];
extern const unsigned char TEMP_HI[200];
extern const unsigned char HUM_HI[170];
extern const unsigned char BUTT_ZERO_HI[750];
extern const unsigned char BUTT_ONE_HI[750];
extern const unsigned char BUTT_ONEPLUS_HI[750];
extern const unsigned char BUTT_TWO_HI[750];
extern const unsigned char BUTT_END_HI[300];
extern const unsigned char REP_TIME_HI[1800];
extern const unsigned char REP_BATT_HI[1800];

extern const unsigned char NCON_ZHO[900];
extern const unsigned char TEMP_ZHO[200];
extern const unsigned char HUM_ZHO[170];
extern const unsigned char BUTT_ZERO_ZHO[750];
extern const unsigned char BUTT_ONE_ZHO[750];
extern const unsigned char BUTT_ONEPLUS_ZHO[750];
extern const unsigned char BUTT_TWO_ZHO[750];
extern const unsigned char BUTT_END_ZHO[300];
extern const unsigned char REP_TIME_ZHO[1800];
extern const unsigned char REP_BATT_ZHO[1800];

/* FILE END */
