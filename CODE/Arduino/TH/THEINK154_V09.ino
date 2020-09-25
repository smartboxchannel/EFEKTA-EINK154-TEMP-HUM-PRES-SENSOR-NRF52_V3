// ######################  Temperature and humidity sensor with electronic ink display 1.54 | nRF52 ####################### //
//                                                                                                                          //
//        @filename   :   EFEKTA_THEINK154_1.9.ino                                                                          //
//        @brief en   :   Wireless, battery-operated temperature and humidity sensor (SHT20, SI7020)                        //
//                        with electronic ink display(good display). Works on nRF52.                                        //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры и влажности(sht20, si7020)                            //
//                        с дисплеем на электронных чернилах(good display). Работает на nRF52.                              //
//        @author     :   Andrew Lamchenko aka Berk                                                                         //
//                                                                                                                          //
//        Copyright (C) EFEKTALAB 2020                                                                                      //
//                                                                                                                          //
// ######################################################################################################################## //

uint8_t colorPrint;
uint8_t opposite_colorPrint;
bool updateink1;
bool updateink2;
bool updateink3;
bool updateink4;
bool updateinkclear;
bool change;
bool chek_h = true;
bool first_send_battery = true;
bool check;
bool tch;
bool hch;
bool bch;
bool configMode;
bool button_flag;
bool nosleep;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool Ack_FP;

uint8_t lang;
uint8_t cpNom;
uint8_t cpCount;
uint8_t timeSend;
uint8_t battSend;
uint8_t battery;
uint8_t old_battery;
uint8_t err_delivery_beat;
uint8_t problem_mode_count;

float temperatureSend;
float humiditySend;
int16_t temperature;
int16_t humidity;
float old_temperature;
float old_humidity;
const float tempThreshold = 1.0;
const float humThreshold = 1.0;
int16_t nRFRSSI;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;

uint16_t temp;
uint16_t batteryVoltage;
uint16_t minuteT = 60000;
uint16_t BATT_TIME;
uint16_t BATT_COUNT;

uint32_t configMillis;
uint32_t previousMillis;
uint32_t SLEEP_TIME;
const uint32_t SLEEP_TIME_WDT = 5000;
uint32_t sleepTimeCount;
const uint32_t shortWait = 50;

float batteryVoltageF;

#include "gdef.h"
#include "eink54.h"
#include "eink54paint.h"
#include "eink54imagedata.h"
unsigned char image[5200];
Paint paint(image, 200, 200);
Epd epd;
#ifdef SHT20
#include "DFRobot_SHT20.h"
DFRobot_SHT20    sensor; // https://github.com/DFRobot/DFRobot_SHT20
#endif

#ifdef SI7020
#include "Adafruit_Si7021.h" // https://github.com/adafruit/Adafruit_Si7021
Adafruit_Si7021 sensor = Adafruit_Si7021();
#endif

#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif
#define MY_RADIO_NRF5_ESB

int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)

#define SN "EFEKTA T&H E-Ink"
#define SV "0.914"

#define TEMP_ID 1
#define HUM_ID 2
#define SIGNAL_Q_ID 100
#define BATTERY_VOLTAGE_ID 101
#define SET_TIME_SEND_ID 102
#define SET_BATT_SEND_ID 103
#define MY_SEND_RESET_REASON 105
#define SET_COLOR_ID 106


#include <MySensors.h>
MyMessage msgTemp(TEMP_ID, V_TEMP);
MyMessage msgHum(HUM_ID, V_HUM);
MyMessage sqMsg(SIGNAL_Q_ID, V_VAR1);
MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
MyMessage setTimeSendMsg(SET_TIME_SEND_ID, V_VAR1);
MyMessage setBattSendMsg(SET_BATT_SEND_ID, V_VAR1);
MyMessage sendMsg(MY_SEND_RESET_REASON, V_VAR1);
MyMessage setColor(SET_COLOR_ID, V_VAR1);

// SDK PORT
uint32_t PIN_BUTTON_MASK;
volatile byte buttIntStatus = 0;
#define APP_GPIOTE_MAX_USERS 1
extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
static app_gpiote_user_id_t m_gpiote_user_id;


void preHwInit() {
  pinMode(PIN_BUTTON, INPUT);
#if defined(EBYTE)
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
#endif
#ifdef DCDC_SDC
  pinMode(BATTERY_PIN, INPUT);
#endif
}


void before()
{
  //########################################## CONFIG MCU ###############################################

  NRF_POWER->DCDCEN = 1;
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;
#if defined(MINEW)
  NRF_TWIM0 ->ENABLE = 0;
  NRF_TWIS0 ->ENABLE = 0;
  NRF_RADIO->TXPOWER = 0x4UL;
#else
  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;
  NRF_RADIO->TXPOWER = 0x8UL;
#endif

#ifdef DCDC_SDC
  analogReadResolution(12);
  analogReference(AR_VDD4);
#endif


  //########################################## INIT HAPPY ##############################################

  happy_init();


  //########################################## CONFIG PROG ###############################################

  timeSend = loadState(102);
  if (timeSend > 30) {
    timeSend = 1;
    saveState(102, timeSend);
  }
  //timeSend = 1; // для теста, 1 минута

  battSend = loadState(103);
  if (battSend > 24) {
    battSend = 3;
    saveState(103, battSend);
  }
  battSend = 1; // для теста, 1 час

  if (loadState(106) > 1) {
    saveState(106, 0);
  }
  colorChange(loadState(106));
  //colorChange(false); // для теста, true или false

  timeConf();


  //########################################## EINK INIT ###############################################
  epd.Init(lut_full_update);
  clearOne();
  epd.Init(lut_partial_update);
  DrawImageWH(&epd, &paint, 40, 30, LOGO, 140, 116, colorPrint, ROTATE_270);
  epd.DisplayFrame();
  delay(2500);
  clearOne();
  DrawImageWH(&epd, &paint, 10, 40, IMAGE_DATA4, 137, 180, colorPrint, ROTATE_270);
  epd.DisplayFrame();
  delay(3500);
  clearOne();
#ifdef DEVICE_PRESENT
#if defined(EBYTE)
  DrawImageWH(&epd, &paint, 36, 90, IMAGE_DATA2, 100, 121, colorPrint, ROTATE_90);
  DrawImageWH(&epd, &paint, 10, 20, IMAGE_DATA3, 62, 180, colorPrint, ROTATE_270);
  epd.DisplayFrame();
  delay(500);
  clearOne();
  DrawImageWH(&epd, &paint, 36, 130, IMAGE_DATA6, 40, 140, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 36, 30, IMAGE_DATA5, 98, 140, colorPrint, ROTATE_270);
  epd.DisplayFrame();
  delay(500);
  clearOne();
  DrawImageWH(&epd, &paint, 38, 150, IMAGE_DATA7, 24, 130, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 10, 70, IMAGE_DATA8, 62, 180, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 61, 50, IMAGE_DATA9, 20, 82, colorPrint, ROTATE_270);
  epd.DisplayFrame();
  delay(500);
#endif
#endif

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 80, NCON_RU, 43, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 80, NCON_DE, 43, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 80, NCON_FR, 43, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 80, NCON_ESP, 43, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 80, NCON_HI, 43, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 80, NCON_ZHO, 43, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 80, NCON_ENG, 43, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
  e_Ink_Deep_Sleep();
}


void presentation()
{
  check = sendSketchInfo(SN, SV);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = sendSketchInfo(SN, SV);
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(TEMP_ID, S_TEMP, "Temperature");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(TEMP_ID, S_TEMP, "Temperature");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(HUM_ID, S_HUM, "Humidity");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(HUM_ID, S_HUM, "Humidity");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_TIME_SEND_ID, S_CUSTOM, "T&H SEND INTERVAL | Min");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_TIME_SEND_ID, S_CUSTOM, "T&H SEND INTERVAL | Min");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
  wait(shortWait * 2);
  sendConfig();
  wait(shortWait);
}


void setup() {
  CORE_DEBUG(PSTR("MyS: CONFIG HAPPY NODE\n"));
  config_Happy_node();
  CORE_DEBUG(PSTR("MyS: READ BATTERY VOLTAGE\n"));
  readBatt();
  CORE_DEBUG(PSTR("MyS: SEND CONFIG PARAMETERS\n"));
  if (flag_nogateway_mode == false) {
    wait(shortWait);
    sendResetReason();
  }
  CORE_DEBUG(PSTR("MyS: INTERRUPT CONFIG\n"));
  interrupt_Init();
  CORE_DEBUG(PSTR("MyS: DISABLE RADIO\n"));
  transportDisable();
  sleepTimeCount = SLEEP_TIME;
#ifdef SHT20
  sensor.initSHT20();
#endif
#ifdef SI7020
  sensor.begin();
#endif
  wait(shortWait);
  CORE_DEBUG(PSTR("MyS: WDT ENABLE\n"));
  wdt_enable();
}


void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_nogateway_mode == false) {
      if (flag_find_parent_process == true) {
        find_parent_process();
      }

      if (configMode == false) {
        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
            wdt_nrfReset();
            epd.Reset();
            button_flag = true;
            previousMillis = millis();
          }
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
            if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 4500)) {
              if (updateink1 == false) {
                einkZeropush();
                updateink1 = true;
              }
            }
            if ((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500)) {
              if (updateink2 == false) {
                einkOnepush();
                updateink2 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 8500) && (millis() - previousMillis <= 9500)) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 12500)) {
              if (updateink4 == false) {
                einkTwopush();
                updateink4 = true;
                updateinkclear = false;
              }
            }
            if (millis() - previousMillis > 12500) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
          }
          if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {

            if ((millis() - previousMillis <= 4500) && (button_flag == true))
            {
              wdt_nrfReset();
              einkZeroend();
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              presentation();
              transportDisable();
              change = true;
              sleepTimeCount = SLEEP_TIME;
            }
            if ((millis() - previousMillis > 5500 && millis() - previousMillis <= 8500) && button_flag == true)
            {
              wdt_nrfReset();
              einkOneend();
              reseteinkset();
              configMode = true;
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              wait(shortWait);
              NRF5_ESB_startListening();
              wait(shortWait);
              configMillis = millis();
            }
            if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 12500) && (button_flag == true))
            {
              einkTwoend();
              new_device();
            }
            if ((((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) || ((millis() - previousMillis > 8500) && (millis() - previousMillis <= 9500)) || (millis() - previousMillis > 12500) ) && (button_flag == true) )
            {
              wdt_nrfReset();
              change = true;
              sleepTimeCount = SLEEP_TIME;
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
            }
          }
          wdt_nrfReset();
        } else {
          sleepTimeCount++;
          nosleep = false;
          wait(10);
          if (!nosleep) {
            nosleep = false;
            wait(10);
          }
          wdt_nrfReset();
          if (sleepTimeCount >= SLEEP_TIME) {
            sleepTimeCount = 0;
            readData();
            if (change == true) {
              change = false;
              wait(shortWait);
              transportReInitialise();
              wait(shortWait);
              sendData();
              transportDisable();
              wait(shortWait);
              epd.Reset();
              wait(shortWait);
              convert(temperature, humidity);
              wait(shortWait);
              e_Ink_Deep_Sleep();
            }
          }
          nosleep = false;
        }
      } else {
        if (millis() - configMillis > 15000) {
          configMode = false;
          button_flag = false;
          buttIntStatus = 0;
          transportDisable();
          wait(shortWait * 2);
          change = true;
          sleepTimeCount = SLEEP_TIME;
#if defined(EBYTE)
          digitalWrite(GREEN_LED, LOW);
          wait(50);
          digitalWrite(GREEN_LED, HIGH);
#endif
        }
        wdt_nrfReset();
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
          wdt_nrfReset();
          epd.Reset();
          button_flag = true;
          previousMillis = millis();
        }
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
          if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 4500)) {
            if (updateink1 == false) {
              einkZeropush();
              updateink1 = true;
            }
          }
          if ((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) {
            if (updateinkclear == false) {
              clearOne();
              updateinkclear = true;
            }
          }
          if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500)) {
            if (updateink4 == false) {
              einkTwopush();
              updateink4 = true;
              updateinkclear = false;
            }
          }
          if (millis() - previousMillis > 8500) {
            if (updateinkclear == false) {
              clearOne();
              updateinkclear = true;
            }
          }
        }
        if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {
          if (millis() - previousMillis <= 4500 && button_flag == true)
          {
            wdt_nrfReset();
            einkZeroend();
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
            transportReInitialise();
            check_parent();
            cpCount = 0;
            change = true;
            sleepTimeCount = SLEEP_TIME;
          }
          if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500) && (button_flag == true))
          {
            einkTwoend();
            new_device();
          }
          if ( ( ( millis() - previousMillis > 4500 && millis() - previousMillis <= 5500 ) || ( millis() - previousMillis > 8500)) && button_flag == true)
          {
            wdt_nrfReset();
            change = true;
            sleepTimeCount = SLEEP_TIME;
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
          }
        }
        wdt_nrfReset();
      } else {
        sleepTimeCount++;
        if (sleepTimeCount >= SLEEP_TIME) {
          sleepTimeCount = 0;
          readData();
          if (change == true) {
            change = false;
            wait(shortWait);
            transportReInitialise();
            wait(shortWait);
            sendData();
            transportDisable();
            wait(shortWait);
            epd.Reset();
            wait(shortWait);
            convert(temperature, humidity);
            wait(shortWait);
            e_Ink_Deep_Sleep();
          }
          sleepTimeCount = 0;
          cpCount++;
          if (cpCount >= cpNom) {
            transportReInitialise();
            check_parent();
            cpCount = 0;
          }
        }
        if ((cpCount < cpNom) && (flag_nogateway_mode == true)) {
          nosleep = false;
        }
      }
    }
  }
  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(201);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(202);
    mypar = _transportConfig.parentNodeId;
    nosleep = false;
    err_delivery_beat = 6;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == false) {
    wdt_nrfReset();
    hwSleep(SLEEP_TIME_WDT);
    nosleep = true;
  }
  wdt_nrfReset();
}


//################################################### EINK ###################################################

void DrawImageWH(Epd * epd, Paint * paint, int X, int Y, const uint8_t* imgData, int Width, int Height, uint8_t intcolored, int Rotate )
{
  if (Rotate == ROTATE_90 || Rotate == ROTATE_270) {
    paint->SetWidth(Height);
    paint->SetHeight(Width);
  } else {
    paint->SetWidth(Width);
    paint->SetHeight(Height);
  }
  paint->SetRotate(Rotate);

  if (intcolored == 0x00) {
    paint->Clear(0xFF);
  } else {
    paint->Clear(0x00);
  }

  int i, j;
  const unsigned char* prt = imgData;
  for (j = 0; j < Height; j++) {
    for (i = 0; i < Width; i++) {
      if (pgm_read_byte(prt) & (0x80 >> (i % 8))) {
        paint->DrawPixel(i, j, intcolored);
      }
      if (i % 8 == 7) {
        prt++;
      }
    }
    if (Width % 8 != 0) {
      prt++;
    }
  }
  epd->SetFrameMemory(paint->GetImage(), X, Y, paint->GetWidth(), paint->GetHeight());
}


void clearOne() {
  epd.ClearFrameMemory(opposite_colorPrint);
  epd.DisplayFrame();
  if (colorPrint == 0xFF) {
    epd.ClearFrameMemory(colorPrint);
    epd.DisplayFrame();
  } else {
    epd.ClearFrameMemory(opposite_colorPrint);
    epd.DisplayFrame();
  }
}


void e_Ink_Deep_Sleep() {
  epd.Init(lut_partial_update);
  wait(10);
  epd.SendCommand(0x10);
  wait(10);
  epd.SendData(0x01);
  wait(130);
}


void reseteinkset() {
  updateink1 = false;
  updateink2 = false;
  updateink3 = false;
  updateink4 = false;
  updateinkclear = false;
}


void einkZeropush() {

  clearOne();
#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_RU, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_ESP, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_DE, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_FR, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_HI, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_ZHO, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_ENG, 36, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkZeroend() {
  if (colorPrint == 0xFF) {
    clearOne();
  }
#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_RU, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_RU, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_ESP, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ESP, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_DE, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_DE, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_FR, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_FR, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_HI, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_HI, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_ZHO, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ZHO, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ZERO_ENG, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ENG, 16, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkOnepush() {

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_RU, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_ESP, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_DE, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_FR, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_HI, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_ZHO, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_ENG, 36, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkOneend() {
  if (colorPrint == 0xFF) {
    clearOne();
  }
#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_RU, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_RU, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_ESP, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ESP, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_DE, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_DE, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_FR, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_FR, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_HI, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_HI, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_ZHO, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ZHO, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONE_ENG, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ENG, 16, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkOnePluspush() {

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_RU, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_ESP, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_DE, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_FR, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_HI, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_ZHO, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_ENG, 36, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkOnePlusend() {
  if (colorPrint == 0xFF) {
    clearOne();
  }
#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_RU, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_RU, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_ESP, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ESP, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_DE, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_DE, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_FR, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_FR, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_HI, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_HI, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_ZHO, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ZHO, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_ONEPLUS_ENG, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ENG, 16, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkTwopush() {

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_RU, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_ESP, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_DE, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_FR, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_HI, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_ZHO, 36, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_ENG, 36, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void einkTwoend() {
  if (colorPrint == 0xFF) {
    clearOne();
  }
#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_RU, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_RU, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_ESP, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ESP, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_DE, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_DE, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_FR, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_FR, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_HI, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_HI, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_ZHO, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ZHO, 16, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 95, BUTT_TWO_ENG, 36, 150, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 30, 60, BUTT_END_ENG, 16, 150, colorPrint, ROTATE_270);
#endif
  epd.DisplayFrame();
}


void reportTimeInk() {
  if (colorPrint == 0xFF) {
    clearOne();
  }
  //timeSend = 5; // для теста

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_RU, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_ESP, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_DE, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_FR, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_HI, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_ZHO, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 70, REP_TIME_ENG, 90, 150, colorPrint, ROTATE_270);
#endif

  if (timeSend >= 10) {
    byte one_t = timeSend / 10;
    byte two_t = timeSend % 10;
    switch (one_t) {
      case 0:
        DrawImageWH(&epd, &paint, 102, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 1:
        DrawImageWH(&epd, &paint, 102, 22, SONE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 2:
        DrawImageWH(&epd, &paint, 102, 22, STWO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 3:
        DrawImageWH(&epd, &paint, 102, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 4:
        DrawImageWH(&epd, &paint, 102, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
        break;
      case 5:
        DrawImageWH(&epd, &paint, 102, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 6:
        DrawImageWH(&epd, &paint, 102, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
        break;
      case 7:
        DrawImageWH(&epd, &paint, 102, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
        break;
      case 8:
        DrawImageWH(&epd, &paint, 102, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
        break;
      case 9:
        DrawImageWH(&epd, &paint, 102, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
        break;
      default:
        DrawImageWH(&epd, &paint, 102, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
        break;
    }

    switch (two_t) {
      case 0:
        DrawImageWH(&epd, &paint, 64, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 1:
        DrawImageWH(&epd, &paint, 64, 22, SONE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 2:
        DrawImageWH(&epd, &paint, 64, 22, STWO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 3:
        DrawImageWH(&epd, &paint, 64, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 4:
        DrawImageWH(&epd, &paint, 64, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
        break;
      case 5:
        DrawImageWH(&epd, &paint, 64, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 6:
        DrawImageWH(&epd, &paint, 64, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
        break;
      case 7:
        DrawImageWH(&epd, &paint, 64, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
        break;
      case 8:
        DrawImageWH(&epd, &paint, 64, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
        break;
      case 9:
        DrawImageWH(&epd, &paint, 64, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
        break;
      default:
        DrawImageWH(&epd, &paint, 64, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
        break;
    }
  } else {
    switch (timeSend) {
      case 0:
        DrawImageWH(&epd, &paint, 83, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 1:
        DrawImageWH(&epd, &paint, 83, 22, SONE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 2:
        DrawImageWH(&epd, &paint, 83, 22, STWO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 3:
        DrawImageWH(&epd, &paint, 83, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 4:
        DrawImageWH(&epd, &paint, 83, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
        break;
      case 5:
        DrawImageWH(&epd, &paint, 83, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 6:
        DrawImageWH(&epd, &paint, 83, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
        break;
      case 7:
        DrawImageWH(&epd, &paint, 83, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
        break;
      case 8:
        DrawImageWH(&epd, &paint, 83, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
        break;
      case 9:
        DrawImageWH(&epd, &paint, 83, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
        break;
      default:
        DrawImageWH(&epd, &paint, 83, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
        break;
    }
  }
  epd.DisplayFrame();
  wait(1500);
}


void reportBattInk() {
  if (colorPrint == 0xFF) {
    clearOne();
  }
  //battSend = 5; // для теста

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_RU, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_ESP, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_DE, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_FR, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_HI, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_ZHO, 90, 150, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 30, 70, REP_BATT_ENG, 90, 150, colorPrint, ROTATE_270);
#endif

  if (battSend >= 10) {
    byte one_t = battSend / 10;
    byte two_t = battSend % 10;
    switch (one_t) {
      case 0:
        DrawImageWH(&epd, &paint, 102, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 1:
        DrawImageWH(&epd, &paint, 102, 22, SONE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 2:
        DrawImageWH(&epd, &paint, 102, 22, STWO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 3:
        DrawImageWH(&epd, &paint, 102, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 4:
        DrawImageWH(&epd, &paint, 102, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
        break;
      case 5:
        DrawImageWH(&epd, &paint, 102, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 6:
        DrawImageWH(&epd, &paint, 102, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
        break;
      case 7:
        DrawImageWH(&epd, &paint, 102, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
        break;
      case 8:
        DrawImageWH(&epd, &paint, 102, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
        break;
      case 9:
        DrawImageWH(&epd, &paint, 102, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
        break;
      default:
        DrawImageWH(&epd, &paint, 102, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
        break;
    }

    switch (two_t) {
      case 0:
        DrawImageWH(&epd, &paint, 64, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 1:
        DrawImageWH(&epd, &paint, 64, 22, SONE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 2:
        DrawImageWH(&epd, &paint, 64, 22, STWO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 3:
        DrawImageWH(&epd, &paint, 64, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 4:
        DrawImageWH(&epd, &paint, 64, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
        break;
      case 5:
        DrawImageWH(&epd, &paint, 64, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 6:
        DrawImageWH(&epd, &paint, 64, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
        break;
      case 7:
        DrawImageWH(&epd, &paint, 64, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
        break;
      case 8:
        DrawImageWH(&epd, &paint, 64, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
        break;
      case 9:
        DrawImageWH(&epd, &paint, 64, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
        break;
      default:
        DrawImageWH(&epd, &paint, 64, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
        break;
    }
  } else {
    switch (battSend) {
      case 0:
        DrawImageWH(&epd, &paint, 83, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 1:
        DrawImageWH(&epd, &paint, 83, 22, SONE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 2:
        DrawImageWH(&epd, &paint, 83, 22, STWO, 40, 30, colorPrint, ROTATE_270);
        break;
      case 3:
        DrawImageWH(&epd, &paint, 83, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 4:
        DrawImageWH(&epd, &paint, 83, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
        break;
      case 5:
        DrawImageWH(&epd, &paint, 83, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
        break;
      case 6:
        DrawImageWH(&epd, &paint, 83, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
        break;
      case 7:
        DrawImageWH(&epd, &paint, 83, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
        break;
      case 8:
        DrawImageWH(&epd, &paint, 83, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
        break;
      case 9:
        DrawImageWH(&epd, &paint, 83, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
        break;
      default:
        DrawImageWH(&epd, &paint, 83, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
        break;
    }
  }
  epd.DisplayFrame();
  wait(1500);
}


void display_Table()
{
  String D = ("ID:" + String(getNodeId()));
  int str_len10 = D.length() + 1;
  char my_sB10[str_len10];
  D.toCharArray(my_sB10, str_len10);
  paint.SetWidth(200);
  paint.SetHeight(24);
  paint.SetRotate(ROTATE_0);
  paint.Clear(opposite_colorPrint);
  paint.DrawHorizontalLine(0, 18 , 200, colorPrint);
  paint.DrawVerticalLine(120, 0 , 18, colorPrint);
  paint.DrawVerticalLine(80, 0 , 18, colorPrint);
  paint.SetRotate(ROTATE_180);
  paint.DrawStringAt(145, 11, my_sB10, &Font12, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  paint.SetRotate(ROTATE_180);
  paint.Clear(opposite_colorPrint);
  paint.DrawHorizontalLine(0, 10 , 45, colorPrint);
  paint.DrawHorizontalLine(155, 10 , 45, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 0, 175, paint.GetWidth(), paint.GetHeight());
  paint.Clear(opposite_colorPrint);
  paint.DrawHorizontalLine(0, 10 , 50, colorPrint);
  paint.DrawHorizontalLine(150, 10 , 50, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 0, 57, paint.GetWidth(), paint.GetHeight());

  uint8_t s = nRFRSSI;
  if (flag_nogateway_mode == true) {
    s = 0;
  }
  if (s == 0) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN0, 24, 16, colorPrint, ROTATE_0);
  }
  if (s > 0 && s < 15) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN15, 24, 16, colorPrint, ROTATE_0);
  }
  if (s >= 15 && s < 30) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN30, 24, 16, colorPrint, ROTATE_0);
  }
  if (s >= 30 && s < 45) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN45, 24, 16, colorPrint, ROTATE_0);
  }
  if (s >= 45 && s <= 60) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN60, 24, 16, colorPrint, ROTATE_0);
  }
  if (s >= 60 && s <= 80) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN80, 24, 16, colorPrint, ROTATE_0);
  }
  if (s >= 80 && s <= 100) {
    DrawImageWH(&epd, &paint, 90, 1, IMAGE_KN100, 24, 16, colorPrint, ROTATE_0);
  }

  uint8_t b = battery;
  if (b >= 0 && b < 10) {
    DrawImageWH(&epd, &paint, 160, 1, IMAGE_B10, 32, 16, colorPrint, ROTATE_0);
  }
  if (b >= 10 && b < 40) {
    DrawImageWH(&epd, &paint, 160, 1, IMAGE_B40, 32, 16, colorPrint, ROTATE_0);
  }
  if (b >= 40 && b < 80) {
    DrawImageWH(&epd, &paint, 160, 1, IMAGE_B70, 32, 16, colorPrint, ROTATE_0);
  }
  if (b >= 80 && b <= 100) {
    DrawImageWH(&epd, &paint, 160, 1, IMAGE_B100, 32, 16, colorPrint, ROTATE_0);
  }

  String C = (String(b) + "%");
  int str_len3 = C.length() + 1;
  char my_sB3[str_len3];
  C.toCharArray(my_sB3, str_len3);
  paint.SetWidth(30);
  paint.SetHeight(16);
  paint.SetRotate(ROTATE_180);
  paint.Clear(opposite_colorPrint);
  paint.DrawStringAt(6, 4, my_sB3, &Font12, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 130, 1, paint.GetWidth(), paint.GetHeight());
  old_battery = b;

#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 52, 180, TEMP_RU, 15, 100, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 52, 180, TEMP_ESP, 15, 100, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 52, 180, TEMP_DE, 15, 100, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 52, 180, TEMP_FR, 15, 100, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 52, 180, TEMP_HI, 15, 100, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 52, 180, TEMP_ZHO, 15, 100, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 52, 180, TEMP_ENG, 15, 100, colorPrint, ROTATE_270);
#endif


#ifdef LANG_RU
  DrawImageWH(&epd, &paint, 60, 64, HUM_RU, 15, 85, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ESP
  DrawImageWH(&epd, &paint, 60, 64, HUM_ESP, 15, 85, colorPrint, ROTATE_270);
#endif
#ifdef LANG_DE
  DrawImageWH(&epd, &paint, 60, 64, HUM_DE, 15, 85, colorPrint, ROTATE_270);
#endif
#ifdef LANG_FR
  DrawImageWH(&epd, &paint, 60, 64, HUM_FR, 15, 85, colorPrint, ROTATE_270);
#endif
#ifdef LANG_HI
  DrawImageWH(&epd, &paint, 60, 64, HUM_HI, 15, 85, colorPrint, ROTATE_270);
#endif
#ifdef LANG_ZHO
  DrawImageWH(&epd, &paint, 60, 64, HUM_ZHO, 15, 85, colorPrint, ROTATE_270);
#endif
#ifdef LANG_EN
  DrawImageWH(&epd, &paint, 60, 64, HUM_ENG, 15, 85, colorPrint, ROTATE_270);
#endif
}


void convert(int t, int h) {
  clearOne();
  //clearOne();
  display_Table();

  //t = 99;  // for test
  byte one_t = t / 10;
  byte two_t = t % 10;

  switch (one_t) {
    case 0:
      DrawImageWH(&epd, &paint, 106, 88, LZERO, 90, 60, colorPrint, ROTATE_270);
      break;
    case 1:
      DrawImageWH(&epd, &paint, 106, 88, LONE, 90, 60, colorPrint, ROTATE_270);
      break;
    case 2:
      DrawImageWH(&epd, &paint, 106, 88, LTWO, 90, 60, colorPrint, ROTATE_270);
      break;
    case 3:
      DrawImageWH(&epd, &paint, 106, 88, LTHREE, 90, 60, colorPrint, ROTATE_270);
      break;
    case 4:
      DrawImageWH(&epd, &paint, 106, 88, LFOUR, 90, 60, colorPrint, ROTATE_270);
      break;
    case 5:
      DrawImageWH(&epd, &paint, 106, 88, LFIVE, 90, 60, colorPrint, ROTATE_270);
      break;
    case 6:
      DrawImageWH(&epd, &paint, 106, 88, LSIX, 90, 60, colorPrint, ROTATE_270);
      break;
    case 7:
      DrawImageWH(&epd, &paint, 106, 88, LSEVEN, 90, 60, colorPrint, ROTATE_270);
      break;
    case 8:
      DrawImageWH(&epd, &paint, 106, 88, LEIGHT, 90, 60, colorPrint, ROTATE_270);
      break;
    case 9:
      DrawImageWH(&epd, &paint, 106, 88, LNINE, 90, 60, colorPrint, ROTATE_270);
      break;
    default:
      DrawImageWH(&epd, &paint, 106, 88, LNONE, 90, 60, colorPrint, ROTATE_270);
      break;
  }

  switch (two_t) {
    case 0:
      DrawImageWH(&epd, &paint, 34, 88, LZERO, 90, 60, colorPrint, ROTATE_270);
      break;
    case 1:
      DrawImageWH(&epd, &paint, 34, 88, LONE, 90, 60, colorPrint, ROTATE_270);
      break;
    case 2:
      DrawImageWH(&epd, &paint, 34, 88, LTWO, 90, 60, colorPrint, ROTATE_270);
      break;
    case 3:
      DrawImageWH(&epd, &paint, 34, 88, LTHREE, 90, 60, colorPrint, ROTATE_270);
      break;
    case 4:
      DrawImageWH(&epd, &paint, 34, 88, LFOUR, 90, 60, colorPrint, ROTATE_270);
      break;
    case 5:
      DrawImageWH(&epd, &paint, 34, 88, LFIVE, 90, 60, colorPrint, ROTATE_270);
      break;
    case 6:
      DrawImageWH(&epd, &paint, 34, 88, LSIX, 90, 60, colorPrint, ROTATE_270);
      break;
    case 7:
      DrawImageWH(&epd, &paint, 34, 88, LSEVEN, 90, 60, colorPrint, ROTATE_270);
      break;
    case 8:
      DrawImageWH(&epd, &paint, 34, 88, LEIGHT, 90, 60, colorPrint, ROTATE_270);
      break;
    case 9:
      DrawImageWH(&epd, &paint, 34, 88, LNINE, 90, 60, colorPrint, ROTATE_270);
      break;
    default:
      DrawImageWH(&epd, &paint, 34, 88, LNONE, 90, 60, colorPrint, ROTATE_270);
      break;
  }
  //h = 99;  // for test
  byte one_h = h / 10;
  byte two_h = h % 10;

  switch (one_h) {
    case 0:
      DrawImageWH(&epd, &paint, 102, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
      break;
    case 1:
      DrawImageWH(&epd, &paint, 102, 22, SONE, 40, 30, colorPrint, ROTATE_270);
      break;
    case 2:
      DrawImageWH(&epd, &paint, 102, 22, STWO, 40, 30, colorPrint, ROTATE_270);
      break;
    case 3:
      DrawImageWH(&epd, &paint, 102, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
      break;
    case 4:
      DrawImageWH(&epd, &paint, 102, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
      break;
    case 5:
      DrawImageWH(&epd, &paint, 102, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
      break;
    case 6:
      DrawImageWH(&epd, &paint, 102, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
      break;
    case 7:
      DrawImageWH(&epd, &paint, 102, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
      break;
    case 8:
      DrawImageWH(&epd, &paint, 102, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
      break;
    case 9:
      DrawImageWH(&epd, &paint, 102, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
      break;
    default:
      DrawImageWH(&epd, &paint, 102, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
      break;
  }

  switch (two_h) {
    case 0:
      DrawImageWH(&epd, &paint, 64, 22, SZERO, 40, 30, colorPrint, ROTATE_270);
      break;
    case 1:
      DrawImageWH(&epd, &paint, 64, 22, SONE, 40, 30, colorPrint, ROTATE_270);
      break;
    case 2:
      DrawImageWH(&epd, &paint, 64, 22, STWO, 40, 30, colorPrint, ROTATE_270);
      break;
    case 3:
      DrawImageWH(&epd, &paint, 64, 22, STHREE, 40, 30, colorPrint, ROTATE_270);
      break;
    case 4:
      DrawImageWH(&epd, &paint, 64, 22, SFOUR, 40, 30, colorPrint, ROTATE_270);
      break;
    case 5:
      DrawImageWH(&epd, &paint, 64, 22, SFIVE, 40, 30, colorPrint, ROTATE_270);
      break;
    case 6:
      DrawImageWH(&epd, &paint, 64, 22, SSIX, 40, 30, colorPrint, ROTATE_270);
      break;
    case 7:
      DrawImageWH(&epd, &paint, 64, 22, SSEVEN, 40, 30, colorPrint, ROTATE_270);
      break;
    case 8:
      DrawImageWH(&epd, &paint, 64, 22, SEIGHT, 40, 30, colorPrint, ROTATE_270);
      break;
    case 9:
      DrawImageWH(&epd, &paint, 64, 22, SNINE, 40, 30, colorPrint, ROTATE_270);
      break;
    default:
      DrawImageWH(&epd, &paint, 64, 22, SNONE, 40, 30, colorPrint, ROTATE_270);
      break;
  }

  DrawImageWH(&epd, &paint, 50, 32, PERS, 30, 20, colorPrint, ROTATE_270);
  DrawImageWH(&epd, &paint, 14, 143, CELS, 30, 20, colorPrint, ROTATE_270);

  epd.DisplayFrame();
}


//########################################## BATTARY ###################################################


void readBatt() {
#ifdef DCDC_SDC
  uint16_t temp2 = 0;
  for (byte i = 0; i < 5; i++) {
    temp = analogRead(BATTERY_PIN);
    temp2 = temp2 + temp;
    wait(5);
  }
  temp = temp2 / 5;
  batteryVoltage = (float)temp * 0.998;
  battery = battery_level_in_percent(batteryVoltage);
  batteryVoltageF = (float)batteryVoltage / 1000.00;
  wait(5);
  CORE_DEBUG(PSTR("battery analog read: %d\n"), temp);
  CORE_DEBUG(PSTR("battery voltage: %d\n"), batteryVoltage);
  CORE_DEBUG(PSTR("battery percentage: %d\n"), battery);
#else
  batteryVoltage = hwCPUVoltage();
  wait(10);
  battery = battery_level_in_percent(batteryVoltage);
  batteryVoltageF = (float)batteryVoltage / 1000.00;
  CORE_DEBUG(PSTR("battery voltage: %d\n"), batteryVoltage);
  CORE_DEBUG(PSTR("battery percentage: %d\n"), battery);
#endif
  if (BATT_TIME != 0) {
    bch = 1;
  }
}


void batLevSend() {
  if (BATT_TIME != 0) {
    if (battery > 100) {
      battery = 100;
    }
    check = sendBatteryLevel(battery, 1);
    wait(500);
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(50);
      setRadio();
      check = sendBatteryLevel(battery, 1);
      wait(500);
    }
    if (check == true) {
      err_delivery_beat = 0;
      if (flag_nogateway_mode == true) {
        flag_nogateway_mode = false;
        CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
        err_delivery_beat = 0;
      }
      CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
      CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
    } else {
      _transportSM.failedUplinkTransmissions = 0;
      if (err_delivery_beat < 6) {
        err_delivery_beat++;
      }
      if (err_delivery_beat == 5) {
        if (flag_nogateway_mode == false) {
          happy_node_mode();
          gateway_fail();
          CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
        }
      }
    }
    check = send(bvMsg.set(batteryVoltageF, 2));
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
      setRadio();
      check = send(bvMsg.set(batteryVoltageF, 2));
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
    } else {
      CORE_DEBUG(PSTR("MyS: SEND BATTERY VOLTAGE\n"));
    }
    lqSend();
  }
}


void lqSend() {
  nRFRSSI = transportGetReceivingRSSI();
  nRFRSSI = map(nRFRSSI, -85, -40, 0, 100);
  if (nRFRSSI < 0) {
    nRFRSSI = 0;
  }
  if (nRFRSSI > 100) {
    nRFRSSI = 100;
  }
#if defined(EBYTE)
  if ((nRFRSSI >= 90) && (NRF_RADIO->TXPOWER == 0x8UL)) {
    NRF_RADIO->TXPOWER = 0x4UL;
  } else if ((nRFRSSI <= 50) && (NRF_RADIO->TXPOWER == 0x4UL))  {
    NRF_RADIO->TXPOWER = 0x8UL;
  }
#else
  if ((nRFRSSI >= 90) && (NRF_RADIO->TXPOWER == 0x4UL)) {
    NRF_RADIO->TXPOWER = 0x0UL;
  } else if ((nRFRSSI <= 50) && (NRF_RADIO->TXPOWER == 0x0UL))  {
    NRF_RADIO->TXPOWER = 0x4UL;
  }
#endif
  check = send(sqMsg.set(nRFRSSI));
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait * 2);
    setRadio();
    check = send(sqMsg.set(nRFRSSI));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    CORE_DEBUG(PSTR("MyS: SEND LINK QUALITY\n"));
    CORE_DEBUG(PSTR("MyS: LINK QUALITY %: %d\n"), nRFRSSI);
  }
}


static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 20) / 100;
  }
  else if (mvolts > 2750)
  {
    battery_level = 80 - ((2900 - mvolts) * 30) / 150;
  }
  else if (mvolts > 2550)
  {
    battery_level = 50 - ((2750 - mvolts) * 40) / 200;
  }
  else if (mvolts > 2250)
  {
    battery_level = 10 - ((2550 - mvolts) * 10) / 300;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}


//####################################### SENSOR DATA ##################################################


void readData() {
  CORE_DEBUG(PSTR("MyS: START SENSOR INIT\n"));
  wait(shortWait);
  CORE_DEBUG(PSTR("MyS: END SENSOR INIT\n"));
  temperatureSend = sensor.readTemperature();
  wait(shortWait);
  temperature = round(temperatureSend);
  if (chek_h == true) {
    humiditySend = sensor.readHumidity();
    wait(shortWait);
    humidity = round(humiditySend);
    if (humidity < 0) {
      humidity = 0;
    }
    if (humidity > 99) {
      humidity = 99;
    }
    chek_h = false;
  } else {
    chek_h = true;
  }

  if (temperature < 0) {
    temperature = 0;
  }
  if (temperature > 99) {
    temperature = 99;
  }

  if (abs(temperatureSend - old_temperature) >= tempThreshold) {
    old_temperature = temperatureSend;
    change = true;
    tch = true;
  }
  if (abs(humiditySend - old_humidity) >= humThreshold) {
    old_humidity = humiditySend;
    change = true;
    hch = true;
  }

  BATT_COUNT++;
  CORE_DEBUG(PSTR("BATT_COUNT: %d\n"), BATT_COUNT);
  if (BATT_COUNT >= BATT_TIME) {
    CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
    change = true;
  }
#if defined(EBYTE)
  if (change == true) {
    digitalWrite(BLUE_LED, LOW);
    wait(30);
    digitalWrite(BLUE_LED, HIGH);
  }
#endif
}

void sendData() {
  if (flag_nogateway_mode == false) {
    if (timeSend != 0) {
      if (tch == true) {
        check = send(msgTemp.setDestination(0).set(temperatureSend, 2));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(shortWait * 4);
          check = send(msgTemp.setDestination(0).set(temperatureSend, 2));
          if (check == false) {
            wait(shortWait * 8);
            _transportSM.failedUplinkTransmissions = 0;
            setRadio();
            check = send(msgTemp.setDestination(0).set(temperatureSend, 2));
            wait(shortWait * 2);
          }
        }
        tch = false;

        if (check == true) {
          err_delivery_beat = 0;
          if (flag_nogateway_mode == true) {
            flag_nogateway_mode = false;
            CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
            err_delivery_beat = 0;
          }
          CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
          CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
        } else {
          _transportSM.failedUplinkTransmissions = 0;
          if (err_delivery_beat < 6) {
            err_delivery_beat++;
          }
          if (err_delivery_beat == 5) {
            if (flag_nogateway_mode == false) {
              happy_node_mode();
              gateway_fail();
              CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
            }
          }
        }
      }

      if (hch == true) {
        check = send(msgHum.setDestination(0).set(humidity, 0));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(shortWait * 4);
          check = send(msgHum.setDestination(0).set(humidity, 0));
          if (check == false) {
            wait(shortWait * 8);
            _transportSM.failedUplinkTransmissions = 0;
            setRadio();
            check = send(msgHum.setDestination(0).set(humidity, 0));
            wait(shortWait * 2);
          }
        }
        hch = false;

        if (check == true) {
          err_delivery_beat = 0;
          if (flag_nogateway_mode == true) {
            flag_nogateway_mode = false;
            CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
            err_delivery_beat = 0;
          }
          CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
          CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
        } else {
          _transportSM.failedUplinkTransmissions = 0;
          if (err_delivery_beat < 6) {
            err_delivery_beat++;
          }
          if (err_delivery_beat == 5) {
            if (flag_nogateway_mode == false) {
              happy_node_mode();
              gateway_fail();
              CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
            }
          }
        }
      }
    }
    if (BATT_COUNT >= BATT_TIME) {
      CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
      wait(10);
      readBatt();
      BATT_COUNT = 0;
    }

    if (bch == true) {
      if (BATT_TIME != 0) {
        batLevSend();
      }
      bch = false;
    }
  } else {
    if (BATT_COUNT >= BATT_TIME) {
      CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
      wait(10);
      readBatt();
      BATT_COUNT = 0;
    }
    tch = false;
    hch = false;
    bch = false;
#if defined(EBYTE)
    digitalWrite(RED_LED, LOW);
    wait(30);
    digitalWrite(RED_LED, HIGH);
#endif
  }
  wdt_nrfReset();
}


//####################################### CONF ##################################################

void wdt_nrfReset() {
  wdt_reset();
  wait(10);
  wdt_reset();
  wait(20);
}

void colorChange(bool flag) {
  if (flag == true) {
    colorPrint = 0xFF;
    opposite_colorPrint = 0x00;
  } else {
    colorPrint = 0x00;
    opposite_colorPrint = 0xFF;
  }
  saveState(106, flag);
}

void setRadio() {
#if defined(EBYTE)
  if (NRF_RADIO->TXPOWER == 0x4UL) {
    NRF_RADIO->TXPOWER = 0x8UL;
  }
#else
  if (NRF_RADIO->TXPOWER == 0x0UL) {
    NRF_RADIO->TXPOWER = 0x4UL;
  }
#endif
}


void timeConf() {
  if (timeSend != 0) {
    SLEEP_TIME = (timeSend * minuteT / SLEEP_TIME_WDT) - 1;
  } else {
    SLEEP_TIME = (minuteT / SLEEP_TIME_WDT) - 1;
  }
  if (battSend != 0) {
    if (timeSend != 0) {
      BATT_TIME = (battSend * 60 / timeSend) - 1;
    } else {
      BATT_TIME = (battSend * 60) - 1;
    }
  } else {
    BATT_TIME = 0;
  }
  if (timeSend != 0) {
    cpNom = (60 / timeSend) - 1;
  } else {
    cpNom = 60 - 1;
  }
  CORE_DEBUG(PSTR("SLEEP_TIME: %d\n"), SLEEP_TIME);
}


void sendConfig() {
  check = send(setTimeSendMsg.set(timeSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setTimeSendMsg.set(timeSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(setBattSendMsg.set(battSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setBattSendMsg.set(battSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
  bool colorPrintTemp = loadState(106);
  check = send(setColor.set(colorPrintTemp));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setColor.set(colorPrintTemp));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
}


void sendResetReason() {
  String reason;
#ifdef MY_RESET_REASON_TEXT
  if (NRF_POWER->RESETREAS == 0) reason = "POWER_ON";
  else {
    if (NRF_POWER->RESETREAS & (1UL << 0)) reason += "PIN_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 1)) reason += "WDT ";
    if (NRF_POWER->RESETREAS & (1UL << 2)) reason += "SOFT_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 3)) reason += "LOCKUP";
    if (NRF_POWER->RESETREAS & (1UL << 16)) reason += "WAKEUP_GPIO ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "LPCOMP ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "WAKEUP_DEBUG";
  }
#else
  reason = NRF_POWER->RESETREAS;
#endif

  check = send(sendMsg.set(reason.c_str()));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(sendMsg.set(reason.c_str()));
    if (check == false) {
      wait(shortWait * 3);
      _transportSM.failedUplinkTransmissions = 0;
      check = send(sendMsg.set(reason.c_str()));
      wait(shortWait);
    }
  }
  if (check) NRF_POWER->RESETREAS = (0xFFFFFFFF);
}


//############################################## RECEIVE CONF ##################################################

void receive(const MyMessage & message)
{
  if (configMode == true) {
    if (message.sensor == SET_TIME_SEND_ID) {
      if (message.type == V_VAR1) {
        timeSend = message.getByte();
        if (timeSend > 60) {
          timeSend = 60;
        }
        saveState(102, timeSend);
        wait(shortWait);
        send(setTimeSendMsg.set(timeSend));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        reportTimeInk();
        configMode = false;
        //nosleep = false;
        change = true;
        timeConf();
        sleepTimeCount = SLEEP_TIME;
      }
    }

    if (message.sensor == SET_BATT_SEND_ID) {
      if (message.type == V_VAR1) {
        battSend = message.getByte();
        if (battSend > 24) {
          battSend = 24;
        }
        saveState(103, battSend);
        wait(shortWait);
        send(setBattSendMsg.set(battSend));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        reportBattInk();
        configMode = false;
        //nosleep = false;
        change = true;
        timeConf();
        sleepTimeCount = SLEEP_TIME;
      }
    }

    if (message.sensor == SET_COLOR_ID) {
      if (message.type == V_VAR1) {
        bool colorPrintTemp = message.getBool();
        colorChange(colorPrintTemp);
        wait(shortWait);
        send(setColor.set(colorPrintTemp));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        configMode = false;
        //nosleep = false;
        change = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }
  }
}


//################################################ INTERRUPTS #################################################

void interrupt_Init() {
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  app_gpiote_user_register(&m_gpiote_user_id, PIN_BUTTON_MASK, PIN_BUTTON_MASK, gpiote_event_handler);
  app_gpiote_user_enable(m_gpiote_user_id);
  buttIntStatus = 0;
}

void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2);

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
    if (buttIntStatus == 0) {
      buttIntStatus = PIN_BUTTON;
    }
  }
}


//################################################ F RESET ########################################################

void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(200, 255);
  hwReboot();
}


//############################################## HAPPY MODE #####################################################

void happy_init() {

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(200) == 0) {
    saveState(200, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(200));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 0;
  } else {
    mtwr = 15000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}


void config_Happy_node() {
  if (mtwr == 0) {
    myid = getNodeId();
    saveState(200, myid);
    mypar = _transportConfig.parentNodeId;
    old_mypar = mypar;
    saveState(201, mypar);
    saveState(202, _transportConfig.distanceGW);
  }
  if (mtwr != 0) {
    myid = getNodeId();
    if (myid != loadState(200)) {
      saveState(200, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      if (mypar != loadState(201)) {
        saveState(201, mypar);
      }
      if (_transportConfig.distanceGW != loadState(202)) {
        saveState(202, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 6;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(1000, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == C_INTERNAL) {
      if (_msg.type == 8) {
        Ack_FP = true;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == true) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    wdt_nrfReset();
    Ack_FP = false;
    transportSwitchSM(stParent);
    flag_nogateway_mode = false;
    flag_find_parent_process = true;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    transportDisable();
    wait(shortWait);
    nosleep = false;
  }
}


void find_parent_process() {
  flag_update_transport_param = true;
  flag_find_parent_process = false;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
}


void gateway_fail() {
  flag_nogateway_mode = true;
  flag_update_transport_param = false;
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0u;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = false;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = true;
    }
  }
}


void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  if (mypar != loadState(201))
  {
    saveState(201, mypar);
  }
  if (_transportConfig.distanceGW != loadState(202))
  {
    saveState(202, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(50);
  flag_update_transport_param = false;
  sleepTimeCount = SLEEP_TIME;
  BATT_COUNT = BATT_TIME;
}


void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}
