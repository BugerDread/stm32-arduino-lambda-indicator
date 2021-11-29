//optimizations vs LCD speed
//opt     LCDinit LCDupdate
//Os      2107ms  222ms (default)
//Os+LTO  1714ms  134ms
//O2      2010ms  200ms
//O2+LTO  1649ms  119ms <=
//O3      2004ms  199ms   
//O3+LTO  1630ms  113ms

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "mercedescut.h"

//menu
  const uint8_t TEXT_W = 5;
  const uint8_t TEXT_H = 7;
  const uint8_t SPACER = 4;
  const uint16_t TXT_PARAM_COLOR = ST77XX_YELLOW;
  const uint16_t TXT_VAL_COLOR = ST77XX_GREEN;
  const uint8_t BATTERY_TXT_Y = 25;
  const uint8_t BATTERY_LINE_Y = BATTERY_TXT_Y + TEXT_H + SPACER;
  const uint8_t LAMBDA_TXT_Y = BATTERY_LINE_Y + SPACER;
  const uint8_t LAMBDA_LINE_Y = LAMBDA_TXT_Y + TEXT_H + SPACER;
  const uint8_t OVP_TXT_Y = LAMBDA_LINE_Y + SPACER;
  const uint8_t OVP_LINE_Y = OVP_TXT_Y + TEXT_H + SPACER;
  const uint8_t RPM_TXT_Y = OVP_LINE_Y + SPACER;
  const uint8_t RPM_LINE_Y = RPM_TXT_Y + TEXT_H + SPACER;
  const uint8_t DUTY_TXT_Y = RPM_LINE_Y + SPACER;
  const uint8_t DUTY_LINE_Y = DUTY_TXT_Y + TEXT_H + SPACER;
  const uint8_t ICV_TXT_Y = DUTY_LINE_Y + SPACER;
  const uint8_t ICV_LINE_Y = ICV_TXT_Y + TEXT_H + SPACER;
  const uint8_t LPG_TXT_Y = ICV_LINE_Y + SPACER;
  const uint8_t LPG_LINE_Y = LPG_TXT_Y + TEXT_H + SPACER;
  const uint8_t TXT_X = 0;
  const uint8_t LINE_X = 0;
  const uint8_t LINE_LEN = 160;
  const uint16_t LINE_COLOR = 0x528A;
  const uint16_t BACKGROUND_COLOR = ST77XX_BLACK;
  const uint16_t HEADER_COLOR = ST77XX_BLUE;
  const uint8_t VAL1_X = 80;
  const uint8_t VAL2_X = 120;
  const uint16_t GOOD_VAL_TXT_COLOR = ST77XX_BLACK;
  const uint16_t WARN_VAL_TXT_COLOR = ST77XX_BLACK;
  const uint16_t FAIL_VAL_TXT_COLOR = ST77XX_BLACK;
  const uint16_t GOOD_VAL_BGR_COLOR = ST77XX_GREEN;
  const uint16_t WARN_VAL_BGR_COLOR = ST77XX_YELLOW;
  const uint16_t FAIL_VAL_BGR_COLOR = ST77XX_RED;
  

//LCD           MOSI MISO SCLK
  SPIClass SPI3(PB5, PB4, PB3);
  const uint16_t TFT_CS = PA15;
  const uint16_t TFT_RST = PB6;      //PB11; ??and PB6 as RST
  const uint16_t TFT_DC = PB8;       //PB10; ??can we use MISO (PB4) for this as MISO is not used (no out from display to STM)?? - NOPE
  //Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
  Adafruit_ST7735 tft = Adafruit_ST7735(&SPI3, TFT_CS, TFT_DC, TFT_RST);

void lcd_init() {
  uint32_t m1 = millis();
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println(F("- DONE\r\nDrawing logo"));
  tft.drawRGBBitmap(26, 6, mblogo, 108, 108);
  tft.setCursor(8, 120);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLUE);
  tft.print(F("BugerDread 2021 ver 0.01"));
  uint32_t m2 = millis();
  Serial.printf(F("LCD init took %ums\r\n"), m2 - m1);  
}

void drawbasicscreen() {
  tft.fillScreen(BACKGROUND_COLOR);     //clear screen with background color

  //header
  tft.setCursor(3, 3);
  tft.setTextColor(HEADER_COLOR);
  tft.setTextSize(2);
  tft.setTextWrap(false);
  tft.print(F("MERCEDES-PILL"));
  tft.drawFastHLine(0, 19, 160, LINE_COLOR); //0x3186);
  tft.drawFastHLine(0, 20, 160, LINE_COLOR); //0x3186);
  tft.drawFastHLine(0, 0, 160, LINE_COLOR); //0x3186);
  tft.drawFastHLine(0, 1, 160, LINE_COLOR); //0x3186);

  //switch to draw parametters
  tft.setTextSize(1);
  tft.setTextColor(TXT_PARAM_COLOR);
  
  //battery
  tft.setCursor(TXT_X, BATTERY_TXT_Y);
  tft.print(F("Battery"));
  tft.drawFastHLine(LINE_X, BATTERY_LINE_Y, LINE_LEN, LINE_COLOR);

  //lambda
  tft.setCursor(TXT_X, LAMBDA_TXT_Y);
  tft.print(F("Lambda"));
  tft.drawFastHLine(LINE_X, LAMBDA_LINE_Y, LINE_LEN, LINE_COLOR);
  
  //OVP
  tft.setCursor(TXT_X, OVP_TXT_Y);
  tft.print(F("OVP relay"));
  tft.drawFastHLine(LINE_X, OVP_LINE_Y, LINE_LEN, LINE_COLOR);

  //RPM
  tft.setCursor(TXT_X, RPM_TXT_Y);
  tft.print(F("Engine RPM"));
  tft.drawFastHLine(LINE_X, RPM_LINE_Y, LINE_LEN, LINE_COLOR);

  //duty
  tft.setCursor(TXT_X, DUTY_TXT_Y);
  tft.print(F("Duty cycle"));
  tft.drawFastHLine(LINE_X, DUTY_LINE_Y, LINE_LEN, LINE_COLOR);

  //ICV
  tft.setCursor(TXT_X, ICV_TXT_Y);
  tft.print(F("ICV"));
  tft.drawFastHLine(LINE_X, ICV_LINE_Y, LINE_LEN, LINE_COLOR);

  //LPG (or whatever else)
  tft.setCursor(TXT_X, LPG_TXT_Y);
  tft.print(F("LPG tank"));
  tft.drawFastHLine(LINE_X, LPG_LINE_Y, LINE_LEN, LINE_COLOR);
}  

void showvalues() {

  //battery
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, BATTERY_TXT_Y);
  tft.printf("%u.%u%uV", battery_voltage / 1000, (battery_voltage / 100) % 10, (battery_voltage / 10) % 10);   
  //tft.printf("%u.%u%uV%n", battery_voltage / 1000, (battery_voltage / 100) % 10, (battery_voltage / 10) % 10, &len); //https://www.geeksforgeeks.org/g-fact-31/
  //tft.printf("%-*s", 6 - len, "V");                                                       //https://stackoverflow.com/questions/276827/string-padding-in-c
  if (battery_voltage < 10000) tft.print(" ");
  tft.setCursor(VAL2_X, BATTERY_TXT_Y);
  if (battery_voltage < V_BATT_FAIL) {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" FAIL "));
  } else if (battery_voltage < V_BATT_LOW) {
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.print(F(" LOW  "));
  } else if (battery_voltage >= V_BATT_HIGH) {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" HIGH "));
  } else {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  }

  //lambda
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, LAMBDA_TXT_Y);
  tft.printf("%umV", lambda_voltage);
  //spaces after to delete previous - total 6 chars, 2 used by "mV", one for first digit => 6-2-1=3
  if (lambda_voltage < 10) tft.print(" ");
  if (lambda_voltage < 100) tft.print(" ");
  if (lambda_voltage < 1000) tft.print(" ");
    
  if (lambda_voltage <= V_LEAN2) {
    //very lean
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F("V-LEAN"));
  } else if((V_LEAN2 < lambda_voltage) and (lambda_voltage <= V_LEAN1)) {
    //lean
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F(" LEAN "));
  } else if((V_LEAN1 < lambda_voltage) and (lambda_voltage < V_RICH1)) {
    //right
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F(" GOOD "));
  } else if((V_RICH1 <= lambda_voltage) and (lambda_voltage < V_RICH2)) {
    //rich
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F(" RICH "));
  } else if(V_RICH2 <= lambda_voltage) {
    //very rich
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F("V-RICH"));
  }      
  
  //ovp
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, OVP_TXT_Y);
  //tft.print(F("0.0V"));
  tft.printf("%u.%u%uV", ovp_voltage / 1000, (ovp_voltage / 100) % 10, (ovp_voltage / 10) % 10);
  if(ovp_voltage < 10000) tft.print(" ");
  //status
  tft.setCursor(VAL2_X, OVP_TXT_Y);
  if((ovp_voltage + 500) >= battery_voltage) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  } else {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" FAIL "));
  }

  //rpm
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, RPM_TXT_Y);
  tft.printf("%-6u", rpm_measured);
  tft.setCursor(VAL2_X, RPM_TXT_Y);
  if(rpm_measured == 0) {
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.print(F(" STOP "));
  } else if(rpm_measured <= RPM_IDLE_MAX) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F(" IDLE "));
  } else if(rpm_measured <= RPM_MAX) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  } else {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" HIGH "));
  }

  //duty
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, DUTY_TXT_Y);
  tft.printf("%u%%", duty_cycle_measured);
  if(duty_cycle_measured < 100) tft.print(" ");
  if(duty_cycle_measured < 10) tft.print(" ");
  //status
  tft.setCursor(VAL2_X, DUTY_TXT_Y);
  if ((duty_cycle_measured > DUTY_FAIL_HIGH) or (duty_cycle_measured < DUTY_FAIL_LOW)) {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.printf(F(" FAIL ")); 
  } else if((duty_freq_measured > DUTY_FREQ_HIGH) or (duty_freq_measured < DUTY_FREQ_LOW)) {
    //bad freq, should be 100Hz
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.printf(F(" FREQ "));
  } else if((duty_cycle_measured >= DUTY_WARN_HIGH) or (duty_cycle_measured <= DUTY_WARN_LOW)) {
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.printf(F(" WARN "));
  } else {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  }

  //ICV
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, ICV_TXT_Y);
  //tft.print(F("0.0V")); 
  tft.printf("%u.%u%uV", icv_voltage / 1000, (icv_voltage / 100) % 10, (icv_voltage / 10) % 10);
  if(icv_voltage < 10000) tft.print(" ");
  tft.setCursor(VAL2_X, ICV_TXT_Y);
  if(icv_voltage >= ICV_VOLTAGE_MIN) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  } else {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" FAIL "));
  } 

  //LPG
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, LPG_TXT_Y);
  tft.print(F("10%"));
  tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, LPG_TXT_Y);
  tft.print(F(" WARN "));  

}
