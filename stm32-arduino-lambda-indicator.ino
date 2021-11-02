//Lambda (O2) sensor indicator by BugerDread

//includes
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "mercedescut.h"

//constants
  //general
  const uint16_t V_REFI = 1208;       //STM32F103 internal reference voltage [mV]
  const uint16_t V_LEAN2 = 100;       //very lean mixture voltage [mV]
  const uint16_t V_LEAN1 = 200;       //lean mixture voltage [mV]
  const uint16_t V_RICH1 = 700;       //rich mixture voltage [mV]
  const uint16_t V_RICH2 = 800;       //very rich mixture voltage [mV]
  const uint16_t CYCLE_DELAY = 50;    //delay for each round [ms]
  const uint8_t N_AVG = 20;           //how many samples to average to show on serial
  
  //inputs
  const uint16_t LAMBDA_INPUT = A0;   //lambda sensor voltage input pin (rich >= ~0.7V, lean <= ~0.2V)

  //outputs
  const uint16_t LED_LEAN2 = PB12;    //very lean mixture LED - on when LAMBDA_INPUT voltage <= V_LEAN2
  const uint16_t LED_LEAN1 = PB13;    //lean mixture LED - on when V_LEAN2 < LAMBDA_INPUT voltage <= V_LEAN1
  const uint16_t LED_RIGHT = PB14;    //right mixture LED - on when V_LEAN1 < LAMBDA_INPUT voltage < V_RICH1
  const uint16_t LED_RICH1 = PB15;    //rich mixture  LED - on when V_RICH1 <= LAMBDA_INPUT voltage < V_RICH2
  const uint16_t LED_RICH2 = PA8;     //very rich mixture LED - on when V_RICH2 <= LAMBDA_INPUT voltage
  const uint16_t LED_ONBOARD = PC13;  //LED on the bluepill board

  //LCD pins
  const uint16_t TFT_CS = PA4;
  const uint16_t TFT_RST = PB0;
  const uint16_t TFT_DC = PB1;

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
  

//global variables
uint16_t lambda_voltage_avg, lambda_voltage, lambda_value, vref_value, i;
uint32_t lambda_voltage_avg_sum ;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void showvlues() {

  //battery
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, BATTERY_TXT_Y);
  tft.print(F("13.6V"));
  tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, BATTERY_TXT_Y);
  tft.print(F("  OK  "));

  //lambda
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, LAMBDA_TXT_Y);
  tft.printf("%umV", lambda_voltage_avg);
  //spaces after to delete previous - total 6 chars, 2 used by "mV", one for first digit => 6-2-1=3
  if (lambda_voltage_avg < 10) tft.print(" ");
  if (lambda_voltage_avg < 100) tft.print(" ");
  if (lambda_voltage_avg < 1000) tft.print(" ");
  
  
  if (lambda_voltage_avg <= V_LEAN2) {
    //very lean
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F("V-LEAN"));
  } else if ((V_LEAN2 < lambda_voltage_avg) and (lambda_voltage_avg <= V_LEAN1)) {
    //lean
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F(" LEAN "));
  } else if ((V_LEAN1 < lambda_voltage_avg) and (lambda_voltage_avg < V_RICH1)) {
    //right
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F(" GOOD "));
  } else if ((V_RICH1 <= lambda_voltage_avg) and (lambda_voltage_avg < V_RICH2)) {
    //rich
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F(" RICH "));
  } else if (V_RICH2 <= lambda_voltage) {
    //very rich
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.setCursor(VAL2_X, LAMBDA_TXT_Y);
    tft.print(F("V-RICH"));
  }      
  

  //ovp
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, OVP_TXT_Y);
  tft.print(F("0.0V"));
  tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, OVP_TXT_Y);
  tft.print(F(" FAIL "));

  //rpm
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, RPM_TXT_Y);
  tft.print(F("2500"));
  tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, RPM_TXT_Y);
  tft.print(F(" RUN  "));

  //duty
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, DUTY_TXT_Y);
  tft.print(F("59.8%"));
  tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, DUTY_TXT_Y);
  tft.print(F("  OK  "));

  //ICV
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, ICV_TXT_Y);
  tft.print(F("0.0V"));  
  tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, ICV_TXT_Y);
  tft.print(F(" FAIL "));  

  //LPG
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, LPG_TXT_Y);
  tft.print(F("10%"));
  tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
  tft.setCursor(VAL2_X, LPG_TXT_Y);
  tft.print(F(" WARN "));  

}

void drawbasicscreen() {
  //clear screen with background color
  tft.fillScreen(BACKGROUND_COLOR);

  //header
  tft.setCursor(3, 3);
  tft.setTextColor(HEADER_COLOR);
  tft.setTextSize(2);
  tft.setTextWrap(false);
  tft.print(F("MERCEDES-DIAG"));
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

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  //while (!Serial);                  //need to be removed, otherwise will not run WO PC :D
  Serial.print(F("\r\n* * * BGR Lambda Indicator v0.01 * * *\r\n\r\nConfiguration\r\n=============\r\nLED update delay: "));
  Serial.print(CYCLE_DELAY);
  Serial.print(F("ms\r\nSamples avg for serial console: "));
  Serial.print(N_AVG);
  Serial.print(F("\r\nVery lean mixture: "));
  Serial.print(V_LEAN2);
  Serial.print(F("mV\r\nLean mixture: "));
  Serial.print(V_LEAN1);
  Serial.print(F("mV\r\nRich mixture: "));
  Serial.print(V_RICH1);
  Serial.print(F("mV\r\nVery rich mixture: "));
  Serial.print(V_RICH2);
  Serial.print(F("mV\r\n\r\nLCD init "));

  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println(F("- DONE\r\nDrawing logo"));
  tft.drawRGBBitmap(26, 6, mblogo, 108, 108);
  tft.setCursor(8, 120);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLUE);
  tft.print(F("BugerDread 2021 ver 0.01"));
  Serial.println(F("- DONE\r\n"));
  
  //ADC will use 12bit accuracy
  analogReadResolution(12);

  //init LED pins
  pinMode(LED_LEAN2, OUTPUT);
  digitalWrite(LED_LEAN2, LOW);
  pinMode(LED_LEAN1, OUTPUT);
  digitalWrite(LED_LEAN1, HIGH);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_RIGHT, HIGH);
  pinMode(LED_RICH1, OUTPUT);
  digitalWrite(LED_RICH1, HIGH);
  pinMode(LED_RICH2, OUTPUT);
  digitalWrite(LED_RICH2, HIGH);
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, HIGH);
  
  //do some fancy fx with LEDs on boot
  delay(250);
  digitalWrite(LED_LEAN2, HIGH);
  digitalWrite(LED_LEAN1, LOW);
  delay(250);
  digitalWrite(LED_LEAN1, HIGH);
  digitalWrite(LED_RIGHT, LOW);
  delay(250);
  digitalWrite(LED_RIGHT, HIGH);
  digitalWrite(LED_RICH1, LOW);
  delay(250);
  digitalWrite(LED_RICH1, HIGH);
  digitalWrite(LED_RICH2, LOW);
  delay(750);

  drawbasicscreen();
  delay(500);
  
 
}

void loop() {
  lambda_voltage_avg_sum = 0;
  for (i = 1; i <= N_AVG; i++) {    //print status to console only every N-th cycle
    vref_value = analogRead(AVREF);
    lambda_value = analogRead(LAMBDA_INPUT);
    lambda_voltage = ((uint32_t)lambda_value * V_REFI) / vref_value;
    lambda_voltage_avg_sum += lambda_voltage;
  
    //control LEDz
    if (lambda_voltage <= V_LEAN2) {
      //very lean
      digitalWrite(LED_LEAN2, LOW);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("#0000");
      
    } else if ((V_LEAN2 < lambda_voltage) and (lambda_voltage <= V_LEAN1)) {
      //lean
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, LOW);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("0#000");
      
    } else if ((V_LEAN1 < lambda_voltage) and (lambda_voltage < V_RICH1)) {
      //right
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, LOW);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("00#00");
      
    } else if ((V_RICH1 <= lambda_voltage) and (lambda_voltage < V_RICH2)) {
      //rich
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, LOW);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("000#0");
      
    } else if (V_RICH2 <= lambda_voltage) {
      //very rich
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, LOW);
      if (i == N_AVG) Serial.print("0000#");
      
    }
    delay(CYCLE_DELAY);
  }
  digitalWrite(LED_ONBOARD, !digitalRead(LED_ONBOARD));   //flash the onboard LED to indicate we are alive
  lambda_voltage_avg = lambda_voltage_avg_sum / N_AVG;
  Serial.printf(" - lambda voltage avg: %umV\r\n", lambda_voltage_avg);
  showvlues();
}
