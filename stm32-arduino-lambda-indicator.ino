//MECEDES-PILL - W124-DIAG
//by BugerDread

//options
//board: buepillF103CB (or C8 with 128k)
//USART support: disabled (no Serial support)
//USB support: CDC (generic Serial supersede USART)

//needed by hardware-timer-based duty-cycle measurement
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

//includes
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "mercedescut.h"

// #define SDEBUG

//constants
  //general
  const uint16_t V_REFI = 1208;       //STM32F103 internal reference voltage [mV]
  const uint16_t V_LEAN2 = 100;       //very lean mixture voltage [mV]
  const uint16_t V_LEAN1 = 200;       //lean mixture voltage [mV]
  const uint16_t V_RICH1 = 700;       //rich mixture voltage [mV]
  const uint16_t V_RICH2 = 800;       //very rich mixture voltage [mV]
  const uint16_t CYCLE_DELAY = 1000;    //delay for each round [ms]
  //const uint8_t N_AVG = 20;           //how many samples to average to show on serial
  const uint16_t V_BATT_FAIL = 11000; //voltage [mV] below that battery is FAILED
  const uint16_t V_BATT_LOW = 12500; //voltage [mV] below that battery is LOW
  const uint16_t V_BATT_HIGH = 14600; //voltage [mV] below that battery is HIGH
  const uint16_t ICV_VOLTAGE_MIN = 3900;  //minimum ICV voltage, if lower error is shown
  const uint8_t RPM_DUTY_HW_TIMER_PRESCALER = 144; //36 DUTY and RPM meter share same timer T4, minimum meas. freq = 72000000/65536/36 = 30Hz, duty signal should be 100Hz, rpm from 30Hz
  const uint8_t FREQ_TO_RPM = 30;     //1Hz = 15rpm
  const uint16_t RPM_IDLE_MAX = 1200;
  const uint16_t RPM_MAX = 7000;
                                              
  const uint8_t DUTY_FAIL_LOW = 10;
  const uint8_t DUTY_FAIL_HIGH = 90;
  const uint8_t DUTY_WARN_LOW = 30;
  const uint8_t DUTY_WARN_HIGH = 70;
  const uint8_t DUTY_FREQ_LOW = 90;
  const uint8_t DUTY_FREQ_HIGH = 110;
  
  //inputs
  const uint16_t LAMBDA_INPUT = A0;   //lambda sensor voltage input pin (rich >= ~0.7V, lean <= ~0.2V)
  const uint16_t BATT_INPUT = A1;     //battery voltage input
  const uint16_t OVP_INPUT = A2;      //OVP voltage input
  const uint16_t ICV_INPUT = A3;      //ICV voltage input
  const uint16_t DUTY_INPUT =PB9;    //duty cycle input - these two needs to share same hw timer but different channel pair
  const uint16_t RPM_INPUT = PB7;     //rpm input - these two needs to share same hw timer but different channel pair (PB6 works also)

  //analog inputs calibration
  const uint16_t LAMBDA_CAL_IN = 629;
  const uint16_t LAMBDA_CAL_READ = 1870;
  const uint16_t VBATT_CAL_IN = 4520;
  const uint16_t VBATT_CAL_READ = 801;
  const uint16_t OVP_CAL_IN = 4520;   //not calibrated yet
  const uint16_t OVP_CAL_READ = 801;  //not calibrated yet
  const uint16_t ICV_CAL_IN = 4520;   //not calibrated yet
  const uint16_t ICV_CAL_READ = 801;  //not calibrated yet

  //outputs
  const uint16_t LED_LEAN2 = PB12;    //very lean mixture LED - on when LAMBDA_INPUT voltage <= V_LEAN2
  const uint16_t LED_LEAN1 = PB13;    //lean mixture LED - on when V_LEAN2 < LAMBDA_INPUT voltage <= V_LEAN1
  const uint16_t LED_RIGHT = PB14;    //right mixture LED - on when V_LEAN1 < LAMBDA_INPUT voltage < V_RICH1
  const uint16_t LED_RICH1 = PB15;    //rich mixture  LED - on when V_RICH1 <= LAMBDA_INPUT voltage < V_RICH2
  const uint16_t LED_RICH2 = PA8;     //very rich mixture LED - on when V_RICH2 <= LAMBDA_INPUT voltage
  const uint16_t LED_ONBOARD = PC13;  //LED on the bluepill board



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
uint16_t battery_raw, ovp_raw, icv_raw;
uint16_t lambda_voltage_avg, lambda_voltage, lambda_value, vref_value;
uint16_t battery_voltage, battery_voltage_uncal;
uint16_t ovp_voltage, ovp_voltage_uncal;
uint16_t icv_voltage, icv_voltage_uncal, icv_voltage_abs;
//uint32_t lambda_voltage_avg_sum ;
uint8_t i, t = 1;
unsigned long millis_pre, millis_post;

//history


//hw-timer-duty-cycle-meter
uint32_t channelRising, channelFalling;
volatile uint32_t FrequencyMeasured, DutycycleMeasured, LastPeriodCapture = 0, CurrentCapture, HighStateMeasured;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;
HardwareTimer *MyTim;
//rpm-meter
uint32_t rpm_channel;
volatile uint32_t rpm_Measured, rpm_LastCapture = 0, rpm_CurrentCapture;
volatile uint32_t rpm_rolloverCompareCount = 0;

//LCD pins
//            MOSI  MISO  SCLK
SPIClass SPI3(PB5, PB4, PB3);
const uint16_t TFT_CS = PA15;
const uint16_t TFT_RST = PB6;      //PB11; ??and PB6 as RST
const uint16_t TFT_DC = PB8;       //PB10; ??can we use MISO (PB4) for this as MISO is not used (no out from display to STM)?? - NOPE

//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7735 tft = Adafruit_ST7735(&SPI3, TFT_CS, TFT_DC, TFT_RST);

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void TIMINPUT_Capture_Rising_IT_callback(void)
{
  CurrentCapture = MyTim->getCaptureCompare(channelRising);
  /* frequency computation */
  if (CurrentCapture > LastPeriodCapture)
  {
    FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
    DutycycleMeasured = (HighStateMeasured * 100) / (CurrentCapture - LastPeriodCapture);
  }
  else if (CurrentCapture <= LastPeriodCapture)
  {
    /* 0x1000 is max overflow value */
    FrequencyMeasured = input_freq / (0x10000 + CurrentCapture - LastPeriodCapture);
    DutycycleMeasured = (HighStateMeasured * 100) / (0x10000 + CurrentCapture - LastPeriodCapture); 
  }

  DutycycleMeasured = 100 - DutycycleMeasured;  //we need to measure duration of "low level" => 100 - DutycycleMeasured
  LastPeriodCapture = CurrentCapture;
  rolloverCompareCount = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void Rollover_IT_callback(void)
{
  //this is called with about 30Hz frequency
  
  rolloverCompareCount++;

  if (rolloverCompareCount > 1)
  {
    FrequencyMeasured = 0;
    DutycycleMeasured = 0;
  }

  rpm_rolloverCompareCount++;

  if (rpm_rolloverCompareCount > 1)
  {
    rpm_Measured = 0;
  }
  
  //read all analog values
  //because if some of them are read in interrupt and some of them in main loop it results if wrong reading (usually 0)
  //most probably when there is one analogread() running, interrupt comes and another one is started
  //these all reagings takes < 1ms together
  vref_value = analogRead(AVREF);
  lambda();
  battery_raw = analogRead(BATT_INPUT);   //here we read just raw values
  ovp_raw =  analogRead(OVP_INPUT);       //we will calculate voltages when we need them (when is time to display them)
  icv_raw = analogRead(ICV_INPUT);        //it does not make sense to calculate these voltages every 1/30s and display them once

}

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void TIMINPUT_Capture_Falling_IT_callback(void)
{
  /* prepare DutyCycle computation */
  CurrentCapture = MyTim->getCaptureCompare(channelFalling);

  if (CurrentCapture > LastPeriodCapture)
  {
    HighStateMeasured = CurrentCapture - LastPeriodCapture;
  }
  else if (CurrentCapture <= LastPeriodCapture)
  {
    /* 0x1000 is max overflow value */
    HighStateMeasured = 0x10000 + CurrentCapture - LastPeriodCapture;
  }
}

void rpm_InputCapture_IT_callback(void)
{
  rpm_CurrentCapture = MyTim->getCaptureCompare(rpm_channel);
  /* frequency computation */
  if (rpm_CurrentCapture > rpm_LastCapture) {
    rpm_Measured = input_freq * FREQ_TO_RPM / (rpm_CurrentCapture - rpm_LastCapture);
  }
  else if (rpm_CurrentCapture <= rpm_LastCapture) {
    /* 0x1000 is max overflow value */
    rpm_Measured = input_freq * FREQ_TO_RPM / (0x10000 + rpm_CurrentCapture - rpm_LastCapture);
  }
  rpm_LastCapture = rpm_CurrentCapture;
  rpm_rolloverCompareCount = 0;
  
  digitalWrite(LED_ONBOARD, !digitalRead(LED_ONBOARD));   //flash the onboard LED to indicate we are alive
}

void hw_timer_rpm_duty_meter_init() {
  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(DUTY_INPUT), PinMap_PWM);
  //selected pins for DUTY_INPUT PB9, PB7 and RPM_INPUT use the same timer T4 therefore another timer instance not needed / possible
  //cant use PB8 as RPM input because duty cycle measurement uses T4C3 also, therefore PB7 used (PB6 also works)
  //TIM_TypeDef *rpm_Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);  
  channelRising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(DUTY_INPUT), PinMap_PWM));
  rpm_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(RPM_INPUT), PinMap_PWM));

  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling and channelRising3 is associated with channelRising4
  switch (channelRising) {
    case 1:
      channelFalling = 2;
      break;
    case 2:
      channelFalling = 1;
      break;
    case 3:
      channelFalling = 4;
      break;
    case 4:
      channelFalling = 3;
      break;
  }

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(Instance);

  // Configure rising edge detection to measure frequency
  MyTim->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, DUTY_INPUT);
  MyTim->setMode(rpm_channel, TIMER_INPUT_CAPTURE_RISING, RPM_INPUT); //or _FALLING if want to detect the other edge

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  //uint32_t PrescalerFactor = DUTY_HW_TIMER_PRESCALER;
  MyTim->setPrescaleFactor(RPM_DUTY_HW_TIMER_PRESCALER);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
  MyTim->attachInterrupt(channelFalling, TIMINPUT_Capture_Falling_IT_callback);
  MyTim->attachInterrupt(rpm_channel, rpm_InputCapture_IT_callback);
  MyTim->attachInterrupt(Rollover_IT_callback);

  MyTim->resume();

  // Compute this scale factor only once
  input_freq = MyTim->getTimerClkFreq() / MyTim->getPrescaleFactor();
}

void lambda() {
  lambda_value = analogRead(LAMBDA_INPUT);
  lambda_voltage = ((((uint32_t)lambda_value * V_REFI) / vref_value ) * LAMBDA_CAL_IN ) / LAMBDA_CAL_READ;
  //=> lambda_voltage * LAMBDA_CAL_READ / LAMBDA_CAL_IN * vref_value / V_REFI = lambda_value
  //lambda_voltage_avg_sum += lambda_voltage;
  lambda_voltage_avg = lambda_voltage;    //temp hack

  //control LEDz
  if (lambda_voltage <= V_LEAN2) {
    //very lean
    digitalWrite(LED_LEAN2, LOW);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if ((V_LEAN2 < lambda_voltage) and (lambda_voltage <= V_LEAN1)) {
    //lean
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, LOW);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if ((V_LEAN1 < lambda_voltage) and (lambda_voltage < V_RICH1)) {
    //right
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if ((V_RICH1 <= lambda_voltage) and (lambda_voltage < V_RICH2)) {
    //rich
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, LOW);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if (V_RICH2 <= lambda_voltage) {
    //very rich
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, LOW);
    
  }
}

void get_battery() {
  //vref_value needs to be known, otherwise vref_value = analogRead(AVREF); is needed
  battery_voltage_uncal = (((uint32_t)battery_raw * V_REFI) / vref_value);
  battery_voltage = ((uint32_t)battery_voltage_uncal * VBATT_CAL_IN) / VBATT_CAL_READ;
#ifdef SDEBUG
  Serial.printf("Vbatt = %umV\r\nVbatt_uncal = %umV\r\n", battery_voltage, battery_voltage_uncal);
#endif
}

void get_ovp() {
  //vref_value needs to be known, otherwise vref_value = analogRead(AVREF); is needed
  ovp_voltage_uncal = (((uint32_t)ovp_raw * V_REFI) / vref_value);
  ovp_voltage = ((uint32_t)ovp_voltage_uncal * OVP_CAL_IN) / OVP_CAL_READ;
#ifdef SDEBUG
  Serial.printf("OVP = %umV\r\nOVP_uncal = %umV\r\n", ovp_voltage, ovp_voltage_uncal);
#endif  
}

void get_icv() {
  //vref_value needs to be known, otherwise vref_value = analogRead(AVREF); is needed
  //ovp_voltage needs to be known
  icv_voltage_uncal = (((uint32_t)icv_raw * V_REFI) / vref_value);
  icv_voltage_abs = ((uint32_t)icv_voltage_uncal * ICV_CAL_IN) / ICV_CAL_READ;
  if (ovp_voltage > icv_voltage_abs) { 
    icv_voltage = ovp_voltage - icv_voltage_abs;
  } else {
    icv_voltage = 0;
  }
#ifdef SDEBUG
  Serial.printf("ICV = %umV\r\nICV_abs = %umV\r\nICV_uncal = %umV\r\n", icv_voltage, icv_voltage_abs, icv_voltage_uncal);
#endif    
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
  //tft.print(F("0.0V"));
  tft.printf("%u.%u%uV", ovp_voltage / 1000, (ovp_voltage / 100) % 10, (ovp_voltage / 10) % 10);
  if (ovp_voltage < 10000) tft.print(" ");
  //status
  tft.setCursor(VAL2_X, OVP_TXT_Y);
  if ((ovp_voltage + 500) >= battery_voltage) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  } else {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" FAIL "));
  }

  //rpm
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, RPM_TXT_Y);
  tft.printf("%-6u", rpm_Measured);
  tft.setCursor(VAL2_X, RPM_TXT_Y);
  if (rpm_Measured == 0) {
    tft.setTextColor(WARN_VAL_TXT_COLOR, WARN_VAL_BGR_COLOR );
    tft.print(F(" STOP "));
  } else if (rpm_Measured <= RPM_IDLE_MAX) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F(" IDLE "));
  } else if (rpm_Measured <= RPM_MAX) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  } else {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" HIGH "));
  }
  

  //duty
  tft.setTextColor(TXT_VAL_COLOR, BACKGROUND_COLOR);
  tft.setCursor(VAL1_X, DUTY_TXT_Y);
  tft.printf("%u%%", DutycycleMeasured);
  if (DutycycleMeasured < 100) tft.print(" ");
  if (DutycycleMeasured < 10) tft.print(" ");
  //status
  tft.setCursor(VAL2_X, DUTY_TXT_Y);
  if ((DutycycleMeasured > DUTY_FAIL_HIGH) or (DutycycleMeasured < DUTY_FAIL_LOW)) {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.printf(F(" FAIL ")); 
  } else if ((FrequencyMeasured > DUTY_FREQ_HIGH) or (FrequencyMeasured < DUTY_FREQ_LOW)) {
    //bad freq, should be 100Hz
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.printf(F(" FREQ "));
  } else if ((DutycycleMeasured >= DUTY_WARN_HIGH) or (DutycycleMeasured <= DUTY_WARN_LOW)) {
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
  if (icv_voltage < 10000) tft.print(" ");
  tft.setCursor(VAL2_X, ICV_TXT_Y);
  if (icv_voltage >= ICV_VOLTAGE_MIN) {
    tft.setTextColor(GOOD_VAL_TXT_COLOR, GOOD_VAL_BGR_COLOR );
    tft.print(F("  OK  "));
  } else {
    tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
    tft.print(F(" FAIL "));
  } 
  //tft.setTextColor(FAIL_VAL_TXT_COLOR, FAIL_VAL_BGR_COLOR );
  //tft.setCursor(VAL2_X, ICV_TXT_Y);
  //tft.print(F(" FAIL "));  

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

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  //while (!Serial);                  //need to be removed, otherwise will not run WO PC :D
  Serial.print(F("\r\n* * * BGR Lambda Indicator v0.01 * * *\r\n\r\nConfiguration\r\n=============\r\nLED update delay: "));
  Serial.print(CYCLE_DELAY);
//  Serial.print(F("ms\r\nSamples avg for serial console: "));
//  Serial.print(N_AVG);
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
  
  //inputs
  pinMode(LAMBDA_INPUT, INPUT_ANALOG);
  pinMode(BATT_INPUT, INPUT_ANALOG);
  pinMode(OVP_INPUT, INPUT_ANALOG);
  pinMode(ICV_INPUT, INPUT_ANALOG);
  pinMode(DUTY_INPUT, INPUT);
  pinMode(RPM_INPUT, INPUT_PULLUP);
  pinMode(PA9, OUTPUT);               //testing signal
  analogWriteFrequency(100);          //100Hz
  analogWriteResolution(8);          //we have 16bit timers so use them
  analogWrite(PA9, 50); //16% ------- 6553500 = 100%
  //analogWrite(PA9, 11797); //18%

  //delay(10000);

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

  vref_value = analogRead(AVREF);
  
  //init hw-timer-duty-cycle-meter
  hw_timer_rpm_duty_meter_init();
  pinMode(RPM_INPUT, INPUT_PULLUP);

  drawbasicscreen();
}

void loop() {
//  lambda_voltage_avg_sum = 0;
  //
  
  //lambda_voltage_avg = lambda_voltage_avg_sum / N_AVG;
  //Serial.printf(" - lambda voltage avg: %umV\r\n", lambda_voltage_avg);
  get_battery();
  get_ovp();
  get_icv();
  
  showvalues();
  
  delay(CYCLE_DELAY);
  
  //Serial.print((String)"Frequency = " + FrequencyMeasured);
  //Serial.println((String)"    Dutycycle = " + DutycycleMeasured);
  //Serial.println((String)"RPM Frequency = " + rpm_Measured);
  
}
