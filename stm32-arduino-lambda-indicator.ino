//MECEDES-PILL - W124-DIAG
//by BugerDread

//STM32 support for arduino
//URL to add STM32 support: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
//Install: STM32 MCU based boards by STMicroelectronics v 2.1.0

//other libs: 
// - Adafruit_ST7735 v 1.7.4
// - Adafruit_GFX v 1.10.12

//options
//board: buepillF103CB (or C8 with 128k)
//USART support: disabled (no Serial support)
//USB support: CDC (generic Serial supersede USART)
//USB speed: low / full
//C Runtime: Newlib Nano + float printf
//Optimize: -O2 + LTO
//Upload method: Maple DFU 2.0

// !!!if some variable is modified by ISR it MUST be declared as volatile

//needed by hardware-timer-based duty-cycle measurement
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

//#define SDEBUG

//constants
//it does not make a sense to make integers smaller than 32b
  //general
  const uint32_t  PULSES_PER_REV = 2;         //number of pulses per revolution from ZEL
  const uint32_t  RPM_METER_MIN = 300;       //minimum PRM we want to be able to measure
  const uint32_t  V_REFI = 1208;              //STM32F103 internal reference voltage [mV]
  const uint32_t  V_LEAN = 250;               //very lean mixture voltage [mV]
  const uint32_t  V_RICH = 650;               //very rich mixture voltage [mV]
  const uint32_t  CYCLE_DELAY = 100;          //delay for each round [ms]
  const uint32_t  V_BATT_LOW = 11000;         //battery voltage [mV] bottom level (for chart)
  const uint32_t  V_BATT_HIGH = 15000;        //battery voltage [mV] top level (for chart)
  const uint32_t  ICV_PWM_BITS = 8;           //number of bits of ICV PWM
  const uint32_t  ICV_PWM_FREQ = 100;         //ICV PWM frequency
  const uint32_t  ICV_PWM_DEFAULT = 0;      //initial value of ICV PWM
  const uint32_t  ICV_PWM_MIN_DEFAULT = 78;   //cca 78 - minimum ICV PWM out during regulation (to skip initial 20% open wo power) = about 4V
  const uint32_t  ICV_PWM_MAX_DEFAULT = 200;  //cca 200 - maximum ICV PWM out during regulation (usually full range) = about 8V

  const uint32_t  RPM_IDLE_DEFAULT = 750;
  const uint32_t  RPM_WARMUP_DEFAULT = 1250;
  const uint32_t  WARMUP_TIME_DEFAULT = 60000;        //default warmup time [ms] during that rpm = rpm_warmup
  const uint32_t  WARMUP_TIME_MAX = 300000;   //maximum warmup time (300000 = 5min)
  const uint32_t  ENGINE_TIMEOUT = 5000;      //when rpm=0 for this time then engine is considered stopped
  const uint32_t  RPM_MAX = 10000;
  const uint32_t  RPM_MIN = 500;

  const uint32_t  PID_BOOST_RPM_DEFAULT = 0;      //boost disabled by default
  const double    PID_BOOST_KP_DEFAULT = 0;       //boost disabled by default

  const double    PID_KP_DEFAULT = 0.02;
  const double    PID_KI_DEFAULT = 0.02;
  const double    PID_KD_DEFAULT = 0.0;
                                              
  const uint32_t  DUTY_LOW = 10;
  const uint32_t  DUTY_HIGH = 90;
  
  //inputs
  const uint32_t  LAMBDA_INPUT = A3;          //lambda sensor voltage input pin (rich >= ~0.7V, lean <= ~0.2V)
  const uint32_t  BATT_INPUT = A0;            //battery voltage input, now from OVP
  const uint32_t  OVP_INPUT = A1;             //this was OVP voltage input, now its AUX in voltage as we power this from OVP, will see how we use it, maybe LPG tank meter
  const uint32_t  DUTY_INPUT = PB9;           //duty cycle input - these two needs to share same hw timer but different channel pair
  const uint32_t  RPM_INPUT = PB7;            //rpm input - these two needs to share same hw timer but different channel pair (PB6 works also)

  //analog inputs calibration
  const uint32_t  LAMBDA_CAL_IN = 1;          //629;
  const uint32_t  LAMBDA_CAL_READ = 1;        //1870;
  const uint32_t  VBATT_CAL_IN = 12370;       //real input voltage (read by multimeter)
  const uint32_t  VBATT_CAL_READ = 2342;      //uncal voltage (shown in serial debug)
  const uint32_t  OVP_CAL_IN = 12370;         //
  const uint32_t  OVP_CAL_READ = 2319;        //

  //outputs
  const uint32_t  LED_LEAN = PB14;            //very lean mixture LED - on when LAMBDA_INPUT voltage <= V_LEAN
  const uint32_t  LED_RIGHT = PB13;           //right mixture LED - on when V_LEAN < LAMBDA_INPUT voltage < V_RICH
  const uint32_t  LED_RICH = PB12;            //very rich mixture LED - on when V_RICH <= LAMBDA_INPUT voltage
  const uint32_t  LED_ONBOARD = PC13;         //LED on the bluepill board
  const uint32_t  ICV_PWM_OUT = PA9;          //ICV PWM output (via FET)

//global variables
//  bool engine_running = false;
  bool warmup_phase = true;
//  unsigned long engine_last_running = 0;
  unsigned long engine_started = 0;
  uint32_t icv_pwm_man = ICV_PWM_DEFAULT;
  uint32_t warmup_time = WARMUP_TIME_DEFAULT;
  uint32_t warmup_remaining = WARMUP_TIME_DEFAULT;
  uint32_t rpm_idle = RPM_IDLE_DEFAULT;
  uint32_t rpm_warmup = RPM_WARMUP_DEFAULT;
  uint32_t battery_voltage, battery_voltage_uncal;
  uint32_t ovp_voltage, ovp_voltage_uncal;
  
//global variables used in interrupts (needs to be volatile)
  volatile uint32_t battery_raw, ovp_raw, vref_value, lambda_voltage;

//rpm-meter
  const uint32_t FREQ_TO_RPM = 60 / PULSES_PER_REV;                       //1Hz = 15rpm if there are 4 pusles per revolution, 1Hz = 30rpm with 2 pulses per revolution
  uint32_t rpm_channel;
  volatile uint32_t rpm_measured, rpm_last_capture = 0, rpm_capture;
  volatile uint32_t rpm_rollover_count = 0;
  uint32_t rpm_history[3]; //history

//hw-timer for rpm and duty-cycle meter
//rpm and duty-cycle meter shares the same timer (T4)
  const uint32_t RPM_DUTY_HW_TIMER_PRESCALER = (SystemCoreClock * FREQ_TO_RPM) / (65536 * RPM_METER_MIN);                                         //36 minimum meas. freq = 72000000/65536/36 = 30Hz, duty signal should be 100Hz, rpm from 450 for 15rpm per Hz
                                                                                           //= 72 000 000 / 65536 / (minimum_rpm / FREQ_TO_RPM) =  1099.6 * FREQ_TO_RPM / minimum_rpm
                                                                                           //= 1099.6*30/300 = 110 (30rpm per Hz and 300rpm minimum)
  const uint32_t PRM_DUTY_TIMER_IFREQ = SystemCoreClock / RPM_DUTY_HW_TIMER_PRESCALER;    //input freq of the timer
  uint32_t duty_ch_rising, duty_ch_falling;
  volatile uint32_t duty_freq_measured, duty_cycle_measured, duty_last_capture = 0, duty_capture, duty_highstate;
  volatile uint32_t duty_rollover_count = 0;
  HardwareTimer *rpm_duty_timer;                                                          //our HW timer

//PID
  //const uint16_t pid_sample_time = 65536000 / PRM_DUTY_TIMER_IFREQ;     //time period [in ms] pid proces is called = time period of rpm_duty_timer overflow = 1 / (PRM_DUTY_TIMER_IFREQ / 65536) * 1000 = 65536000 / PRM_DUTY_TIMER_IFREQ;
  const double pid_sample_time_s = (double)65536 / PRM_DUTY_TIMER_IFREQ;   //time period [in s] pid proces is called = time period of rpm_duty_timer overflow = 1 / (PRM_DUTY_TIMER_IFREQ / 65536) = 65536 / PRM_DUTY_TIMER_IFREQ;
  volatile uint32_t pid_output;      //this variable needs to be able to hold values in range pid_out_min .. pid_out_max
  volatile uint32_t pid_debug_cnt = 0;
  uint32_t pid_setpoint = RPM_WARMUP_DEFAULT;
  double pid_iterm = 0;
  uint32_t pid_lastinput = 0;
  double pid_kp = PID_KP_DEFAULT;
  double pid_ki = PID_KI_DEFAULT;
  double pid_kd = PID_KD_DEFAULT;
  double pid_kp_internal, pid_ki_internal, pid_kd_internal; 
  uint32_t pid_out_min = ICV_PWM_MIN_DEFAULT;
  uint32_t pid_out_max = ICV_PWM_MAX_DEFAULT;
  bool pid_on = true;
  uint32_t pid_boost_rpm = PID_BOOST_RPM_DEFAULT;
  double pid_boost_kp = PID_BOOST_KP_DEFAULT;

void setup() {
  Serial.begin(115200);
  //while(!Serial);   //remove or comment out after debugging, will not run wo PC connected :D
  Serial.print(F("\r\n\r\n* * * BGR Lambda Indicator v0.01 * * *\r\n\r\n"));

  //ADC will use 12bit accuracy
  analogReadResolution(12);
  
  //inputs
  pinMode(LAMBDA_INPUT, INPUT_ANALOG);
  pinMode(BATT_INPUT, INPUT_ANALOG);
  pinMode(OVP_INPUT, INPUT_ANALOG);
  pinMode(DUTY_INPUT, INPUT);
  pinMode(RPM_INPUT, INPUT);
  analogWriteFrequency(ICV_PWM_FREQ);          //100Hz
  analogWriteResolution(ICV_PWM_BITS);          //we have 16bit timers so use them or not?

  //ICV PWM OUTPUT
  pinMode(ICV_PWM_OUT, OUTPUT);               //ICV PWM signal
  analogWrite(ICV_PWM_OUT, ICV_PWM_DEFAULT);              

  //init LED pins and tur on front ledz
  pinMode(LED_LEAN, OUTPUT);
  digitalWrite(LED_LEAN, HIGH);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_RIGHT, HIGH);
  pinMode(LED_RICH, OUTPUT);
  digitalWrite(LED_RICH, HIGH);
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, HIGH);

  vref_value = analogRead(AVREF);

  eeload();                                     //get configuration from eeprom
  pid_set_tunings(pid_kp, pid_ki, pid_kd); 
  pid_setpoint = rpm_warmup;
  warmup_remaining = warmup_time;
  
  if (!pid_on) {                                //manual od PID control?
    pid_output = icv_pwm_man;                   //manual - lets set the output to pwm_man
  } else {                                      //PID control
    pid_iterm = pid_out_min;                    //set the PID Iterm and output to minimum (if STM restarts by bug this cause car will not start to accel by itself)
    pid_output = pid_out_min;
  }
  analogWrite(ICV_PWM_OUT, pid_output);       //send output to ICV
  
  //init hw-timer-duty-cycle-meter
  hw_timer_rpm_duty_meter_init();

  lcd_init();
  delay (1000); //delay to show the logo on the screen, regulation already running :D
  
  //Serial.printf(F("sampletime: %ums = %uHz\r\n"), (uint32_t)(pid_sample_time_s * 1000), PRM_DUTY_TIMER_IFREQ / 65536);
  drawbasicscreen();
}

unsigned long loop_last_millis = 0;

void loop() {
  if ((millis() - loop_last_millis) > CYCLE_DELAY) {
    loop_last_millis = millis();
    get_battery();
    get_ovp();
    showvalues();
  }

  check_serial();
  check_warmup();
}
