//MECEDES-PILL - W124-DIAG
//by BugerDread

//options
//board: buepillF103CB (or C8 with 128k)
//USART support: disabled (no Serial support)
//USB support: CDC (generic Serial supersede USART)
//C Runtime: Newlib Nano + float printf
//Optimize: -O2 + LTO

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
  const uint32_t  V_LEAN = 100;               //very lean mixture voltage [mV]
  const uint32_t  V_RICH = 700;               //very rich mixture voltage [mV]
  const uint32_t  CYCLE_DELAY = 100;          //delay for each round [ms]
  const uint32_t  V_BATT_LOW = 11000;         //battery voltage [mV] bottom level (for chart)
  const uint32_t  V_BATT_HIGH = 14600;        //battery voltage [mV] top level (for chart)
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

  const double    PID_KP_DEFAULT = 1.0;
  const double    PID_KI_DEFAULT = 1.0;
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
  bool engine_running = false;
  bool warmup_phase = true;
  unsigned long engine_last_running = 0;
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

void pid_compute()
{     
      if (!pid_on) return;    //exit if pid is not on
  
      //do we need to filter the rpm_measured to make sure it is sane?
      //rpm_measured == 0 = motor is not spinning or we cant measure such low rpms
      //maybe we will count failed passes and disable ICV control if sane signal not received for a while? - future
      uint32_t input = rpm_measured;   //rpm_measured is a volatile variable, we dont want it to change during computation

      if (input == 0) {                     //if we lost the rpm signal, turn off the ICV
        pid_output = ICV_PWM_DEFAULT;
        analogWrite(ICV_PWM_OUT, pid_output);
        return;
      }
      
      /*Compute all the working error variables*/
      int32_t error = pid_setpoint - input;
      
      pid_iterm += (pid_ki_internal * error);
      if(pid_iterm > pid_out_max) pid_iterm = pid_out_max;
      else if (pid_iterm < pid_out_min) pid_iterm = pid_out_min;
      
      int32_t dInput = input - pid_lastinput;
 
      /*Compute PID Output*/
      double output = pid_kp_internal * error + pid_iterm - pid_kd_internal * dInput;
      if(output > pid_out_max) output = pid_out_max;
      else if(output < pid_out_min) output = pid_out_min;
      //now we have the result computed
      pid_output = output;

      //send output to output :D
      analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
 
      /*Remember some variables for next time*/
      pid_lastinput = input;

//      if (++pid_debug_cnt >= 10) {
//        Serial.printf(F("RPM: %-5u ERR: %-5d OUT: %u\r\n"), rpm_measured, error, pid_output);
//        pid_debug_cnt = 0;
//      }
}

void pid_set_tunings(double Kp, double Ki, double Kd)
{
   pid_kp_internal = Kp;
   pid_ki_internal = Ki * pid_sample_time_s;
   pid_kd_internal = Kd / pid_sample_time_s;

   Serial.printf(F("PID params:\r\n"
                   "pid_sample_time = %.2fms / %uHz\r\n"
                   "pid_kp = %.2f\r\n"
                   "pid_ki = %.2f\r\n"
                   "pid_kd = %.2f\r\n"),
                   pid_sample_time_s * 1000, PRM_DUTY_TIMER_IFREQ / 65536,
                   pid_kp, pid_ki, pid_kd);
}

void duty_it_capture_rising(void)
{
  uint32_t duty = 0;
  
  duty_capture = rpm_duty_timer->getCaptureCompare(duty_ch_rising);
  /* frequency computation */
  if(duty_capture > duty_last_capture)
  {
    duty_freq_measured = PRM_DUTY_TIMER_IFREQ / (duty_capture - duty_last_capture);
    duty = (duty_highstate * 100) / (duty_capture - duty_last_capture);
  }
  else if(duty_capture <= duty_last_capture)
  {
    /* 0x1000 is max overflow value */
    duty_freq_measured = PRM_DUTY_TIMER_IFREQ / (0x10000 + duty_capture - duty_last_capture);
    duty = (duty_highstate * 100) / (0x10000 + duty_capture - duty_last_capture); 
  }

  if (duty <= 100) {  //check if result is sane, it cannot be < 0 because its unsigned integer
    duty_cycle_measured = 100 - duty;  //we need to measure duration of "low level" => 100 - duty_cycle_measured
  } else {
    duty_cycle_measured = 0;  //we got invalid result, override it with 0
  }
  
  duty_last_capture = duty_capture;
  duty_rollover_count = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rpm_duty_timer_it_rollover(void)
{
  //this is called with frequency PRM_DUTY_TIMER_IFREQ / 65536
  
  duty_rollover_count++;

  if(duty_rollover_count > 1)
  {
    duty_freq_measured = 0;
    duty_cycle_measured = 0;
  }

  rpm_rollover_count++;

  if(rpm_rollover_count > 1)   //toto ceknout podle me mam byt > 1 [ses kokot, >= 2 a >1 je to same]
  {
    rpm_measured = 0;                 //mozna tady taky prumerovat nebo nejak ignorovat par chybejicich pulzu
    rpm_rollover_count = 2;     //nene tady nic neprumerovat, v presuseni od inputu se prumeruji jen platne hodnoty, vynechane pusly se ignoruji
    //v PID procesu pak dame neco jako ze hodnota kdyz rpm_measured = 0 tak bud je to chyba ze nam upadl signal nebo nam stoji motor
    //pokud stoji nenadelame uz nic, pokud upadl signal tak zbesilym pridanim zbytecne proturuje motor
    //=> je nesmysl vytocit ICV na max, spis chvili ho drzet kde zrovna bylo (3s?) a pokud stale nic tak uplne vypnout do failsafe
    //podobne kdyz bude rpm_measured = -1 bude to znamenat ze nam jdou na vstup nesmysly (ruseni) a vychazi rpm > RPM_MAX
  }
  
  //read all analog values
  //because if some of them are read in interrupt and some of them in main loop it results if wrong reading (usually 0)
  //most probably when there is one analogread() running, interrupt comes and another one is started
  //these all reagings takes < 1ms together
  
  vref_value = analogRead(AVREF);
  lambda();
  battery_raw = analogRead(BATT_INPUT);   //here we read just raw values
  ovp_raw =  analogRead(OVP_INPUT);       //we will calculate voltages when we need them (when is time to display them)

  pid_compute(); //process pid
}

/**
    @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
*/
void duty_it_capture_falling(void)
{
  /* prepare DutyCycle computation */
  duty_capture = rpm_duty_timer->getCaptureCompare(duty_ch_falling);

  if(duty_capture > duty_last_capture)
  {
    duty_highstate = duty_capture - duty_last_capture;
  }
  else if(duty_capture <= duty_last_capture)
  {
    /* 0x1000 is max overflow value */
    duty_highstate = 0x10000 + duty_capture - duty_last_capture;
  }
}

void rpm_it_capture(void)
{
  rpm_capture = rpm_duty_timer->getCaptureCompare(rpm_channel);
  /* frequency computation */
  uint32_t rpm = 0;

  //if(rpm_rollover_count < 2) {    //calculate rpm only when _LastCapture is valid (or <=1)

    if(rpm_rollover_count == 0) {   //!!!rpm measurement from example was buggy causing freuqncies 15-30Hz to translate to 0-10000rpm
      //rpm_measured 
      rpm = PRM_DUTY_TIMER_IFREQ * FREQ_TO_RPM / (rpm_capture - rpm_last_capture);
    } else if ((rpm_capture <= rpm_last_capture) and (rpm_rollover_count == 1)){   //the second condition is no more useless
      /* 0x1000 is max overflow value */
      //rpm_measured 
      rpm = PRM_DUTY_TIMER_IFREQ * FREQ_TO_RPM / (0x10000 + rpm_capture - rpm_last_capture);
    }

    if((rpm <= RPM_MAX) and (rpm > 0)) {                 //check if the rpm we got makes sense - ignore if its way high (dont even update rpm_measured)
      //calculate average from 4 last good measurements and rotate the history
      rpm_measured = (rpm + rpm_history[0] ) / 2; //+ rpm_history[1] + rpm_history[2]) / 4; //average from last 4 values, rpm is uint32_t so the result of sum will fit - only two
      //rpm_history[2] = rpm_history[1];
      //rpm_history[1] = rpm_history[0];
      rpm_history[0] = rpm;
    } 
  //} 
    
  rpm_last_capture = rpm_capture;

  //Serial.printf("\r\n%u\t%u\t%u\t%u\t%u\t%u",rpm_rollover_count, rpm_measured, rpm, rpm_history[0], rpm_history[1], rpm_history[2]);
  
  rpm_rollover_count = 0;
  
//  digitalWrite(LED_ONBOARD, !digitalRead(LED_ONBOARD));   //flip the onboard LED to indicate we are alive
   
}

void hw_timer_rpm_duty_meter_init() {
  //clear rpm history array
  rpm_history[0]=0;
  rpm_history[1]=0;
  rpm_history[2]=0;
  
  // Automatically retrieve TIM instance and duty_ch_rising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(DUTY_INPUT), PinMap_PWM);
  //selected pins for DUTY_INPUT PB9, PB7 and RPM_INPUT use the same timer T4 therefore another timer instance not needed / possible
  //cant use PB8 as RPM input because duty cycle measurement uses T4C3 also, therefore PB7 used (PB6 also works)
  //TIM_TypeDef *rpm_Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);  
  duty_ch_rising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(DUTY_INPUT), PinMap_PWM));
  rpm_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(RPM_INPUT), PinMap_PWM));

  // duty_ch_risings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // duty_ch_rising1 is associated to duty_ch_falling and duty_ch_rising3 is associated with duty_ch_rising4
  switch (duty_ch_rising) {
    case 1:
      duty_ch_falling = 2;
      break;
    case 2:
      duty_ch_falling = 1;
      break;
    case 3:
      duty_ch_falling = 4;
      break;
    case 4:
      duty_ch_falling = 3;
      break;
  }

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  rpm_duty_timer = new HardwareTimer(Instance);

  // Configure rising edge detection to measure frequency
  rpm_duty_timer->setMode(duty_ch_rising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, DUTY_INPUT);
  rpm_duty_timer->setMode(rpm_channel, TIMER_INPUT_CAPTURE_FALLING, RPM_INPUT); //or _FALLING if want to detect the other edge

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  //uint32_t PrescalerFactor = DUTY_HW_TIMER_PRESCALER;
  rpm_duty_timer->setPrescaleFactor(RPM_DUTY_HW_TIMER_PRESCALER);
  rpm_duty_timer->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  rpm_duty_timer->attachInterrupt(duty_ch_rising, duty_it_capture_rising);
  rpm_duty_timer->attachInterrupt(duty_ch_falling, duty_it_capture_falling);
  rpm_duty_timer->attachInterrupt(rpm_channel, rpm_it_capture);
  rpm_duty_timer->attachInterrupt(rpm_duty_timer_it_rollover);

  rpm_duty_timer->resume();
}

void lambda() {
  //lambda_value = analogRead(LAMBDA_INPUT);
  //lambda_voltage = ((uint32_t)lambda_value * V_REFI) / vref_value;                              //uncal
  //Serial.printf("lambda uncal: %u\tvalue: %u\r\n", lambda_voltage, lambda_value);  
  //lambda_voltage = ((uint32_t)lambda_voltage * LAMBDA_CAL_IN) / LAMBDA_CAL_READ;                //cal
  lambda_voltage = ((uint32_t)analogRead(LAMBDA_INPUT) * V_REFI * LAMBDA_CAL_IN) / (vref_value * LAMBDA_CAL_READ);

  //control LEDz
  if(lambda_voltage <= V_LEAN) {
    //lean
    digitalWrite(LED_LEAN, LOW);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH, HIGH);
    
  } else if((lambda_voltage > V_LEAN) and (lambda_voltage < V_RICH)) {
    //right
    digitalWrite(LED_LEAN, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_RICH, HIGH);
    
  } else if(lambda_voltage >= V_RICH) {
    //rich
    digitalWrite(LED_LEAN, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH, LOW);
  }
}

void get_battery() {
  //vref_value needs to be known, otherwise vref_value = analogRead(AVREF); is needed
  battery_voltage_uncal = ((battery_raw * V_REFI) / vref_value);
  battery_voltage = (battery_voltage_uncal * VBATT_CAL_IN) / VBATT_CAL_READ;
#ifdef SDEBUG
  Serial.printf("Vbatt = %umV\r\nVbatt_uncal = %umV\r\n", battery_voltage, battery_voltage_uncal);
#endif
}

void get_ovp() {
  //vref_value needs to be known, otherwise vref_value = analogRead(AVREF); is needed
  ovp_voltage_uncal = ((ovp_raw * V_REFI) / vref_value);
  ovp_voltage = (ovp_voltage_uncal * OVP_CAL_IN) / OVP_CAL_READ;
#ifdef SDEBUG
  Serial.printf("OVP = %umV\r\nOVP_uncal = %umV\r\n", ovp_voltage, ovp_voltage_uncal);
#endif  
}

void setup() {
  Serial.begin(115200);
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
  pinMode(ICV_PWM_OUT, OUTPUT);               //ICV PWM signal
  analogWrite(ICV_PWM_OUT, ICV_PWM_DEFAULT);           

  //init LED pins and tur on front ledz
  pinMode(LED_LEAN, OUTPUT);
  digitalWrite(LED_LEAN, LOW);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_RIGHT, LOW);
  pinMode(LED_RICH, OUTPUT);
  digitalWrite(LED_RICH, LOW);
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, HIGH);

  lcd_init();
  
  vref_value = analogRead(AVREF);

  eeload();                                     //get configuration from eeprom
  pid_set_tunings(pid_kp, pid_ki, pid_kd); 
  pid_setpoint = rpm_warmup;
  if (!pid_on) {
    pid_output = icv_pwm_man;
    analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
  }
  
  //init hw-timer-duty-cycle-meter
  hw_timer_rpm_duty_meter_init();

  delay (3000); //delay to show the logo on the screen, regulation already running :D
  
  //Serial.printf(F("sampletime: %ums = %uHz\r\n"), (uint32_t)(pid_sample_time_s * 1000), PRM_DUTY_TIMER_IFREQ / 65536);
  drawbasicscreen();
}

unsigned long loop_last_millis = 0;

void check_engine_running() {
  //find out if engine is running or not
  //engine is not running if there is no rpm signal (rpm = 0) for ENGINE_TIMEOUT [ms]
  if (rpm_measured == 0) {
    if (engine_running and ((millis() - engine_last_running) > ENGINE_TIMEOUT)) {
      //engine has stopped
      engine_running = false;
      Serial.println(F("Engine stopped"));
    }
  } else {
    //engine running
    if (!engine_running) {
      //engine has been just started
      engine_started = millis();
      engine_running = true;
      warmup_phase = true;    //commnet this out to require reboot to warm up again, makes no difference if powered from OVP
      pid_setpoint = rpm_warmup;
      Serial.println(F("Engine started"));
    }
    
    engine_last_running = millis();
    if ((engine_last_running - engine_started) < warmup_time) {                          //engine_last_running used because we assgned millis() to it one line above
      //warmup phase
      warmup_remaining = warmup_time - (engine_last_running - engine_started);
    } else {
      warmup_remaining = 0;
    }

    if ((warmup_phase) and (warmup_remaining == 0)) {
      //warmup is over
      warmup_phase = false;
      pid_setpoint = rpm_idle;
      Serial.println(F("Warmup over"));
    }
  }
}

void loop() {
  if ((millis() - loop_last_millis) > CYCLE_DELAY) {
    loop_last_millis = millis();
    get_battery();
    get_ovp();
    showvalues();
  }

  checkserial();
  check_engine_running();
}
