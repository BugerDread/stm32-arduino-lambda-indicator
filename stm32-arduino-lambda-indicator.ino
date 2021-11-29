//MECEDES-PILL - W124-DIAG
//by BugerDread

//options
//board: buepillF103CB (or C8 with 128k)
//USART support: disabled (no Serial support)
//USB support: CDC (generic Serial supersede USART)

// !!!if some variable is modified by ISR it MUST be declared as volatile

//needed by hardware-timer-based duty-cycle measurement
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

//#define SDEBUG

//constants
  //general
  const uint16_t V_REFI = 1208;       //STM32F103 internal reference voltage [mV]
  const uint16_t V_LEAN2 = 100;       //very lean mixture voltage [mV]
  const uint16_t V_LEAN1 = 200;       //lean mixture voltage [mV]
  const uint16_t V_RICH1 = 700;       //rich mixture voltage [mV]
  const uint16_t V_RICH2 = 800;       //very rich mixture voltage [mV]
  const uint16_t CYCLE_DELAY = 1000;    //delay for each round [ms]
  const uint16_t V_BATT_FAIL = 11000; //voltage [mV] below that battery is FAILED
  const uint16_t V_BATT_LOW = 12500; //voltage [mV] below that battery is LOW
  const uint16_t V_BATT_HIGH = 14600; //voltage [mV] below that battery is HIGH
  const uint16_t ICV_VOLTAGE_MIN = 3900;  //minimum ICV voltage, if lower error is shown

  const uint16_t RPM_IDLE_MAX = 1200;
  const uint16_t RPM_MAX = 10000;
                                              
  const uint8_t DUTY_FAIL_LOW = 10;
  const uint8_t DUTY_FAIL_HIGH = 90;
  const uint8_t DUTY_WARN_LOW = 30;
  const uint8_t DUTY_WARN_HIGH = 70;
  const uint8_t DUTY_FREQ_LOW = 90;
  const uint8_t DUTY_FREQ_HIGH = 110;
  
  //inputs
  const uint16_t LAMBDA_INPUT = A3;   //lambda sensor voltage input pin (rich >= ~0.7V, lean <= ~0.2V)
  const uint16_t BATT_INPUT = A0;     //battery voltage input
  const uint16_t OVP_INPUT = A1;      //OVP voltage input
  const uint16_t ICV_INPUT = A2;      //ICV voltage input
  const uint16_t DUTY_INPUT = PB9;    //duty cycle input - these two needs to share same hw timer but different channel pair
  const uint16_t RPM_INPUT = PB7;     //rpm input - these two needs to share same hw timer but different channel pair (PB6 works also)

  //analog inputs calibration
  const uint16_t LAMBDA_CAL_IN = 629;
  const uint16_t LAMBDA_CAL_READ = 1870;
  const uint16_t VBATT_CAL_IN = 12090;  //real input voltage (read by multimeter)
  const uint16_t VBATT_CAL_READ = 2288; //uncal voltage (shown in serial debug)
  const uint16_t OVP_CAL_IN = 11670;   //
  const uint16_t OVP_CAL_READ = 2194;  //
  const uint16_t ICV_CAL_IN = 4520;   //not calibrated yet
  const uint16_t ICV_CAL_READ = 801;  //not calibrated yet

  //outputs
  const uint16_t LED_LEAN2 = PA8;    //very lean mixture LED - on when LAMBDA_INPUT voltage <= V_LEAN2
  const uint16_t LED_LEAN1 = PB15;    //lean mixture LED - on when V_LEAN2 < LAMBDA_INPUT voltage <= V_LEAN1
  const uint16_t LED_RIGHT = PB14;    //right mixture LED - on when V_LEAN1 < LAMBDA_INPUT voltage < V_RICH1
  const uint16_t LED_RICH1 = PB13;    //rich mixture  LED - on when V_RICH1 <= LAMBDA_INPUT voltage < V_RICH2
  const uint16_t LED_RICH2 = PB12;     //very rich mixture LED - on when V_RICH2 <= LAMBDA_INPUT voltage
  const uint16_t LED_ONBOARD = PC13;  //LED on the bluepill board
  const uint16_t ICV_PWM_OUT = PA9;   //ICV PWM output (via FET)

//global variables
  uint16_t battery_voltage, battery_voltage_uncal;
  uint16_t ovp_voltage, ovp_voltage_uncal;
  uint16_t icv_voltage, icv_voltage_uncal, icv_voltage_abs;
  
//global variables used in interrupts (needs to be volatile)
  volatile uint16_t battery_raw, ovp_raw, icv_raw, vref_value, lambda_voltage;

//hw-timer for rpm and duty-cycle meter
//rpm and duty-cycle meter shares the same timer (T4)
  const uint8_t RPM_DUTY_HW_TIMER_PRESCALER = 36;                                         //36 minimum meas. freq = 72000000/65536/36 = 30Hz, duty signal should be 100Hz, rpm from 450
  const uint32_t PRM_DUTY_TIMER_IFREQ = SystemCoreClock / RPM_DUTY_HW_TIMER_PRESCALER;    //input freq of the timer
  uint32_t duty_ch_rising, duty_ch_falling;
  volatile uint32_t duty_freq_measured, duty_cycle_measured, duty_last_capture = 0, duty_capture, duty_highstate;
  volatile uint32_t duty_rollover_count = 0;
  HardwareTimer *rpm_duty_timer;                                                          //our HW timer
  
//rpm-meter
  const uint8_t FREQ_TO_RPM = 15;                       //1Hz = 15rpm if there are 4 pusles per rpm
  uint32_t rpm_channel;
  volatile uint32_t rpm_measured, rpm_last_capture = 0, rpm_capture;
  volatile uint32_t rpm_rollover_count = 0;
  uint16_t rpm_history[3]; //history

//PID
  const uint16_t pid_sample_time = 65536000 / PRM_DUTY_TIMER_IFREQ; //time period [in ms] pid proces is called = time period of rpm_duty_timer overflow = 1 / (PRM_DUTY_TIMER_IFREQ / 65536) * 1000 = 65536000 / PRM_DUTY_TIMER_IFREQ;
  const double pid_out_min = 0;    //needs to be set to ICV fully closed
  const double pid_out_max = 255;
  volatile double pid_output;
  double pid_setpoint = 600;
  double pid_iterm = 0;
  double pid_lastinput = 0;
  double pid_kp = 0;
  double pid_ki = 0;
  double pid_kd = 0;

void pid_compute()
{     
      //we need to filter the rpm_measured to make sure it is sane:
      //rpm_measured == 0 = motor is not spinning or we cant measure such low rpms
      //maybe we will count failed passes and disable ICV control if sane signal not received for a while? - future
      double pid_input = rpm_measured;   //rpm_measured is a volatile variable, we dont want it to change during computation
      
      if(pid_input == 0) {
        //simply skip everything for now so no rpm signal to stm will not cause fully-open ICV
        return;
      }

      /*Compute all the working error variables*/
      double error = pid_setpoint - pid_input;
      pid_iterm += (pid_ki * error);
      if(pid_iterm > pid_out_max) pid_iterm = pid_out_max;
      else if (pid_iterm < pid_out_min) pid_iterm = pid_out_min;
      double dInput = (pid_input - pid_lastinput);
 
      /*Compute PID Output*/
      pid_output = pid_kp * error + pid_iterm - pid_kd * dInput;
      if(pid_output > pid_out_max) pid_output = pid_out_max;
      else if(pid_output < pid_out_min) pid_output = pid_out_min;

      //send output to output :D
      analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
 
      /*Remember some variables for next time*/
      pid_lastinput = pid_input;

      Serial.print("\r\nRPM:\t");
      Serial.print(pid_input);
      Serial.print(";\tError:\t");
      Serial.print(error);
      Serial.print(";\tpid_iterm:\t");
      Serial.print(pid_iterm);
      Serial.print(";\tpid_output:\t");
      Serial.print(pid_output);
}

void pid_set_tunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)pid_sample_time)/1000;
   pid_kp = Kp;
   pid_ki = Ki * SampleTimeInSec;
   pid_kd = Kd / SampleTimeInSec;
}

void duty_it_capture_rising(void)
{
  duty_capture = rpm_duty_timer->getCaptureCompare(duty_ch_rising);
  /* frequency computation */
  if(duty_capture > duty_last_capture)
  {
    duty_freq_measured = PRM_DUTY_TIMER_IFREQ / (duty_capture - duty_last_capture);
    duty_cycle_measured = (duty_highstate * 100) / (duty_capture - duty_last_capture);
  }
  else if(duty_capture <= duty_last_capture)
  {
    /* 0x1000 is max overflow value */
    duty_freq_measured = PRM_DUTY_TIMER_IFREQ / (0x10000 + duty_capture - duty_last_capture);
    duty_cycle_measured = (duty_highstate * 100) / (0x10000 + duty_capture - duty_last_capture); 
  }

  duty_cycle_measured = 100 - duty_cycle_measured;  //we need to measure duration of "low level" => 100 - duty_cycle_measured
  duty_last_capture = duty_capture;
  duty_rollover_count = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rpm_duty_timer_it_rollover(void)
{
  //this is called with about 30Hz frequency
  
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
  icv_raw = analogRead(ICV_INPUT);        //it does not make sense to calculate these voltages every 1/30s and display them once

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

    if((rpm < RPM_MAX) and (rpm > 0)) {                 //check if the rpm we got makes sense - ignore if its way high (dont even update rpm_measured)
      //calculate average from 4 last good measurements and rotate the history
      rpm_measured = (rpm + rpm_history[0] + rpm_history[1] + rpm_history[2]) / 4; //average from last 4 values, rpm is uint32_t so the result of sum will fit
      rpm_history[2] = rpm_history[1];
      rpm_history[1] = rpm_history[0];
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
  if(lambda_voltage <= V_LEAN2) {
    //very lean
    digitalWrite(LED_LEAN2, LOW);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if((V_LEAN2 < lambda_voltage) and (lambda_voltage <= V_LEAN1)) {
    //lean
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, LOW);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if((V_LEAN1 < lambda_voltage) and (lambda_voltage < V_RICH1)) {
    //right
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_RICH1, HIGH);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if((V_RICH1 <= lambda_voltage) and (lambda_voltage < V_RICH2)) {
    //rich
    digitalWrite(LED_LEAN2, HIGH);
    digitalWrite(LED_LEAN1, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_RICH1, LOW);
    digitalWrite(LED_RICH2, HIGH);
    
  } else if(V_RICH2 <= lambda_voltage) {
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
  if(ovp_voltage > icv_voltage_abs) { 
    icv_voltage = ovp_voltage - icv_voltage_abs;
  } else {
    icv_voltage = 0;
  }
#ifdef SDEBUG
  Serial.printf("ICV = %umV\r\nICV_abs = %umV\r\nICV_uncal = %umV\r\n", icv_voltage, icv_voltage_abs, icv_voltage_uncal);
#endif    
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

  lcd_init();
  
  //ADC will use 12bit accuracy
  analogReadResolution(12);
  
  //inputs
  pinMode(LAMBDA_INPUT, INPUT_ANALOG);
  pinMode(BATT_INPUT, INPUT_ANALOG);
  pinMode(OVP_INPUT, INPUT_ANALOG);
  pinMode(ICV_INPUT, INPUT_ANALOG);
  pinMode(DUTY_INPUT, INPUT);
  pinMode(RPM_INPUT, INPUT);
  analogWriteFrequency(100);          //100Hz
  analogWriteResolution(8);          //we have 16bit timers so use them
  pinMode(ICV_PWM_OUT, OUTPUT);               //ICV PWM signal
  analogWrite(ICV_PWM_OUT, 127);             //16% ------- 6553500 = 100%
  
  pinMode(PB1, OUTPUT);             //debug test out - to be removed
  analogWrite(PB1, 127);

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
  //pinMode(RPM_INPUT, INPUT_PULLUP); //timer init sets the pin to INPUT, when pullup is needed it needs to be changed here again
  pid_set_tunings(2, 3, 0); //0.2, 0.1, 0
  Serial.printf("sampletime: %ums = %uHz", pid_sample_time, PRM_DUTY_TIMER_IFREQ / 65536);
  drawbasicscreen();
}

void loop() {
  get_battery();
  get_ovp();
  get_icv();
  
  showvalues();
  
  delay(CYCLE_DELAY);
  
  //Serial.print((String)"Frequency = " + duty_freq_measured);
  //Serial.println((String)"    Dutycycle = " + duty_cycle_measured);
  //Serial.println((String)"RPM Frequency = " + rpm_measured);
  
}
