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
