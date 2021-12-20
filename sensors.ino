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
