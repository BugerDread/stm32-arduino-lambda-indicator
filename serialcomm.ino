const double PID_TUNING_STEP = 0.01; 

char sdata, sdata_last = '\r';

void checkserial() {
  while (Serial.available() > 0) {
    sdata = Serial.read();
    switch (sdata) {
      case 'r':
        if (pid_setpoint < RPM_MAX) {
          pid_setpoint++;
        }
        Serial.printf(F("idle rpm = %u\r\n"), pid_setpoint);
        break;
      case 'R':
        if (pid_setpoint > 0) {
          pid_setpoint--;
        }
        Serial.printf(F("idle rpm = %u\r\n"), pid_setpoint);
        break;

      case 'p':
        pid_kp += PID_TUNING_STEP;
        pid_set_tunings(pid_kp, pid_ki, pid_kd);
        break;
      case 'P':
        if (pid_kp < PID_TUNING_STEP) {
          pid_kp = 0;
        } else {
          pid_kp -= PID_TUNING_STEP;
        }
        pid_set_tunings(pid_kp, pid_ki, pid_kd);
        break;

      case 'i':
        pid_ki += PID_TUNING_STEP;
        pid_set_tunings(pid_kp, pid_ki, pid_kd);
        break;
      case 'I':
        if (pid_ki < PID_TUNING_STEP) {
          pid_ki = 0;
        } else {
          pid_ki -= PID_TUNING_STEP;
        }
        pid_set_tunings(pid_kp, pid_ki, pid_kd);
        break;

      case 'd':
        pid_kd += PID_TUNING_STEP;
        pid_set_tunings(pid_kp, pid_ki, pid_kd);
        break;
      case 'D':
        if (pid_kd < PID_TUNING_STEP) {
          pid_kd = 0;
        } else {
          pid_kd -= PID_TUNING_STEP;
        }
        pid_set_tunings(pid_kp, pid_ki, pid_kd);
        break;

      case 'b':
        if (pid_out_min < 255) {
          pid_out_min++;
        }
        Serial.printf(F("pid_out_min = %u\r\n"), pid_out_min);
        break;
      case 'B':
        if (pid_out_min > 0) {
          pid_out_min--;
        }
        Serial.printf(F("pid_out_min = %u\r\n"), pid_out_min);
        break;

      case 't':
        if (pid_out_max < 255) {
          pid_out_max++;
        }
        Serial.printf(F("pid_out_max = %u\r\n"), pid_out_max);
        break;
      case 'T':
        if (pid_out_max > 0) {
          pid_out_max--;
        }
        Serial.printf(F("pid_out_max = %u\r\n"), pid_out_max);
        break;
      
      case 's':
        //write_confirm = true;
        Serial.println(F("Save enabled, press \"S\" to save to EEPROM"));
        break;   
      case 'S':
        if (sdata_last == 's') {
          Serial.println(F("Saving to EEPROM"));
          eesave();
          //write_confirm = false;
        } else {
          Serial.println(F("Please press \"s\" to enable save first"));
        }
        break;

      case 'l':
        //load_confirm = true;
        Serial.println(F("Load enabled, press \"L\" to load from EEPROM"));
        break;   
      case 'L':
        if (sdata_last == 'l') {
          Serial.println(F("Loading from EEPROM"));
          eeload();
          //load_confirm = false;
        } else {
          Serial.println(F("Please press \"l\" to enable load first"));
        }
        break;

      case 'c':
        pid_on = true;
        Serial.println(F("PID controller enabled"));
        break;
      case 'C':
        pid_on = false;
        Serial.println(F("PID controller disabled"));
        break;       

      case '+':
        if (pid_on) {
          Serial.println(F("Disable PID controller first"));
          break;
        } 
        if (pid_output < pid_out_max) {
          pid_output++;
          analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
        }
        Serial.printf(F("pid_out = %u\r\n"), pid_output);
        break;
      case '-':
        if (pid_on) {
          Serial.println(F("Disable PID controller first"));
          break;
        } 
        if (pid_output > pid_out_min) {
          pid_output--;
          analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
        }
        Serial.printf(F("pid_out = %u\r\n"), pid_output);
        break;

      case 'a':
        //Serial.println(F("Status:"));
        Serial.printf(F("\r\nStatus:\r\n"
                        "rpm_measured = %u\r\n"
                        "pid_on = %s\r\n"
                        "pid_setpoint = %u\r\n"
                        "pid_output = %u\r\n"
                        "pid_out_min = %u\r\n"
                        "pid_out_max = %u\r\n"
                        "pid_kp = %.2f\r\n"
                        "pid_ki = %.2f\r\n"
                        "pid_kd = %.2f\r\n"
                        ), rpm_measured, pid_on ? "true" : "false", pid_setpoint, pid_output, pid_out_min, pid_out_max, pid_kp, pid_ki, pid_kd);
        break;                        
      
      case 'h':
        Serial.println(F("\r\nHelp:\r\n"
                       "r / R = change target idle rpm\r\n"
                       "b / B = change pwm_out_min\r\n"
                       "t / T = change pwm_out_max\r\n"
                       "p / P / i / I / d / D = change PID configuration\r\n"
                       "c / C = enable / disable PID controller\r\n"
                       "+ / - = manual ICV PWM control (PID needs to be disabled first)\r\n"
                       "s / S = save config to EEPROM\r\n"
                       "l / L = load config from EEPROM\r\n"
                       "a = actual status\r\n"
                       "h = this help"));
        break;
        

    }
    
    if ((sdata != '\r') or (sdata != '\n')) {
      sdata_last = sdata;
    }
    
  }
}
