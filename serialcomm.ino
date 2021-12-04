const double PID_TUNING_STEP = 0.01; 

char sdata, sdata_last = '\r';

void checkserial() {
  while (Serial.available() > 0) {
    sdata = Serial.read();
    switch (sdata) {
      case 'r':
        if (pid_setpoint < RPM_MAX) {
          pid_setpoint += 1;
        }
        Serial.printf(F("idle rpm = %.0f\r\n"), pid_setpoint);
        break;
      case 'R':
        if (pid_setpoint > 0) {
          pid_setpoint -= 1;
        }
        Serial.printf(F("idle rpm = %.0f\r\n"), pid_setpoint);
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
        Serial.print(F("Save enabled, press \"S\" to save to EEPROM\r\n"));
        break;   
      case 'S':
        if (sdata_last == 's') {
          eesave();
          //write_confirm = false;
        } else {
          Serial.print(F("Please press \"s\" to enable save first\r\n"));
        }
        break;

      case 'l':
        //load_confirm = true;
        Serial.print(F("Load enabled, press \"L\" to load from EEPROM\r\n"));
        break;   
      case 'L':
        if (sdata_last == 'l') {
          eeload();
          //load_confirm = false;
        } else {
          Serial.print(F("Please press \"l\" to enable load first\r\n"));
        }
        break;

      case 'h':
        Serial.print(F("Help:\r\n"
                       "r / R = change target idle rpm\r\n"
                       "b / B = change pwm_out_min\r\n"
                       "t / T = change pwm_out_max\r\n"
                       "p / P / i / I / d / D = change PID configuration\r\n"
                       "s / S = save config to EEPROM\r\n"
                       "l / L = load config from EEPROM\r\n"
                       "h = this help\r\n"));
        break;
        

    }
    
    if ((sdata != '\r') or (sdata != '\n')) {
      sdata_last = sdata;
    }
    
  }
}
