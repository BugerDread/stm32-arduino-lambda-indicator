const double PID_TUNING_STEP = 0.001; 
const uint32_t WARMUP_TIME_STEP = 1000;    //1000ms = 1s, its also the minimum for warmup_time

char sdata, sdata_last = '\r';

void checkserial() {
  while (Serial.available() > 0) {
    sdata = Serial.read();
    switch (sdata) {
      case 'r':
        if (rpm_idle < RPM_MAX) {
          rpm_idle++;
          pid_setpoint = rpm_idle;
        }
        Serial.printf(F("rpm_idle = %u\r\n"), rpm_idle);
        break;
      case 'R':
        if (rpm_idle > RPM_MIN) {
          rpm_idle--;
          pid_setpoint = rpm_idle;
        }
        Serial.printf(F("rpm_idle = %u\r\n"), rpm_idle);
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
          icv_pwm_man = pid_output;
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
          icv_pwm_man = pid_output;
          analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
        }
        Serial.printf(F("pid_out = %u\r\n"), pid_output);
        break;

      case 'w':
        if (rpm_warmup < RPM_MAX) {
          //increase warmup rpm
          rpm_warmup++;
          if (warmup_phase) {
            pid_setpoint = rpm_warmup;
          }
        }
        Serial.printf(F("rpm_warmup = %u\r\n"), rpm_warmup);
        break;

      case 'W':
        if (rpm_warmup > RPM_MIN) {
          //decrease warmup rpm
          rpm_warmup--;
          if (warmup_phase) {
            pid_setpoint = rpm_warmup;
          }
        }
        Serial.printf(F("rpm_warmup = %u\r\n"), rpm_warmup);
        break;

      case 'e':
        if (warmup_time < (WARMUP_TIME_MAX - WARMUP_TIME_STEP)) {
          warmup_time += WARMUP_TIME_STEP;
        } else {
          warmup_time = WARMUP_TIME_MAX;
        }
        Serial.printf(F("warmup_time = %us\r\n"), warmup_time / 1000);
        break;

      case 'E':
        if (warmup_time > WARMUP_TIME_STEP) {
          warmup_time -= WARMUP_TIME_STEP;
        } else {
          warmup_time = WARMUP_TIME_STEP;
        }
        Serial.printf(F("warmup_time = %us\r\n"), warmup_time / 1000);
        break;

      case 'a':
        //Status
        Serial.printf(F("\r\nStatus:\r\n"
                        "rpm_measured = %u\r\n"
                        "pid_on = %s\r\n"
                        "pid_setpoint = %u\r\n"
                        "pid_output = %u\r\n"
                        "pid_out_min = %u\r\n"
                        "pid_out_max = %u\r\n"
                        "pid_kp = %.3f\r\n"
                        "pid_ki = %.3f\r\n"
                        "pid_kd = %.3f\r\n"
                        "rpm_idle = %u\r\n"
                        "rpm_warmup = %u\r\n"
                        "warmup_time = %u/%us\r\n\r\n"
                        ), rpm_measured, pid_on ? "true" : "false", pid_setpoint, pid_output, pid_out_min, pid_out_max,
                        pid_kp, pid_ki, pid_kd, rpm_idle, rpm_warmup, warmup_remaining / 1000, warmup_time / 1000);
        break;                        
      
      case 'h':
        Serial.println(F("\r\nHelp:\r\n"
                       "r / R = change target idle rpm\r\n"
                       "w / W = change warmup rpm\r\n"
                       "e / E = change warmup time\r\n"
                       "b / B = change pwm_out_min\r\n"
                       "t / T = change pwm_out_max\r\n"
                       "p / P / i / I / d / D = change PID configuration\r\n"
                       "c / C = enable / disable PID controller\r\n"
                       "+ / - = manual ICV PWM control (PID needs to be disabled first)\r\n"
                       "s / S = save config to EEPROM\r\n"
                       "l / L = load config from EEPROM\r\n"
                       "a = actual status\r\n"
                       "h = this help\r\n"));
        break;
    }
    
    if ((sdata != '\r') or (sdata != '\n')) {
      sdata_last = sdata;
    }
    
  }
}
