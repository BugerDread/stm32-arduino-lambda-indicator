const double PID_TUNING_STEP = 0.01; 

void checkserial() {
  while (Serial.available() > 0) {
    switch (Serial.read()) {
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

      case 'l':
        if (pid_out_min < 255) {
          pid_out_min++;
        }
        Serial.printf(F("pid_out_min = %u\r\n"), pid_out_min);
        break;
      case 'L':
        if (pid_out_min > 0) {
          pid_out_min--;
        }
        Serial.printf(F("pid_out_min = %u\r\n"), pid_out_min);
        break;

      case 'h':
        if (pid_out_max < 255) {
          pid_out_max++;
        }
        Serial.printf(F("pid_out_min = %u\r\n"), pid_out_max);
        break;
      case 'H':
        if (pid_out_max > 0) {
          pid_out_max--;
        }
        Serial.printf(F("pid_out_max = %u\r\n"), pid_out_max);
        break;
      
        
    }
  }
}
