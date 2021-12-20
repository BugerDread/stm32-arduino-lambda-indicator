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
      if (abs(error) < 5) {                   //ignore very small errors
        error = 0;
      }
      
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
