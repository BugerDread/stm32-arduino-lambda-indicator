void pid_compute()
{     
      if (!pid_on) return;    //exit if pid disabled
      
      uint32_t input = rpm_measured;   //rpm_measured is a volatile variable, we dont want it to change during computation
        
      //do we need to filter the rpm_measured to make sure it is sane?
      //rpm_measured == 0 = motor is not spinning or we cant measure such low rpms
      //maybe we will count failed passes and disable ICV control if sane signal not received for a while? - future
//      if (input == 0) {                     //if we lost the rpm signal
//        //pid_output = ICV_PWM_DEFAULT;
//        //analogWrite(ICV_PWM_OUT, pid_output);
//        return;
//      }
   
      //Compute PID error variables
      int32_t pid_error = pid_setpoint - input;
      int32_t dInput = input - pid_lastinput;

      //compute iterm
      //as a hack lets use Kp as Ki for speed up the engine and Ki to slow it down (so Kp should be > Ki if we want to slow down slower than speed up)
      //because we are not going to use the Pterm now
      //but down forget to fix pid_set_tunings (remove "* pid_sample_time_s;" from Kp when Kp will be used again)
      if (pid_error > 0) {
        //motor spinning too slow - open ICV faster - use kp as a temp hack
        pid_iterm += pid_kp_internal * pid_error;
      } else {
        //motor spinning too fast - close ICV slowly
        pid_iterm += pid_ki_internal * pid_error;
      }

      //add (in fact substract) derivative part to iterm if engine is slowing down
      if (dInput < 0) {
        //engine slowing down
        pid_iterm -= pid_kd_internal * dInput;
      }
      
      //prevent iterm to windup (limit iterm to pid_out_max / pid_out_min)
      if(pid_iterm > pid_out_max) pid_iterm = pid_out_max;
        else if (pid_iterm < pid_out_min) pid_iterm = pid_out_min;
      
      /*Compute PID Output*/
      double output = pid_iterm;  // (pid_kp_internal * pid_error) - pid_kd_internal * dInput;

      //check if we need boost (to avoid engine dying on lpg)
      if (input < pid_boost_rpm) {
        //yes booost pleeeassssseeee
        output += (pid_boost_rpm - input) * pid_boost_kp;   //add error * boost_kp to output
      }
      
      //limit PID out range
      if (output > pid_out_max) output = pid_out_max;
        else if (output < pid_out_min) output = pid_out_min;
        
      //now we have the result computed
      pid_output = output;
      analogWrite(ICV_PWM_OUT, pid_output);               //send output to ICV
 
      /*Remember some variables for next time*/
      pid_lastinput = input;

//      if (++pid_debug_cnt >= 10) {
//        Serial.printf(F("RPM: %-5u ERR: %-5d OUT: %u\r\n"), rpm_measured, pid_error, pid_output);
//        pid_debug_cnt = 0;
//      }
}

void pid_set_tunings(double Kp, double Ki, double Kd)
{
   pid_kp_internal = Kp * pid_sample_time_s;  //!!!should be without "* pid_sample_time_s" but we are using this constant as temporary 2nd iterm paramatter
   pid_ki_internal = Ki * pid_sample_time_s;
   pid_kd_internal = Kd / pid_sample_time_s / 10; // divided by 10 to make the setting more fine

   Serial.printf(F("PID params:\r\n"
                   "pid_sample_time = %.2fms / %uHz\r\n"
                   "pid_kp = %.3f\r\n"
                   "pid_ki = %.3f\r\n"
                   "pid_kd = %.3f\r\n"),
                   pid_sample_time_s * 1000, PRM_DUTY_TIMER_IFREQ / 65536,
                   pid_kp, pid_ki, pid_kd);
}
