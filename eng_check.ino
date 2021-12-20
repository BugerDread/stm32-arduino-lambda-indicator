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
