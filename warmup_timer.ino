void check_warmup() {
  //check if warmup phase already passed or not
  if (!warmup_phase) {
    //warmup phase already passed - return immediatelly
    return;
  }

  //get time when engine_started
  if ((engine_started == 0) and (rpm_measured > 0)) {
    //engine started just now
    engine_started = millis();  //remember this time
    //pid_setpoint = rpm_warmup;    //this is done in setup() after eeload() - no need to set it here again;
    Serial.println(F("Engine warmup started"));
  }

  //check if engine was already started
  if (engine_started == 0) {
    //engine not started yet - exit
    return;
  }

  //we are in warmup phase and engine was started
  //check if warmup time passed
  unsigned long m = millis();                     //time now
  if ((m - engine_started) >= warmup_time) {
    //warmup time passed
    warmup_phase = false;
    pid_setpoint = rpm_idle;
    warmup_remaining = 0;
    Serial.println(F("Warmup over"));
    return;
  }

  //warmup time did not passed yet - calculate remaining warmup time
  warmup_remaining = engine_started + warmup_time - m;
}
