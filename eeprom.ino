#include <EEPROM.h>
const double MAGIC_CONSTANT = 42.42;

struct eeconfig_t {
  bool pid_on;
  uint32_t rpm, rpm_warmup, boost_rpm, warmup_time, out_min, out_max, icv_pwm_man;
  double kp, ki_open, ki_close, kd, boost_kp;
  double sum;
};

double eesum (struct eeconfig_t eedata) {
  return (eedata.kp + eedata.ki_open + eedata.ki_close + eedata.kd + eedata.rpm + eedata.out_min + eedata.out_max + eedata.boost_kp + eedata.boost_rpm
          + eedata.rpm_warmup + eedata.warmup_time + eedata.icv_pwm_man + (eedata.pid_on * MAGIC_CONSTANT) + MAGIC_CONSTANT);
}

void eesave() {
  unsigned long m1 = millis();
  eeconfig_t eedata;

  eedata.icv_pwm_man = icv_pwm_man;
  eedata.pid_on = pid_on;
  eedata.rpm = rpm_idle;
  eedata.rpm_warmup = rpm_warmup;
  eedata.warmup_time = warmup_time;
  eedata.out_min = pid_out_min;
  eedata.out_max = pid_out_max;
  eedata.kp = pid_kp;
  eedata.ki_open = pid_ki_open;
  eedata.ki_close = pid_ki_close;
  eedata.kd = pid_kd;
  eedata.boost_rpm = pid_boost_rpm;
  eedata.boost_kp = pid_boost_kp;
  eedata.sum = eesum(eedata);

  eeprint(eedata);
  
  EEPROM.put(0, eedata);
  
  Serial.printf(F("Config saved to EEPROM in %ums\r\n"), millis() - m1);
}

void eeprint(struct eeconfig_t eedata) {
  Serial.printf(F("\r\nEEPROM data\r\n"
                  "pid_on = %s\r\n"
                  "rpm = %u\r\n"
                  "rpm_warmup = %u\r\n"
                  "warmup_time = %u\r\n"
                  "out_min = %u\r\n"
                  "out_max = %u\r\n"
                  "icv_pwm_man = %u\r\n"
                  "pid_kp = %.3f\r\n"
                  "pid_ki_open = %.3f\r\n"
                  "pid_ki_close = %.3f\r\n"
                  "pid_kd = %.3f\r\n"
                  "pid_boost_rpm = %u\r\n"
                  "pid_boost_kp = %.3f\r\n"
                  "sum = %s\r\n\r\n"
                  ), eedata.pid_on ? "true" : "false", eedata.rpm, eedata.rpm_warmup, eedata.warmup_time, 
                  eedata.out_min, eedata.out_max, eedata.icv_pwm_man, eedata.kp, eedata.ki_open, eedata.ki_close, eedata.kd,
                  eedata.boost_rpm, eedata.boost_kp, (eedata.sum == eesum(eedata)) ? "valid" : "invalid");
}

void eeload() {
  unsigned long m1 = millis();
  eeconfig_t eedata;
  
  EEPROM.get(0, eedata);

  eeprint(eedata);

  //checks
  if (eedata.sum != eesum(eedata)) {
    Serial.println(F("Invalid checksum, can\'t load EEPROM data, FLASH defaults will be used"));
    return;
  }
  if ((eedata.rpm < RPM_MIN) or (eedata.rpm > RPM_MAX)) {
    Serial.println(F("Invalid rpm_idle data, FLASH defaults will be used"));
    return;
  }
  if ((eedata.rpm_warmup < RPM_MIN) or (eedata.rpm_warmup > RPM_MAX)) {
    Serial.println(F("Invalid rpm_warmup data, FLASH defaults will be used"));
    return;
  }
  if (eedata.warmup_time > WARMUP_TIME_MAX) {
    Serial.println(F("Invalid warmup_time data, FLASH defaults will be used"));
    return;
  }
  if (eedata.out_min > 255) {
    Serial.println(F("Invalid pid_out_min data, FLASH defaults will be used"));
    return;
  }  
  if (eedata.out_max > 255) {
    Serial.println(F("Invalid pid_out_max data, FLASH defaults will be used"));
    return;
  }
  if ((eedata.kp < 0) or (eedata.ki_open < 0) or (eedata.ki_close < 0) or (eedata.kd < 0)) {
    Serial.println(F("Invalid pid constants data, FLASH defaults will be used"));
    return;
  }
  if (eedata.icv_pwm_man > 255) {
    Serial.println(F("Invalid icv_pwm_man, FLASH defaults will be used"));
    return;
  }
  if (eedata.boost_rpm > eedata.rpm) {
    Serial.println(F("Invalid pid_boost_rpm, FLASH defaults will be used"));
    return;
  }
  if (eedata.boost_kp < 0) {
    Serial.println(F("Invalid pid_boost_kp, FLASH defaults will be used"));
    return;
  }
  
  icv_pwm_man = eedata.icv_pwm_man;
  rpm_idle = eedata.rpm;
  rpm_warmup = eedata.rpm_warmup;
  warmup_time = eedata.warmup_time;
  pid_out_min = eedata.out_min;
  pid_out_max = eedata.out_max;
  pid_kp = eedata.kp;
  pid_ki_open = eedata.ki_open;
  pid_ki_close = eedata.ki_close;
  pid_kd = eedata.kd;
  pid_on = eedata.pid_on;
  pid_boost_rpm = eedata.boost_rpm;
  pid_boost_kp = eedata.boost_kp;

  Serial.printf(F("Config loaded from EEPROM in %ums\r\n"), millis() - m1);
  return;
}
