#include <EEPROM.h>
const double MAGIC_CONSTANT = 42.42;

struct eeconfig_t {
  uint32_t rpm, rpm_warmup, warmup_time, out_min, out_max;
  double kp, ki, kd;
  double sum;
};

double eesum (struct eeconfig_t eedata) {
  return (eedata.kp + eedata.ki + eedata.kd + eedata.rpm + eedata.out_min + eedata.out_max + eedata.rpm_warmup + eedata.warmup_time + MAGIC_CONSTANT);
}

void eesave() {
  unsigned long m1 = millis();
  eeconfig_t eedata;

  eedata.rpm = rpm_idle;
  eedata.rpm_warmup = rpm_warmup;
  eedata.warmup_time = warmup_time;
  eedata.out_min = pid_out_min;
  eedata.out_max = pid_out_max;
  eedata.kp = pid_kp;
  eedata.ki = pid_ki;
  eedata.kd = pid_kd;
  eedata.sum = eesum(eedata);
  
  EEPROM.put(0, eedata);
  
  Serial.printf(F("Config saved to EEPROM in %ums\r\n"), millis() - m1);
}

void eeload() {
  unsigned long m1 = millis();
  eeconfig_t eedata;
  
  EEPROM.get(0, eedata);

  //checksw
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
  if ((eedata.kp < 0) or (eedata.ki < 0) or (eedata.kd < 0)) {
    Serial.println(F("Invalid pid constants data, FLASH defaults will be used"));
    return;
  }
  
  rpm_idle = eedata.rpm;
  rpm_warmup = eedata.rpm_warmup;
  warmup_time = eedata.warmup_time;
  pid_out_min = eedata.out_min;
  pid_out_max = eedata.out_max;
  pid_kp = eedata.kp;
  pid_ki = eedata.ki;
  pid_kd = eedata.kd;

  Serial.printf(F("Config loaded from EEPROM in %ums\r\n"), millis() - m1);
  return;
}
