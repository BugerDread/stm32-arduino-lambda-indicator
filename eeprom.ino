#include <EEPROM.h>
const double MAGIC_CONSTANT = 42.42;

struct eeconfig_t {
  uint32_t rpm, out_min, out_max;
  double kp, ki, kd;
  double sum;
};

double eesum (struct eeconfig_t eedata) {
  return (eedata.kp + eedata.ki + eedata.kd + eedata.rpm + eedata.out_min + eedata.out_max + MAGIC_CONSTANT);
}

void eesave() {
  unsigned long m1 = millis();
  eeconfig_t eedata;

  eedata.rpm = rpm_idle;
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

  if (eedata.sum != eesum(eedata)) {
    Serial.print(F("Invalid checksum, can\'t load EEPROM data, FLASH defaults will be used\r\n"));
    return;
  }
  
  rpm_idle = eedata.rpm;
  pid_out_min = eedata.out_min;
  pid_out_max = eedata.out_max;
  pid_kp = eedata.kp;
  pid_ki = eedata.ki;
  pid_kd = eedata.kd;

  Serial.printf(F("Config loaded from EEPROM in %ums\r\n"), millis() - m1);
  return;
}
