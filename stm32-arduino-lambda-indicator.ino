//Lambda (O2) sensor indicator by BugerDread

//constants
  //general
  const uint16_t V_REFI = 1200;       //STM32F103 internal reference voltage [mV]
  const uint16_t V_LEAN2 = 100;       //very lean mixture voltage [mV]
  const uint16_t V_LEAN1 = 200;       //lean mixture voltage [mV]
  const uint16_t V_RICH1 = 700;       //rich mixture voltage [mV]
  const uint16_t V_RICH2 = 800;       //very rich mixture voltage [mV]
  const uint16_t CYCLE_DELAY = 50;    //delay for each round [ms]
  const uint8_t N_AVG = 10;           //how many samples to average to show on serial
  
  //inputs
  const uint16_t LAMBDA_INPUT = A0;   //lambda sensor voltage input pin (rich >= ~0.7V, lean <= ~0.2V)

  //outputs
  const uint16_t LED_LEAN2 = PB12;    //very lean mixture LED - on when LAMBDA_INPUT voltage <= V_LEAN2
  const uint16_t LED_LEAN1 = PB13;    //lean mixture LED - on when V_LEAN2 < LAMBDA_INPUT voltage <= V_LEAN1
  const uint16_t LED_RIGHT = PB14;    //right mixture LED - on when V_LEAN1 < LAMBDA_INPUT voltage < V_RICH1
  const uint16_t LED_RICH1 = PB15;    //rich mixture  LED - on when V_RICH1 <= LAMBDA_INPUT voltage < V_RICH2
  const uint16_t LED_RICH2 = PA8;     //very rich mixture LED - on when V_RICH2 <= LAMBDA_INPUT voltage

//global variables
uint16_t lambda_voltage, lambda_value, vref_value, i;
uint32_t lambda_voltage_avg_sum;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  while (!Serial);
  Serial.print(F("\r\n* * * BGR Lambda Indicator v0.01 * * *\r\n\r\nConfiguration\r\n=============\r\nLED update delay: "));
  Serial.print(CYCLE_DELAY);
  Serial.print(F("ms\r\nSamples avg for serial console: "));
  Serial.print(N_AVG);
  Serial.print(F("\r\nVery lean mixture: "));
  Serial.print(V_LEAN2);
  Serial.print(F("mV\r\nLean mixture: "));
  Serial.print(V_LEAN1);
  Serial.print(F("mV\r\nRich mixture: "));
  Serial.print(V_RICH1);
  Serial.print(F("mV\r\nVery rich mixture: "));
  Serial.print(V_RICH2);
  Serial.print(F("mV\r\n\r\n"));
  
  //ADC will use 12bit accuracy
  analogReadResolution(12);

  //init LED pins
  pinMode(LED_LEAN2, OUTPUT);
  digitalWrite(LED_LEAN2, LOW);
  pinMode(LED_LEAN1, OUTPUT);
  digitalWrite(LED_LEAN1, HIGH);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_RIGHT, HIGH);
  pinMode(LED_RICH1, OUTPUT);
  digitalWrite(LED_RICH1, HIGH);
  pinMode(LED_RICH2, OUTPUT);
  digitalWrite(LED_RICH2, HIGH);
  
  //do some fancy fx with LEDs on boot
  delay(100);
  digitalWrite(LED_LEAN2, HIGH);
  digitalWrite(LED_LEAN1, LOW);
  delay(100);
  digitalWrite(LED_LEAN1, HIGH);
  digitalWrite(LED_RIGHT, LOW);
  delay(100);
  digitalWrite(LED_RIGHT, HIGH);
  digitalWrite(LED_RICH1, LOW);
  delay(100);
  digitalWrite(LED_RICH1, HIGH);
  digitalWrite(LED_RICH2, LOW);
  delay(100);
}

void loop() {
  lambda_voltage_avg_sum = 0;
  for (i = 1; i <= N_AVG; i++) {    //print status to console only every N-th cycle
    vref_value = analogRead(AVREF);
    lambda_value = analogRead(LAMBDA_INPUT);
    lambda_voltage = ((uint32_t)lambda_value * V_REFI) / vref_value;
    lambda_voltage_avg_sum += lambda_voltage;
  
    //control LEDz
    if (lambda_voltage <= V_LEAN2) {
      //very lean
      digitalWrite(LED_LEAN2, LOW);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("#0000");
      
    } else if ((V_LEAN2 < lambda_voltage) and (lambda_voltage <= V_LEAN1)) {
      //lean
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, LOW);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("0#000");
      
    } else if ((V_LEAN1 < lambda_voltage) and (lambda_voltage < V_RICH1)) {
      //right
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, LOW);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("00#00");
      
    } else if ((V_RICH1 <= lambda_voltage) and (lambda_voltage < V_RICH2)) {
      //rich
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, LOW);
      digitalWrite(LED_RICH2, HIGH);
      if (i == N_AVG) Serial.print("000#0");
      
    } else if (V_RICH2 <= lambda_voltage) {
      //very rich
      digitalWrite(LED_LEAN2, HIGH);
      digitalWrite(LED_LEAN1, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
      digitalWrite(LED_RICH1, HIGH);
      digitalWrite(LED_RICH2, LOW);
      if (i == N_AVG) Serial.print("0000#");
      
    }
    delay(CYCLE_DELAY);
  }
  Serial.printf(" - lambda voltage avg: %umV\r\n", lambda_voltage_avg_sum / N_AVG);
}
