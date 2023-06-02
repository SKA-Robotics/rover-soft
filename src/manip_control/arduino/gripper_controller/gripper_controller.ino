#define PWM_PIN 9

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(9600);
}

int value = 100;

void loop() {
  while (Serial.available()) {
    value = Serial.read();
  }

  analogWrite(PWM_PIN, value);
  delay(10);
}