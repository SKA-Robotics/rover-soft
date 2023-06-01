#define PWM_PIN 9

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(9600);
}

int command = 0;

void loop() {
  int byte;
  while (Serial.available()) {
    byte = Serial.read();
  }

  analogWrite(PWM_PIN, command);
  delay(10);
}