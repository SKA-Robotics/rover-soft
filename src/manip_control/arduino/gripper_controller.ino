#define PWM_PIN 9

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(9600);
}

int state = 0;
int command = 0;
int temp = 0;

void loop() {
  int byte;
  while (Serial.available()) {
    byte = Serial.read();
    switch(state) {
      case 0:
        if (byte == 128) state = 1;
        break;
      case 1:
        temp = byte;
        state = 2;
        break;
      case 2:
        if (byte == 192) command = temp;
        state = 0;
        break;
    }
  }

  analogWrite(PWM_PIN, command);
  delay(10);
}