#define Bridge1Mot1_PWM 3
#define Bridge1Mot1_dirA 4
#define Bridge1Mot1_dirB 5
#define Bridge1Mot2_PWM 6
#define Bridge1Mot2_dirA 7
#define Bridge1Mot2_dirB 8

#define Bridge2Mot1_PWM 10
#define Bridge2Mot1_dirA A0
#define Bridge2Mot1_dirB A1
#define Bridge2Mot2_PWM 11
#define Bridge2Mot2_dirA A2
#define Bridge2Mot2_dirB A3

#define COMMAND_TIMEOUT 1000 // milliseconds
int lastCommandTime;

void setup() {
  pinMode(Bridge1Mot1_PWM, OUTPUT);
  pinMode(Bridge1Mot1_dirA, OUTPUT);
  pinMode(Bridge1Mot1_dirB, OUTPUT);
  pinMode(Bridge1Mot2_PWM, OUTPUT);
  pinMode(Bridge1Mot2_dirA, OUTPUT);
  pinMode(Bridge1Mot2_dirB, OUTPUT);
  pinMode(Bridge2Mot1_PWM, OUTPUT);
  pinMode(Bridge2Mot1_dirA, OUTPUT);
  pinMode(Bridge2Mot1_dirB, OUTPUT);
  pinMode(Bridge2Mot2_PWM, OUTPUT);
  pinMode(Bridge2Mot2_dirA, OUTPUT);
  pinMode(Bridge2Mot2_dirB, OUTPUT);
  Serial.begin(9600);
}
/*
0 - wait for start
1 - wait for motor_index
2 - wait for pwm value
3 - wait for end
*/
int state = 0;
int targetMotor = 0;
int targetSpeed = 0;
int inputByte = 0;

struct MotorValues {
  int pwm[4];
  bool direction[4];
};

MotorValues motors = {0};

void loop() {
  while (Serial.available() > 0) {
    int inputByte = Serial.read();
    // Serial.println(inputByte);
    switch(state) { 
      case 0:
        if (inputByte == 128) {
          state = 1;
        }
        break;
      case 1:
        if (inputByte < 0 || inputByte > 3) {
          state = 0;
          break;
        }
        targetMotor = inputByte;
        state = 2;
        break;
      case 2:
        if (abs(inputByte) > 255) {
          state = 0;
          break;
        }
        targetSpeed = (inputByte - 127) * 2;
        state = 3;
        break;
      case 3:
        if (inputByte == 192) {
          motors.pwm[targetMotor] = abs(targetSpeed);
          motors.direction[targetMotor] = (targetSpeed > 0);
          lastCommandTime = millis();
          state = 0;
        } else {
          state = 0;
        }
        break;
      default:
        break;
    }
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // Stop all motors if no new command was received
    motors.pwm[0] = 0;
    motors.pwm[1] = 0;
    motors.pwm[2] = 0;
    motors.pwm[3] = 0;
  }

  digitalWrite(Bridge1Mot1_dirA, motors.direction[0] ? LOW : HIGH);
  digitalWrite(Bridge1Mot1_dirB, motors.direction[0] ? HIGH : LOW);
  analogWrite(Bridge1Mot1_PWM,   motors.pwm[0]);

  digitalWrite(Bridge1Mot2_dirA, motors.direction[1] ? LOW : HIGH);
  digitalWrite(Bridge1Mot2_dirB, motors.direction[1] ? HIGH : LOW);
  analogWrite(Bridge1Mot2_PWM, motors.pwm[1]);
  
  digitalWrite(Bridge2Mot1_dirA, motors.direction[2] ? LOW : HIGH);
  digitalWrite(Bridge2Mot1_dirB, motors.direction[2] ? HIGH : LOW);
  analogWrite(Bridge2Mot1_PWM, motors.pwm[2]);

  digitalWrite(Bridge2Mot2_dirA, motors.direction[3] ? LOW : HIGH);
  digitalWrite(Bridge2Mot2_dirB, motors.direction[3] ? HIGH : LOW);
  analogWrite(Bridge2Mot2_PWM, motors.pwm[3]);
}

