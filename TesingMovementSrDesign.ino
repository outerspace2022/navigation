// Motor A connections
int enA = 5;
int in1 = 0;
int in2 = 1;
// Motor B connections
int enB = 4;
int in3 = 2;
int in4 = 3;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  stopMotors();
}

void loop() {

  forwards(2, 1.5);
  delay(1000);
  backwards(2, 1.5);
  delay(1000);
  turn(1, 90);
  delay(1000);
  turn(0, 90);
  delay(1000);
  
}


void forwards(int seconds, double speed)
{
  // 127 sets the PWM to 50%
  // speed should be used here somehow
  analogWrite(enA, 127);
  analogWrite(enB, 127);
  //analogWrite(enA, 255);
  //analogWrite(enB, 255);

  // Set motors to backwards
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(1000 * seconds);

  stopMotors();

}

void backwards(int seconds, double speed) 
{
  // Set PWM to 50%
  // speed should be used here somehow
  analogWrite(enA, 127);
  analogWrite(enB, 127);

  // Set motors to forwards
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(1000 * seconds);

  stopMotors();
}


//Direction is either CK (1) or CCK (0)
void turn(bool direction, int angle)
{
  if (direction) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
     digitalWrite(in1, LOW);
     digitalWrite(in2, HIGH);
     digitalWrite(in3, HIGH);
     digitalWrite(in4, LOW);
  }
  delay(1000); //This should be dependent on speed and angle somehow
  stopMotors();
}

void stopMotors() 
{
  //Stop motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
