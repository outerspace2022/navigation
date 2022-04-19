// Motor A connections
int enA = 5; // Green (sq)
int in1 = 0; // Red
int in2 = 1; // Black
// Motor B connections
int enB = 6; // Orange (sq) This is 6 on the Uno but 4 on the old one
int in3 = 2; // Blue (sq)
int in4 = 3; // White (sq)

void setup() {
  Serial.begin(9600);
  //while(!Serial);
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

// "0123,5,789"
// "forw,2,1.5" 
// "back,3,030"
// "turn,0,180"
// "turn,1,090"

void loop() {
  //backwards(2,1.5);
  //delay(1000);
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(data);
//    if(data.equals("Words")) {
//      forwards(2, 1.5);
//    } else {
//      backwards(2,1.5);
//    }
//    delay(1000);
    //Serial.print("You sent me: ");
    //Serial.println(data);

    String direct = data.substring(0, 4);
    String param2 = data.substring(5,6);
    String param3 = data.substring(7,10);
    String pwm_str = data.substring(11,14);
    int pwm = pwm_str.toInt();


    if(direct.equals("forw")){
      forwards(param2.toInt(),param3.toDouble(), pwm);
    } else if(direct.equals("back")) {
      backwards(param2.toInt(),param3.toDouble(), pwm);
    } else if(direct.equals("turn")) {
      turn(param2.toInt(),param3.toInt(), pwm);
    }
    delay(1000);

//    
//    if(direct.equals("forw")) {
//      forwards(param2.toInt(), param3.toDouble());
//    } else if(direct.equals("back")) {
//      backwards(param2.toInt(), param3.toDouble());
//    } else {
//      // Turning case
//      turn(param2.toInt(), param3.toInt());
//    }
    
    //forwards(2, 1.5);
//    delay(1000);

    
  }

  // Wait for input
  // Execute input and delay

//  forwards(2, 1.5);
//  delay(1000);
//  backwards(2, 1.5);
//  delay(1000);
//  turn(1, 90);
//  delay(1000);
//  turn(0, 90);
//  delay(1000);
  
}


void forwards(int seconds, double speed, int pwm)
{
  // 127 sets the PWM to 50%
  // speed should be used here somehow
  analogWrite(enA, pwm);
  analogWrite(enB, pwm);
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

void backwards(int seconds, double speed, int pwm) 
{
  // Set PWM to 50%
  // speed should be used here somehow
  analogWrite(enA, 200);
  analogWrite(enB, 200);

  // Set motors to forwards
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(1000 * seconds);

  stopMotors();
}



//Direction is either CK (1) or CCK (0)
void turn(bool direction, int angle, int pwm)
{
  analogWrite(enA, pwm);
  analogWrite(enB, pwm);
  
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
  // 1 sec = 100 degrees
  // 1000 = 100 degrees
  // 
  delay(angle * 10); //This should be dependent on speed and angle somehow
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
