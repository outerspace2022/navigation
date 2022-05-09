/******************************************************************************

  Code adapted from online

******************************************************************************/

//For this example to work, you must daisy chain together two Qwiic LIDAR sensors with different I2C addresses
//To change the address of a Qwiic LIDAR sensor, please visit example 2

#include <LIDARLite_v4LED.h> //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun

//instantiate both LIDARs
LIDARLite_v4LED myLIDAR1;
LIDARLite_v4LED myLIDAR3;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Qwiic LIDARLite_v4 examples");
  Wire.begin(); //Join I2C bus

  //check if LIDARs will acknowledge over I2C
  //Connect to Qwiic LIDAR at address 0x5B
  if (myLIDAR1.begin(0x5B) == false) {
    Serial.println("LIDAR 1 did not acknowledge! Freezing.");
    while(1);
  }

    if (myLIDAR3.begin(0x4A) == false) {
    Serial.println("LIDAR 3 did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("All LIDARs acknowledged.");
}

void loop() {
  float newDistance1;
  float newDistance3;

  newDistance1 = myLIDAR1.getDistance();
  newDistance3 = myLIDAR3.getDistance();


  Serial.print("1,");
  Serial.println(newDistance1/100);

  Serial.print("2,");
  Serial.println(newDistance3/100);

  delay(200); //Don't hammer too hard on the I2C bus, used to be 20

}
