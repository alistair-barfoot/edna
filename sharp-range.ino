/**
 * @file sharp-range.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */

// Arduino analog input pin to which the Sharp sensor is connected

#define audio 10

const byte SHARP_PIN_L = A0; // left sensor
const byte SHARP_PIN_R = A1;  // right sensor
const byte SHARP_PIN_F = A2;  // front sensor
const byte speaker = 10;

// Variables to store the proximity measurement
int sharp_val_L = 0; // integer read from analog pin
int sharp_val_R = 0; // integer read from analog pin
int sharp_val_F = 0; // integer read from analog pin

float sharp_range_L; // range measurement [cm]
float sharp_range_R; // range measurement [cm]
float sharp_range_F; // range measurement [cm]

int Distance_L; // left distance [cm]
int Distance_R; // right distance [cm]
int Distance_F; // front distance [cm]

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);
}
  int countF = 0;
  int countL = 0;
  int countR = 0;
void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_F = analogRead(SHARP_PIN_F);
    int voltage = map(sharp_val_F,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Voltage


    if(voltage > 900){
      countF++;
      if(countF > 5){
        digitalWrite(audio, HIGH);
      }
      Serial.println("F  ");
       
      Distance_F = (voltage - 1300)*-10/(1300 - 920) + 10;
      // 20-30
      
    }else{
      countF = 0;
    }
    

    

//-----------------------------------------------------------------------------------

    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_R = analogRead(SHARP_PIN_R);
    int voltage_R = map(sharp_val_R,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Voltage

    
    if(voltage_R > 900){
      countR++;
      if(countR > 5){
        digitalWrite(audio, HIGH);
      }
      Serial.println("R  ");
      Distance_F = (voltage - 1300)*-10/(1300 - 920) + 10;
      // 20-30
    }else{
      countR = 0;
    }
   

   

//-----------------------------------------------------------------------------------//

    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_L = analogRead(SHARP_PIN_L);
    int voltage_L = map(sharp_val_L,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Distance_F

    
    if(voltage_L > 900){
      countL++;
      if(countL > 5){
        digitalWrite(audio, HIGH);
      }
      Serial.println("L");
      Distance_F = (voltage - 1300)*-10/(1300 - 920) + 10;
      
      // 20-30
    }else{
      countL = 0;
    }
    
    
    // Print all values
    delay(100);
    digitalWrite(audio, LOW);
}
