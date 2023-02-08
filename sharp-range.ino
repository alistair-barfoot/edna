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
const byte SHARP_PIN = A2;

// Variables to store the proximity measurement
int sharp_val = 0; // integer read from analog pin
float sharp_range; // range measurement [cm]
int Fistance; // front distance [cm]

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);
}

void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val = analogRead(SHARP_PIN);
    int voltage = map(sharp_val,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Voltage

    Serial.println(voltage);
    if(voltage > 2310){
      Serial.println("<10");
//      break;
    }
    else if(voltage > 1300){
      Serial.println("<13");
      //Fistance = (voltage - 2310)*-10/(2310 - 1300) + 10; // front distance
      // multply by scaling factor 10-20cm
    }
    else if(voltage > 920){
      Serial.println("13-23");
      Fistance = (voltage - 1300)*-10/(1300 - 920) + 10;
      // 20-30
    }
    else if(voltage > 740){
      Serial.println("23-33");
      Fistance = (voltage - 920)*-10/(920 - 740) + 10;
      // 30-40
    }
    else if(voltage > 610){
      Serial.println("33-43");
      Fistance = (voltage - 740)*-10/(740 - 610) + 10;
      // 40-50
    }
    else if(voltage > 510){
      Serial.println("43-53");
      // 50-60
    }
    else if(voltage > 450){
      Serial.println("53-63");
      // 60-70
    }
    else if(voltage > 410){
      Serial.println("63-73");
      // 70-80
    } else {
      Serial.println("Out of the range");
    }
    
    // Print all values
    Serial.print("\n");

    // Delay for a bit before reading the sensor again
    delay(200);
}
