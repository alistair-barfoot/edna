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

const byte SHARP_PIN_L = A0; // left sensor
const byte SHARP_PIN_R = A1;  // right sensor
const byte SHARP_PIN_F = A2;  // front sensor

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

void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_F = analogRead(SHARP_PIN_F);
    int voltage = map(sharp_val_F,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Voltage

    Serial.println("Front sensor");

    Serial.println(voltage);
    if(voltage > 2310){
      Serial.println("<10");
//      break;
    }
    else if(voltage > 1300){
      Serial.println("<13");
      //Distance_F = (voltage - 2310)*-10/(2310 - 1300) + 10; // front distance
      // multply by scaling factor 10-20cm
    }
    else if(voltage > 920){
      Serial.println("13-23");
      Distance_F = (voltage - 1300)*-10/(1300 - 920) + 10;
      // 20-30
    }
    else if(voltage > 740){
      Serial.println("23-33");
      Distance_F = (voltage - 920)*-10/(920 - 740) + 10;
      // 30-40
    }
    else if(voltage > 610){
      Serial.println("33-43");
      Distance_F = (voltage - 740)*-10/(740 - 610) + 10;
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

//-----------------------------------------------------------------------------------

    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_R = analogRead(SHARP_PIN_R);
    int voltage_R = map(sharp_val_R,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Voltage

    Serial.println("Right Sensor");

    Serial.println(voltage_R);
    if(voltage_R > 2310){
      Serial.println("<10");
//      break;
    }
    else if(voltage_R > 1300){
      Serial.println("<13");
      //Distance_F = (voltage - 2310)*-10/(2310 - 1300) + 10; // front distance
      // multply by scaling factor 10-20cm
    }
    else if(voltage_R > 920){
      Serial.println("13-23");
      Distance_F = (voltage - 1300)*-10/(1300 - 920) + 10;
      // 20-30
    }
    else if(voltage_R > 740){
      Serial.println("23-33");
      Distance_F = (voltage - 920)*-10/(920 - 740) + 10;
      // 30-40
    }
    else if(voltage_R > 610){
      Serial.println("33-43");
      Distance_F = (voltage - 740)*-10/(740 - 610) + 10;
      // 40-50
    }
    else if(voltage_R > 510){
      Serial.println("43-53");
      // 50-60
    }
    else if(voltage_R > 450){
      Serial.println("53-63");
      // 60-70
    }
    else if(voltage_R > 410){
      Serial.println("63-73");
      // 70-80
    } else {
      Serial.println("Out of the range");
    }
    
    // Print all values
    Serial.print("\n");

    // Delay for a bit before reading the sensor again
    delay(200);

//-----------------------------------------------------------------------------------//

    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_L = analogRead(SHARP_PIN_L);
    int voltage_L = map(sharp_val_L,0,1023,0,3300); // taken from FY code (ty brian surgenor)Analog val to Distance_F

    Serial.println("Left sensor, sensor?");

    Serial.println(voltage_L);
    if(voltage_L > 2310){
      Serial.println("<10");
//      break;
    }
    else if(voltage_L > 1300){
      Serial.println("<13");
      //Distance_F = (voltage_L - 2310)*-10/(2310 - 1300) + 10; // front distance
      // multply by scaling factor 10-20cm
    }
    else if(voltage_L > 920){
      Serial.println("13-23");
      Distance_F = (voltage_L - 1300)*-10/(1300 - 920) + 10;
      // 20-30
    }
    else if(voltage_L > 740){
      Serial.println("23-33");
      Distance_F = (voltage_L - 920)*-10/(920 - 740) + 10;
      // 30-40
    }
    else if(voltage_L > 610){
      Serial.println("33-43");
      Distance_F = (voltage_L - 740)*-10/(740 - 610) + 10;
      // 40-50
    }
    else if(voltage_L > 510){
      Serial.println("43-53");
      // 50-60
    }
    else if(voltage_L > 450){
      Serial.println("53-63");
      // 60-70
    }
    else if(voltage_L > 410){
      Serial.println("63-73");
      // 70-80
    } else {
      Serial.println("Out of the range");
    }
    
    // Print all values
    Serial.print("\n");
}
