#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6tKACaSrZ"
#define BLYNK_TEMPLATE_NAME "Transmission Line Cable Fault Detection"
#define BLYNK_AUTH_TOKEN "qi_iJjp_gLiby-V94mWuTBGqE5TuiGUS"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Arduino.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Jontrotantrik";                                     // Your WiFi credentials.
char pass[] = "jtl.robotics";

// Pin assignments for current sensors
const int currentSensor = 34;
int mVperAmp = 185;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

// Pin assignments for voltage sensors
const int voltageSensor = 35;
float voltage=0;

#define RL 13
#define BL 27
#define YL 14
#define in4 26
#define load 18

#define BUTTON_PIN_1 25
#define BUTTON_PIN_2 32
#define BUTTON_PIN_3 33
//#define BUTTON_PIN_4 13
//#define BUTTON_PIN_5 15
//#define BUTTON_PIN_6 13

LiquidCrystal_I2C lcd(0x27,16,2);//for i2c lcd
BlynkTimer timer;
int phase[3] = {16, 17, 19}; //******************************************************* 

// Initialize Wi-Fi
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  lcd.init(); 
   lcd.backlight();
  lcd.begin(16, 2);
  setupWiFi();
  Blynk.begin(auth, ssid, pass);
  pinMode(RL,OUTPUT);
   pinMode(BL,OUTPUT);
    pinMode(YL,OUTPUT);
     pinMode(in4,OUTPUT);
      ledcSetup(0,1E5,12);
  ledcAttachPin(23,0);

  pinMode(BUTTON_PIN_1, INPUT_PULLUP);

  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  pinMode(BUTTON_PIN_3, INPUT_PULLUP);

  //pinMode(BUTTON_PIN_4, INPUT_PULLUP);

  //pinMode(BUTTON_PIN_5, INPUT_PULLUP);

  //pinMode(BUTTON_PIN_6, INPUT_PULLUP);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Transmission Line");
  lcd.setCursor(0, 1);
  lcd.print("Cable Fault");
  delay(2000);
  for (int j = 0; j < 3; j++)
            { 
              pinMode(phase[j], OUTPUT);
            //  pinMode(relay,OUTPUT);
              //  pinMode(relay,OUTPUT);
               
              //Serial.begin(9600);
              } 
}


void loop() {
  int buttonPressed1=digitalRead(BUTTON_PIN_1);
  int buttonPressed2=digitalRead(BUTTON_PIN_2);
  int buttonPressed3=digitalRead(BUTTON_PIN_3);
  //int buttonPressed4=digitalRead(BUTTON_PIN_4);
  Serial.println(buttonPressed1);
   Serial.println(buttonPressed2);
    Serial.println(buttonPressed3);
    // Serial.println(buttonPressed4);

  Blynk.run();
  timer.run();
  
  voltage_sense();
   digitalWrite(YL,HIGH);
      digitalWrite(BL,HIGH);
       digitalWrite(RL,HIGH);
       // digitalWrite(in4,LOW);
  if (buttonPressed1==LOW && buttonPressed2==LOW) {
    //Serial.println("Button 1 & 2 pressed");
    digitalWrite(RL,LOW);
    digitalWrite(YL,LOW);
    digitalWrite(BL,HIGH);
   buzzer();
   lcd.clear();
    lcd.setCursor(0, 0);
      lcd.print("Line To Line");
      lcd.setCursor(0, 1);
      lcd.print("Fault Detected");
    delay(1000);
    Blynk.virtualWrite(V2, "Line To Line Fault Detected");
   Blynk.virtualWrite(V2, " \n");
  
      
    // Perform actions for button 1
  }
   else if (buttonPressed2==LOW && buttonPressed3==LOW) {
    //Serial.println("Button 1 & 2 pressed");
    digitalWrite(RL,HIGH);
    digitalWrite(YL,LOW);
    digitalWrite(BL,LOW);
   buzzer();
   lcd.clear();
    lcd.setCursor(0, 0);
      lcd.print("Double Line-GND");
      lcd.setCursor(0, 1);
      lcd.print("Fault Detected");
    delay(1000);
    Blynk.virtualWrite(V2, "Double Line To Ground Fault Detected");
   Blynk.virtualWrite(V2, " \n");
      
    // Perform actions for button 1
  }

  else if (buttonPressed1==LOW) {
   // Serial.println("Button 3 & 4 pressed");
    // Perform actions for button 1
    digitalWrite(RL,LOW);
      buzzer();
      //digitalWrite(buzzer,HIGH);
      lcd.setCursor(0, 0);
      lcd.print("Red Line-Ground");
      lcd.setCursor(0, 1);
      lcd.print("Fault Detected");
      delay(1000);
      Blynk.virtualWrite(V2, "Red Line To Ground Fault Detected ");
  Blynk.virtualWrite(V2, " \n");
  }
  else if (buttonPressed2==LOW) {
   // Serial.println("Button 3 & 4 pressed");
    // Perform actions for button 1
    digitalWrite(YL,LOW);
      buzzer();
      //digitalWrite(buzzer,HIGH);
      lcd.setCursor(0, 0);
      lcd.print("Yellow Line-Ground");
      lcd.setCursor(0, 1);
      lcd.print("Fault Detected");
      delay(1000);
      Blynk.virtualWrite(V2, "Yellow Line To Ground Fault Detected ");
  Blynk.virtualWrite(V2, " \n");
  }
  else if (buttonPressed3==LOW) {
   // Serial.println("Button 3 & 4 pressed");
    // Perform actions for button 1
    digitalWrite(BL,LOW);
      buzzer();
      //digitalWrite(buzzer,HIGH);
      lcd.setCursor(0, 0);
      lcd.print("Blue Line-Ground");
      lcd.setCursor(0, 1);
      lcd.print("Fault Detected");
      delay(1000);
      Blynk.virtualWrite(V2, "Blue Line To Ground Fault Detected ");
  Blynk.virtualWrite(V2, " \n");
  }
  else
  {
    over_head();
  }
// else if (buttonPressed2==LOW && buttonPressed3==LOW) {
//    // Serial.println("Button 3 & 4 pressed");
//     // Perform actions for button 1
//     delay(80);
//     digitalWrite(YL,HIGH);
//       buzzer();
//         //digitalWrite(buzzer,HIGH);
//       lcd.setCursor(0, 0);
//       lcd.print("Double Line-Ground");
//       lcd.setCursor(0, 1);
//       lcd.print("Fault Detected");
//       delay(1000);
//        Blynk.virtualWrite(V2, "Double Line To Ground Fault Detected ");
//  Blynk.virtualWrite(V2, " \n");
//   }
  // Additional code in the loop
}

void voltage_sense()
{
  lcd.clear();
  int sensorValue = analogRead(voltageSensor);
//    int inVolts = 297; // approximately input voltage for the suppler
//  int outputVolts = (sensorValue+inVolts)/4.5; 
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 250V):
  int voltage = sensorValue * (250.0 / 4095.0);

  //current sense
  Voltage = getVPP();
  VRMS = (Voltage/2.0) *0.707;   //root 2 is 0.707
  AmpsRMS = (VRMS * 1000)/mVperAmp;
  // print out the value you read:
  Serial.print("AC Voltage: ");
  Serial.print(voltage);
  Serial.println(" Volts");
  lcd.setCursor(1, 0);
      lcd.print("V");
       lcd.setCursor(0, 1);
      lcd.print(voltage);
      lcd.print("V");
         Blynk.virtualWrite(V3, "Voltage: ");
  Blynk.virtualWrite(V3, voltage);
  Blynk.virtualWrite(V3, " V\n");

      Serial.print(AmpsRMS);
  Serial.print(" Amps RMS  ---  ");
     Blynk.virtualWrite(V3, "Current: ");
  Blynk.virtualWrite(V3, AmpsRMS);
  Blynk.virtualWrite(V3, " A\n");
  Watt = (AmpsRMS*voltage/1.3);      // 1.3 is an empirical calibration factor
  Serial.print(Watt);
  Serial.println(" W");
   Blynk.virtualWrite(V3, "Power: ");
  Blynk.virtualWrite(V3, Watt);
  Blynk.virtualWrite(V3, " W\n");
  lcd.setCursor(9, 0);
      lcd.print("C");
      lcd.setCursor(5, 1);
      lcd.print(AmpsRMS);
      lcd.print("A");
      lcd.setCursor(13, 0);
      lcd.print("P");
      lcd.setCursor(11, 1);
      lcd.print(Watt);
      lcd.print("W");
      delay(2000);

  // delay(1000);

    if (voltage > 240)
 {
   
  digitalWrite(in4,LOW);
  buzzer();
  lcd.clear();
  lcd.setCursor(0,0);
  // lcd.print("Voltage: ");
  // lcd.print(outputVolts);
  // lcd.print("Volts");
  // lcd.setCursor(0,1);
  lcd.print("..Over Voltage..");
  delay(1000);
  Blynk.virtualWrite(V3, "Over Voltage");
  Blynk.virtualWrite(V3, "\n");
 }
  else if(voltage >=160 && voltage <=259)
 {
   
  digitalWrite(in4,HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  // lcd.print("Voltage: ");
  // lcd.print(outputVolts);
  // lcd.print("Volts");
  // lcd.setCursor(0,1);
  lcd.print("..Normal Voltage..");
   delay(1000);
   Blynk.virtualWrite(V3, "Normal Voltage");
  Blynk.virtualWrite(V3, "\n");
    }

    else if(voltage >80 && voltage <=160)

 {
   digitalWrite(in4,LOW);
   buzzer();
   lcd.clear();
  lcd.setCursor(0,0);
  // lcd.print("Voltage: ");
  // lcd.print(outputVolts);
  // lcd.print("Volts");
  // lcd.setCursor(0,1);
  lcd.print("..Under Voltage..");
   delay(1000);
   Blynk.virtualWrite(V3, "Under Voltage");
  Blynk.virtualWrite(V3, "\n");
 }
}

float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4095;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 4000) //sample for 1 Sec
   {
       readValue = analogRead(currentSensor);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4095.0;
      
   return result;
 }


 void buzzer() {
  ledcWriteTone(0,800);
  delay(1000);
  uint8_t octave = 1;
  ledcWriteNote(0,NOTE_C,octave);  
  delay(1000);
}

int distance(int inputVoltage)
{
  Serial.println(inputVoltage); 
  if (inputVoltage >= 3500 && inputVoltage < 3600)
  {
    return 8; 
    } 
    else if (inputVoltage >= 3200 && inputVoltage < 3400)
    {
      return 6; 
      }
      else if (inputVoltage >= 2600 && inputVoltage < 3100)
      {
        return 4; 
        }
        else if (inputVoltage >= 2400 && inputVoltage < 2500)
        {
          return 2; 
          }
          else return 0 ;
          }

void over_head()
{
  lcd.clear();
   digitalWrite(phase[0], HIGH);
                 delay(500);
                int dist1 = distance(analogRead(36)); 
                if (dist1 == 0)
                {
                  lcd.setCursor(0, 0); 
                  lcd.print("R:"); 
                  lcd.setCursor(3, 0); 
                  lcd.print("NF");
                  } 
                  else
                  {
                    lcd.setCursor(0, 0); 
                    lcd.print("R:"); 
                    lcd.setCursor(3, 0);
                    lcd.print(dist1); 
                    lcd.setCursor(4, 0); 
                    lcd.print("KM");
                    Blynk.virtualWrite(V2, "R: ");
  Blynk.virtualWrite(V2, dist1);
  Blynk.virtualWrite(V2, " KM\n");
                    }
                    digitalWrite(phase[0], LOW);
                    //================================================ 
                    digitalWrite(phase[1], HIGH); 
                    delay(500); 
                    int dist2 = distance(analogRead(36));
                    if (dist2 == 0)
                    {
                      lcd.setCursor(8, 0); 
                      lcd.print("Y:");
                      lcd.setCursor(11, 0); 
                      lcd.print("NF");
                      }
                      else
                      {
                        lcd.setCursor(8, 0);
                        lcd.print("Y:"); 
                        lcd.setCursor(11, 0);
                        lcd.print(dist2); 
                        lcd.setCursor(12, 0);
                        lcd.print("KM");
                        Blynk.virtualWrite(V2, "Y: ");
  Blynk.virtualWrite(V2, dist2);
  Blynk.virtualWrite(V2, " KM\n");
                    }
                      
                        digitalWrite(phase[1], LOW); 
                        //================================================= 
                        digitalWrite(phase[2], HIGH);
                        delay(500);
                        int dist3 = distance(analogRead(36)); 
                        if (dist3 == 0)
                        {
                          lcd.setCursor(0, 1); 
                          lcd.print("B:");
                          lcd.setCursor(3, 1); 
                          lcd.print("NF");
                          }
                          else {
                            lcd.setCursor(0, 1);
                            lcd.print("B:");
                            lcd.setCursor(3, 1);
                            lcd.print(dist3); 
                            lcd.setCursor(4, 1);
                            lcd.print("KM");
                            Blynk.virtualWrite(V2, "B: ");
  Blynk.virtualWrite(V2, dist3);
  Blynk.virtualWrite(V2, " KM\n");
                    }
                  
                            digitalWrite(phase[2], LOW);
                            delay(500);
              }
