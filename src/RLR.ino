#include <Wire.h>
#include <LIDARLite.h>
#include <SoftwareSerial.h>
#include <math.h>

#define PIN_NUMBER 5  //Radar Pin
#define AVERAGE 2

#define LED_PIN 7   //LED PIN

SoftwareSerial mySerial(2,3);       //Bluetooth
LIDARLite myLidarLite;               //Lidar

double WEIGHT = 1000;
int test = 0;

//For Lidar
unsigned long  prev_distance = 0;
unsigned long  curr_distance = 0;

double curr_speed = 0;
unsigned long curr_time = 0;

int k = 0;


//For Radar
unsigned int doppler_div = 19;
unsigned int samples[AVERAGE];
unsigned int x;
unsigned int Ttime = 0;
unsigned int Freq = 0;


void setup()
{
  Serial.begin(9600); // Initialize serial connection to display distance readings
  while(!Serial) {
    ;
  }
  pinMode(PIN_NUMBER, INPUT);         //Set radar pin
  pinMode(LED_PIN, OUTPUT);           //Set LED pin

  
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0); // Change this number to try out alternate configurations
  mySerial.begin(9600);
  
}
double Lidar() {
  long prev_time;
  long T;
  
  prev_time = curr_time;
  curr_time = millis();
  
  T = curr_time - prev_time;

  prev_distance = curr_distance;
  curr_distance = (double)myLidarLite.distance(); 

  double distance = prev_distance - curr_distance;
  
  if(distance <= 2 && distance >= -2) {
    curr_speed = 0;
    return curr_speed;
  }

  if(distance < 0) {
    curr_speed = -distance / T;
  }
  else if(distance >= 0) {
    curr_speed = distance/T;       //  M / S
  }
  
  if (curr_distance <= 4000 && curr_speed <= 1000)
  {
    return curr_speed;

  }
  return -1;

}
double Radar() {
  noInterrupts();
  pulseIn(PIN_NUMBER, HIGH);
  unsigned int pulse_length = 0;

  for(x = 0; x < AVERAGE; x++) {
    pulse_length = pulseIn(PIN_NUMBER, HIGH);
    pulse_length += pulseIn(PIN_NUMBER, LOW);
    samples[x] = pulse_length;
   }

  interrupts();

  bool samples_ok = true;
  unsigned int nbPulsesTime = samples[0];
  for(x = 1; x<AVERAGE; x++) {
    nbPulsesTime += samples[x];
    if ((samples[x] > samples[0] * 2) || (samples[x] < samples[0] / 2)){
      samples_ok = false;
    }
  }

  if (samples_ok){
    double speed = 0;
    Ttime = nbPulsesTime / AVERAGE;
    Freq = 1000000 / Ttime;
    speed = Freq/doppler_div;
    
    return speed;
  }
  return -1;

  
}
double calculateSSD(double speed) {
  double SSD = 0;       // Meter

  double reaction_d = 0.278*speed*2.5;              
  double braking_d = speed*speed*0.039/3.4;

  SSD = reaction_d + braking_d;
  
  return SSD;
}

void loop()
{ 

  double lidar_speed = Lidar();
  //double radar_speed = Radar();

  if(lidar_speed >= 0) {
    double SSD = calculateSSD(lidar_speed);              // Meter
    double D = double(curr_distance/100) - SSD;
    
    double radian = 0;
    int degree = 0;

    
    if(D < 0) {
      radian = atan(-D/1);
      degree = (radian*180/3.14) + 90;
      
      digitalWrite(LED_PIN, HIGH);
      
    }
    else {
      radian = atan(D/1);
      degree = radian*180/3.14;
      
      digitalWrite(LED_PIN, LOW);
    }
    Serial.print("Distance : ");
    Serial.print(curr_distance);
    Serial.print("\tD : ");
    Serial.print(D);
    Serial.print("\tSSD: ");
    Serial.print(SSD);
    Serial.print("M");
    Serial.print("\tSpeed: ");
    Serial.print(lidar_speed);
    Serial.print("m/s");
    Serial.print("\tAngle: ");
    Serial.println(degree);
    
    mySerial.print(degree);
    mySerial.write("\n");
    
    delay(50);
   

    
  }
  

  
}
