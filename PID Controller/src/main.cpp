#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>

#define BLYNK_PRINT SerialUSB
#include <Blynk.h>
//#include <BlynkSimpleWiFiShield101.h> //uncomment for wifi connection
#include <BlynkSimpleSerialBLE.h> //uncomment for BLE connection

///////////////////////Blynk Variables//////////////////////
char auth[] = "zQR0v3kaPt-5QaGTjO-eNckoIXq82dEp";
bool kp_manual = 0,ki_manual=0,kd_manual=0;
float KP_MAN, KI_MAN, KD_MAN; //manual values for PID constants
BlynkTimer timer;
////////////////////////////////////////////////////////////

///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////


////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0,voltage,adc;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////

//////////////Communication with Other arduino///////////
int voltage_out_pin = A1;
int distance_out_pin = A2;
/////////////////////////////////////////////////////////

///////////////////PID constants///////////////////////
float kp=1.5;
float ki = 0.2;
float kd=3100;
float distance_setpoint = 20;    //distance to set ball from sensor
float PID_p, PID_i, PID_d, PID_total;
float startLoop, endLoop;
///////////////////////////////////////////////////////

//Function prototypes
float map_to_float(float x, float a, float b, float c, float d);
float get_dist(int n);

void setup() { 
  Serial1.begin(9600); //Setup serial stream for BLE connection through Serial1 (13,14)[Rx,Tx]
  SerialUSB.begin(9600); //setup serial stream for USB debugging
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(125); //Put the servco at angle 125, so the balance is in the middle
  pinMode(Analog_in,INPUT);  //Define input pin as an input for servo input
  time = millis(); //Initialize current time

  //Blynk stuff
  //Blynk.begin(auth,ssid,pass); //uncomment for wifi connection
  Blynk.begin(Serial1,auth); //uncomment for BLE connection
}

void loop() {
  Blynk.run(); //start blynk checking for updates
  timer.run(); //start blynk checking for interrupts

  if (millis() > time+period){ //only loop if a period of time has passed
    time = millis(); //reset begin time for each loop    
    distance = get_dist(50);   
    distance_error = distance_setpoint - distance;   
    PID_p = (kp_manual ? KP_MAN : kp) * distance_error; //proportional correction
    float dist_difference = distance_error - distance_previous_error;
    PID_d = (kd_manual ? KD_MAN : kd)*((dist_difference)/period); //differential correction

    //integral correction only works when close to target as this is only for small adjustments
    if(-3 < distance_error && distance_error < 3){
      PID_i = PID_i + ((ki_manual ? KI_MAN : ki ) * distance_error);
    }
    else{
      PID_i = 0;
    }

    PID_total = PID_p + PID_i + PID_d; //Sum up total correction
    PID_total = map(PID_total, -150, 150, 0, 150); //Map PID correction to an angle on the servo
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
  
    myservo.write(PID_total+0); //PID total correction plus a correction constant
    distance_previous_error = distance_error;

  }
}

float map_to_float(float x, float a, float b, float c, float d){
      float f=x/(b-a)*(d-c)+c;
      return f;
}

float get_dist(int n){
  long sum=0;
  for(int i=0;i<n;i++)
  {
    sum=sum+analogRead(Analog_in);
  }  
  adc=sum/n;
  
  voltage = adc*3.3/1024;
  float distance_cm = 5461/(adc-17)-2; //linear fit to trend of ADC value vs distance
  if(distance_cm > 80) distance_cm = 81;
  if(distance_cm < 10) distance_cm = 9;

  //Debugging lines for sensor accuracy
  String outputValues = String("Start,"+String(kp_manual ? KP_MAN : kp )+','+String(ki_manual ? KI_MAN : ki)+','+String(kd_manual ? KD_MAN : kd )+','+String(voltage)+','+String(adc)+','+String(distance_cm)+','+String(millis()));
  SerialUSB.println(outputValues);

  return(distance_cm);
}

//Blynk interupts to change values
BLYNK_WRITE(V0){
  if(kp_manual){
  KP_MAN = param.asFloat();// assigning incoming value from pin V1 to a variable
  } 
}
BLYNK_WRITE(V1){
  if(kd_manual){
  KD_MAN = param.asFloat(); // assigning incoming value from pin V1 to a variable
  }
}
BLYNK_WRITE(V2){
  if(ki_manual){
  KI_MAN = param.asFloat(); // assigning incoming value from pin V1 to a variable
  }
}
BLYNK_WRITE(V3){
  kp_manual = param.asInt(); //toggle variable for manual override of Kp
}
BLYNK_WRITE(V4){
  ki_manual = param.asInt(); //toggle variable for manual override of Ki
}
BLYNK_WRITE(V5){
  kd_manual = param.asInt(); //toggle variable for manual override of Kd
}
