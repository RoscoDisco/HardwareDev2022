#include<Encoder.h>

// Initalise argurments
// RX variables
String rx_x;
String rx_y;

// TX Variables
String tx_str = "No Data sent yet";

// Position variables
int m1_desired, m2_desired;
float lamda = 1;
int velocity = 150;

// encoder information
int PPR= 14;
int gearRatio = 298;
Encoder m1_encoder(2,7);
Encoder m2_encoder(3,4);
float m1_position = 0;
float m2_position = 0;

// Motor pins
const int mp1_1 = 6;
const int mp1_2 = 5;
const int mp2_1 = 10;
const int mp2_2 = 9;
const int m1_enable = 12;
const int m2_enable = 13;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  Serial.print("System Setup");
  

  pinMode(mp1_1, OUTPUT);
  pinMode(mp1_2, OUTPUT);
  pinMode(mp2_1, OUTPUT);
  pinMode(mp2_2, OUTPUT);
  pinMode(m1_enable, OUTPUT);
  pinMode(m2_enable, OUTPUT); 

  // Enable the motors
  digitalWrite(m1_enable, HIGH);
  digitalWrite(m2_enable, HIGH);
  

  Serial.print("Motors Initiated");
}

void loop() {
  // Read the encoder position
  long new_pos = m1_encoder.read();
  m1_position = (float)new_pos*360/(PPR*gearRatio*2);
  new_pos = m2_encoder.read();
  m2_position = (float)new_pos*360/(PPR*gearRatio*2);
  
  // Read data from serial
  while(Serial.available()){

    rx_x = Serial.readStringUntil(',');
    Serial.read();
    rx_y = (Serial.readString());
   
    m1_desired = rx_x.toInt();
    m2_desired = rx_y.toInt();

    
    tx_str = String("M1: ");
    tx_str = String(tx_str + m1_position);
    tx_str = String(tx_str + "; M2: ");
    tx_str = String(tx_str + m2_position);
    Serial.print(tx_str);

    
  }

  if (m1_position < m1_desired - lamda){
    analogWrite(mp1_1, velocity);
    analogWrite(mp1_2, 0);
  } else if (m1_position > m1_desired + lamda){
    analogWrite(mp1_1, 0);
    analogWrite(mp1_2, velocity);
  } else {
    analogWrite(mp1_1, 0);
    analogWrite(mp1_2, 0);
  }

  
  if (m2_position < m2_desired - lamda){
    analogWrite(mp2_1, velocity);
    analogWrite(mp2_2, 0);
  } else if (m2_position > m2_desired + lamda){
    analogWrite(mp2_1, 0);
    analogWrite(mp2_2, velocity);
  } else {
    analogWrite(mp2_1, 0);
    analogWrite(mp2_2, 0);
  }

}
