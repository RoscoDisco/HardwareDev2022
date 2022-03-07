

// Initalise argurments
String rx_x;
String rx_y;
String tx_str;
int x, y;

// Motor pins
const int motorPin1 = 5;
const int motorPin2 = 6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){

    rx_x = Serial.readStringUntil(',');
    Serial.read();
    rx_y = (Serial.readString());
   
    x = rx_x.toInt();
    y = rx_y.toInt();

  
    if (x > 0){
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, x);  
    } else {
      analogWrite(motorPin1, -x);
      analogWrite(motorPin2, 0);  
    }
  
    tx_str = String("x: ");
    tx_str = String(tx_str + x);
    tx_str = String(tx_str + "; y: ");
    tx_str = String(tx_str + y);

    Serial.println(tx_str);
    
  }
}
