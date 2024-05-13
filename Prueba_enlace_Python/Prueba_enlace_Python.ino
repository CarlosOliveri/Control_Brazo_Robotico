#include <String.h>

int led = 13;

String msg;
void setup() {
  // put your setup code here, to run once:
  pinMode(led,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0 ){
    msg = Serial.readStringUntil('\n');
    Serial.println(msg);
    if(msg == "1"){
      digitalWrite(led,HIGH);
      //Serial.println(msg);
    }else{
      digitalWrite(led,LOW);
      //Serial.println(msg);
    }
  }
}
