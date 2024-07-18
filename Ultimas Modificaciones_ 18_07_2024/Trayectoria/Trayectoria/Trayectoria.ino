#include "Motor.h"

// Articulacion 1
#define dirPinA1 36
#define stepPinA1 28
#define sensorPin1 42
#define stepsPerUnit1 44.4444//2.7777

// // Articulacion 2
#define dirPin2A 34
#define stepPin2A 26
#define dirPin2B 32
#define stepPin2B 24
#define sensorPin2 44
#define stepsPerUnit2 270

//Articulacion 3
 #define dirPin3 30
 #define stepPin3 22
 #define sensorPin3 46
 #define stepsPerUnit3 275

//Articulacion 4
#define dirPin4 31
#define stepPin4 23
#define sensorPin4 48
#define stepsPerUnit4 17.7777

//Articulacion 5
#define dirPin5 33
#define stepPin5 25
#define sensorPin5 43
#define stepsPerUnit5 8.2//17.7777

//Articulacion 6 NO SE USA
#define dirPin6 35
#define stepPin6 27
#define sensorPin6 47
#define stepsPerUnit6 44.4444//17.7777

//Articulacion 7 NO SE USA
#define dirPin7 37
#define stepPin7 29
#define sensorPin7 47
#define stepsPerUnit7 44.4444

float q1,q2,q3,q4,q5,q6;
float speed,speed1,speed2,speed3;
int Type;
int b1 = 0;
int b2 = 0;
int b3 = 0;
int bandera = 0;
//int motorTarget;

//Para movimiento coordinado
void coordinado (float a1, float a2, float a3);

Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);
Articulacion articulacion3(dirPin3,stepPin3,sensorPin3,stepsPerUnit3);
Articulacion articulacion4(dirPin4,stepPin4,sensorPin4,stepsPerUnit4);
Articulacion articulacion5(dirPin5,stepPin5,sensorPin5,stepsPerUnit5);
Articulacion articulacion6(dirPin6,stepPin6,sensorPin6,stepsPerUnit6);
Articulacion articulacion7(dirPin7,stepPin7,sensorPin7,stepsPerUnit7);

void setup() {
  pinMode(8,OUTPUT);
  digitalWrite(8,0);
  Serial.begin(9600);
  //Serial.println("Listo");
  //Debemos anhadir la busqueda al cero 
  //delay(1000);
  //articulacion2.Art2Ofsset();
  //articulacion1.Art360Ofsset();
  //articulacion3.Art3Ofsset();
  //articulacion4.Art360Ofsset();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    speed = 10;
    if(Type == 1){
      articulacion1.moveTo(q1, speed);
      articulacion2.moveTo(q2, speed);
      articulacion3.moveTo(q3, speed);
    }else if(Type == 2){
      coordinado(q1,q2,q3);
      articulacion1.moveTo(q1, speed1);
      articulacion2.moveTo(q2, speed2);
      articulacion3.moveTo(q3, speed3);
    }
    
    //delay(10);
    b1 = 0;
    b2 = 0;
    b3 = 0;
  }
  articulacion1.motor1.runSpeedToPosition();
  articulacion2.motor1.runSpeedToPosition();
  articulacion2.motor2.runSpeedToPosition();
  articulacion3.motor1.runSpeedToPosition();
  if(articulacion1.motor1.currentPosition() <= q1*stepsPerUnit1+1 && articulacion1.motor1.currentPosition() >= q1*stepsPerUnit1-1 /*&& articulacion1.motor1.currentPosition() != 0*/ && b1==0){/*Serial.println("Articulacion 1 Lista")*/;b1=1; }
  if(articulacion2.motor1.currentPosition() <= q2*stepsPerUnit2+1 && articulacion2.motor1.currentPosition() >= q2*stepsPerUnit2-1 /*&& articulacion2.motor1.currentPosition() != 0*/ && b2==0){/*Serial.println("Articulacion 2 Lista")*/;b2=1; }
  if(articulacion3.motor1.currentPosition() <= q3*stepsPerUnit3+1 && articulacion3.motor1.currentPosition() >= q3*stepsPerUnit3-1 /*&& articulacion3.motor1.currentPosition() != 0*/ && b3==0){/*Serial.println("Articulacion 3 Lista")*/;b3=1; }
  if(b1 == 1 && b2 == 1 && b3 == 1){
    Serial.println("Listo");
    while(true){
      if(Serial.available() > 0){
        break;
      }
    }
  }
}

void parseCommand(String cmd) {
  int commaIndex[3];
  int i = 0;
  int lastCommaIndex = -1;

  // Encontrar todas las comas
  while (i < 3) {
    int nextCommaIndex = cmd.indexOf(",", lastCommaIndex + 1);
    if (nextCommaIndex == -1) break;
    commaIndex[i] = nextCommaIndex;
    lastCommaIndex = nextCommaIndex;
    i++;
  }

  if (i == 3) { // Asegurarse de que se encontraron exactamente 6 comas
    String q1String = cmd.substring(0, commaIndex[0]);
    String q2String = cmd.substring(commaIndex[0] + 1, commaIndex[1]);
    String q3String = cmd.substring(commaIndex[1] + 1, commaIndex[2]);
    String speedString = cmd.substring(commaIndex[2] + 1);

    q1 = q1String.toInt(); 
    q2 = q2String.toInt(); 
    q3 = q3String.toInt(); 
    Type = speedString.toInt();

    /*Serial.print(q1);
    Serial.print(",");
    Serial.print(q2);
    Serial.print(",");
    Serial.print(q3);
    Serial.print(",");
    Serial.println(Type);*/

    if (int(Type) < 1 || int(Type) > 2) {
      Serial.println("Tipo de movimiento Invalido");
      return;
    }
    else if (abs(q1) >360 || abs(q2)>90 || abs(q3)>90) {
      Serial.println("Anguo Inavalido");
      return;}

  } else {
    Serial.println("[Comando Invalido]");
  }
}

void coordinado (float a1, float a2, float a3){
  int pos1,pos2,pos3;
  float max_speed = 10;
  pos1 = a1-articulacion1.motor1.currentPosition()/stepsPerUnit1;
  pos2 = a2-articulacion2.motor1.currentPosition()/stepsPerUnit2;
  pos3 = a3-articulacion3.motor1.currentPosition()/stepsPerUnit3;
  Serial.println(pos1);
  Serial.println(pos2);
  Serial.println(pos3);
  if(abs(pos1)>abs(pos2) && abs(pos1)>abs(pos3)){speed1 = max_speed; speed2 = abs(pos2)/(abs(pos1)/max_speed); speed3 = abs(pos3)/(abs(pos1)/max_speed);}
  else if(abs(pos2)>abs(pos1) && abs(pos2)>abs(pos3)){speed2 = max_speed; speed1 = abs(pos1)/(abs(pos2)/max_speed); speed3 = abs(pos3)/(abs(pos2)/max_speed);}
  else if(abs(pos3)>abs(pos2) && abs(pos3)>abs(pos1)){speed3 = max_speed; speed2 = abs(pos2)/(abs(pos3)/max_speed); speed1 = abs(pos1)/(abs(pos3)/max_speed);}
  else {speed1 = max_speed; speed2 = max_speed; speed3 = max_speed;}
}
