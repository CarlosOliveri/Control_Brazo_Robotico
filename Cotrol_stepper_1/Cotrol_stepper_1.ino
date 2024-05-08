// Include the AccelStepper Library
#include <AccelStepper.h>


int posicion;
int velocidad;
/* float pasos = 200.0;
float gradosApasos = (pasos/360.0)*5.0;
 */
const int dirPin = 36;
const int stepPin = 28;
int sensor = 43;



AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);// works for a4988 (Bipolar, constant current, step/direction driver)

void setup() {
  // set the maximum speed, acceleration factor,
  // and the target position
  pinMode(sensor,INPUT);
  Serial.begin(9600);
  myStepper.setMaxSpeed(10000.0);
  myStepper.setAcceleration(50.0);
}
bool isSetPosition = false;
bool isSetPositioned = false;
void loop() {

  /* if(!isSetPosition){
    float a = 360*44.44;//(*3200*6/360);
    int b = (int)a;
    float stepSeg = 20*44.44;
    velocidad = (int)stepSeg;
    // Mueve el motor a la posición deseada
    myStepper.moveTo(b);
    myStepper.setSpeed(velocidad);
    Serial.print(b);
    Serial.print("  ");
    Serial.println(velocidad);
    isSetPosition = true;
  }

  while(!isSetPositioned){
    myStepper.runSpeedToPosition();
    if(!digitalRead(sensor)){
      myStepper.stop();
      Serial.println(myStepper.currentPosition());
      myStepper.setCurrentPosition(myStepper.currentPosition());
      Serial.println(myStepper.currentPosition());
      myStepper.setSpeed(0);
      isSetPositioned = true;
      Serial.println("Cero Encontrado");
    }      
  }
  Serial.println(myStepper.currentPosition()); */

  // Verifica si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    // Lee el comando ingresado desde el monitor serial
    String command = Serial.readStringUntil('\n');

    parseCommand(command);
    
    // Convierte el comando a un número entero (posición en pasos)
    //int targetPosition = command.toInt();
    float a = posicion*44.44;//(*3200*6/360);
    int b = (int)a;
    float stepSeg = velocidad*44.44;
    velocidad = (int)stepSeg;
    // Mueve el motor a la posición deseada
    myStepper.moveTo(b);
    myStepper.setSpeed(velocidad);
    Serial.print(b);
    Serial.print("  ");
    Serial.println(velocidad);
  }
  // Llama a run() en cada iteración del loop para mover el motor hacia la posición deseada
  myStepper.runSpeedToPosition();
}

void parseCommand(String cmd) {
  int commaIndex = cmd.indexOf(','); // Find the index of the comma
  if (commaIndex != -1) { // Check if comma is found
    String variable1String = cmd.substring(0, commaIndex); // Extract the first part of the command
    String variable2String = cmd.substring(commaIndex + 1); // Extract the second part of the command
    posicion = variable1String.toInt(); // Convert the first part to integer
    velocidad = variable2String.toInt(); // Convert the second part to integer
  }
}