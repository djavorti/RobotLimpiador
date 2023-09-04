#include <Arduino.h>
#include <NewPing.h>    // Importar bibliotecas
#include "SSD1306Wire.h" // Incluye la biblioteca de Adafruit para el controlador de la pantalla OLED
#include "BluetoothSerial.h"

// Declaración de funciones
void manualMode();
void automaticMode();
void moveRight();
void moveForward();
void moveLeft();
void moveBackward();
void moveStop();
int readSensor_R();
int readSensor_L();
int readSensor_M();

// Variables relacionadas con OLED
#define OLED_ADDR   0x3c
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    16
SSD1306Wire  display(OLED_ADDR, OLED_SDA, OLED_SCL);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error ¡El Bluetooth no está habilitado! Por favor, ejecuta `make menuconfig` y habilítalo.
#endif
BluetoothSerial SerialBT;

const int echo_L = 12;   
const int trig_L = 14;
const int echo_M = 5;
const int trig_M = 18;
const int echo_R = 2;
const int trig_R = 4;

const int freq = 30000;
const int canalPWM1 = 0;
const int canalPWM2 = 1;
const int canalPWM3 = 2;
const int canalPWM4 = 3;
const int resolucion = 8;

const int L1 = 27;
const int L2 = 26;
const int R1 = 25;
const int R2 = 33;

const int button = 32;
const int pump = 23;
int motor_speed = 100;
int max_distance = 200;
int distance_L = 0;
int distance_M = 0;
int distance_R = 0;
unsigned long tiempo_actual;
unsigned long tiempo_anterior = 0;
unsigned long delta_tiempo;
char incomingByte;

NewPing sonar_L(trig_L, echo_L, max_distance);
NewPing sonar_M(trig_M, echo_M, max_distance);
NewPing sonar_R(trig_R, echo_R, max_distance);

void initOLED() {
   pinMode(OLED_RST, OUTPUT);
   digitalWrite(OLED_RST, LOW);
   delay(20);
   digitalWrite(OLED_RST, HIGH);
}

void showOLEDMessage(String line1, String line2, String line3) {
   display.init();
   display.setFont(ArialMT_Plain_16);
   display.drawString(0, 0, line1);
   display.drawString(0, 20, line2);
   display.drawString(0, 40, line3);
   display.display();
}

void setup()
{ 
  SerialBT.begin("Limpiador"); 
  Serial.println("El dispositivo ha iniciado, ahora puedes emparejarlo por Bluetooth.");
  
  ledcSetup(canalPWM1, freq, resolucion);
  ledcSetup(canalPWM2, freq, resolucion);
  ledcSetup(canalPWM3, freq, resolucion);
  ledcSetup(canalPWM4, freq, resolucion);
  ledcAttachPin(L1, canalPWM1);
  ledcAttachPin(L2, canalPWM2);
  ledcAttachPin(R1, canalPWM3);
  ledcAttachPin(R2, canalPWM4);
  
  pinMode(button, INPUT_PULLUP);
  pinMode(pump, OUTPUT);
  ledcWrite(canalPWM1, 0);
  digitalWrite(pump, LOW);
  delay(200);
  
  initOLED();
  Serial.begin(115200);
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(40, 15, "Hola");
  display.setFont(ArialMT_Plain_10);
  display.drawString(25, 40, "Bienvenido/a!");
  display.display();
  delay(2000);
}

void loop()
{
  if(digitalRead(button) == LOW)
  {
    display.clear();
    display.drawString(10, 5, "Manual");
    display.drawString(27, 40, "Modo");
    display.display();
    Serial.print("MODO MANUAL");
    Serial.print('\n');
    while(true)
    {
      manualMode();
      if(digitalRead(button) == HIGH)
      {
        moveStop();
        break;
      }
    }
    delay(100);
  }
  else
  {
    display.clear();
    display.drawString(27, 5, "Auto");
    display.drawString(27, 40, "Modo");
    display.display();
    delay(1000);
    tiempo_anterior = millis();
    while(true)
    {
      automaticMode();
      Serial.print("MODO AUTOMÁTICO");
      Serial.print('\n');
      if(digitalRead(button) == LOW)
      {
        moveStop();
        break;
      }
    }
    delay(100);
  }
}

void manualMode()
{
  if (SerialBT.available()) {
    incomingByte = SerialBT.read();
    SerialBT.write(incomingByte);
  }

  switch(incomingByte)
  {
    case 'F':
    moveForward();
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Adelante");
    display.display();
    incomingByte='*';
    break;
    
    case 'B':
    moveBackward();
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Atrás");
    display.display();
    incomingByte='*';
    break;
    
    case 'L':
    moveLeft();
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Izquierda");
    display.display();
    incomingByte='*';
    break;
    
    case 'R':
    moveRight();
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Derecha");
    display.display();
    incomingByte='*';
    break;
    
    case 'S':
    moveStop();
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Detenido");
    display.display();
    incomingByte='*';
    break;
    
    case 'P':
    digitalWrite(pump, HIGH);
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Bomba ON");
    display.display();
    incomingByte='*';
    break;
    
    case 'p':
    digitalWrite(pump, LOW); 
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Bomba OFF");
    display.display();
    delay(500);
    display.clear();
    display.drawString(35,0,"Modo Manual");
    display.drawString(15,25,"Detenido");
    display.display();
    incomingByte='*';
    break;
    delay(5000);
  }
}

void automaticMode()
{
  distance_L = readSensor_L();
  distance_M = readSensor_M();
  distance_R = readSensor_R();
  display.clear();
  display.drawString(35,0,"Modo Automático");
  display.drawString(0,5, "L= "+String(distance_L)+"cm");
  display.drawString(0,15, "M= "+String(distance_M)+"cm");
  display.drawString(0,25, "R= "+String(distance_R)+"cm");
  display.display();
  
  Serial.print('\n');
  Serial.print(distance_L);
  Serial.print('\n');
  Serial.print(distance_M);
  Serial.print('\n');
  Serial.print(distance_R);
  Serial.print('\n');
  Serial.print('\n');
  Serial.print('\n');
  
  tiempo_actual = millis();
  delta_tiempo = tiempo_actual - tiempo_anterior;
  Serial.println(delta_tiempo);

  if(delta_tiempo > 10000){
    moveStop();
    Serial.print("BOMBA ON");
    digitalWrite(pump, HIGH);
    display.clear();
    display.drawString(35,0,"Modo Automático");
    display.drawString(40,20,"STOP");
    display.drawString(15,35,"BOMBA ON");
    display.display();
    delay(2000);
    digitalWrite(pump, LOW);
    display.clear();
    display.drawString(35,0,"Modo Automático");
    display.drawString(40,20,"STOP");
    display.drawString(15,35,"BOMBA OFF");
    display.display();
    tiempo_anterior = millis();
  }
  
  if(distance_M <= 20)
  {
    if(distance_R > distance_L)
    {
      if((distance_R <= 20) && (distance_L <= 20))
      {
        moveStop();
        delay(200);
        moveBackward();
        delay(2000);
      }
      else
      {
        moveBackward();
        delay(500);
        moveRight();
        delay(2000);
      }
    }
    else 
    if(distance_R < distance_L)
    {
      if((distance_R <= 20) && (distance_L <= 20))
      {
        moveStop();
        delay(200);
        moveBackward();
        delay(2000);
      }
      else
      {
        moveBackward();
        delay(500);
        moveLeft();
        delay(2000);
      }
    }
  }
  else 
  if(distance_R <= 15)
  {
    moveLeft();
    delay(500);
  }
  else
  if(distance_L <= 15)
  {
    moveRight();
    delay(500);
  }
  else
  {
    moveForward();
  }
}

int readSensor_L()
{ 
  delay(70);
  int cm_L = sonar_L.ping_cm();
  if(cm_L==0)
  {
    cm_L = 250;
  }
  return cm_L;
}

int readSensor_M()
{ 
  delay(70);
  int cm_M = sonar_M.ping_cm();
  if(cm_M==0)
  {
    cm_M = 250;
  }
  return cm_M;
}

int readSensor_R()
{ 
  delay(70);
  int cm_R = sonar_R.ping_cm();
  if(cm_R==0)
  {
    cm_R = 250;
  }
  return cm_R;
}

void moveForward()
{
  Serial.print("\n");
  Serial.print("ADELANTE");
  ledcWrite(canalPWM1, 0);
  ledcWrite(canalPWM2, motor_speed);
  ledcWrite(canalPWM3, motor_speed);
  ledcWrite(canalPWM4, 0);
}

void moveBackward()
{
  Serial.print("\n");
  Serial.print("ATRÁS");
  ledcWrite(canalPWM1, motor_speed);
  ledcWrite(canalPWM2, 0);
  ledcWrite(canalPWM3, 0);
  ledcWrite(canalPWM4, motor_speed);
}

void moveLeft()
{
  Serial.print("\n");
  Serial.print("IZQUIERDA");
  ledcWrite(canalPWM1, motor_speed);
  ledcWrite(canalPWM2, 0);
  ledcWrite(canalPWM3, motor_speed);
  ledcWrite(canalPWM4, 0);
}

void moveRight()
{
  Serial.print("\n");
  Serial.print("DERECHA");
  ledcWrite(canalPWM1, 0);
  ledcWrite(canalPWM2, motor_speed);
  ledcWrite(canalPWM3, 0);
  ledcWrite(canalPWM4, motor_speed);
}

void moveStop()
{
  Serial.print("\n");
  Serial.print("DETENIDO");
  ledcWrite(canalPWM1, 0);
  ledcWrite(canalPWM2, 0);
  ledcWrite(canalPWM3, 0);
  ledcWrite(canalPWM4, 0);
}
