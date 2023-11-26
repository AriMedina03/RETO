#include <mcp_can.h>
#include <SPI.h> //no se porque estaba esa libreria pero vamos viendo4

MCP_CAN CAN(10);  // CS pin on SPI for the MCP2515


volatile unsigned muestreoActualInterrupcionI = 0; // variables para definicion del tiempo de interrupcion y calculo de la velocidad del motor derecho
volatile unsigned muestreoAnteriorInterrupcionI = 0;
volatile unsigned deltaMuestreoInterrupcionI = 0;

volatile unsigned muestreoActualInterrupcionR = 0; // variables para definicion del tiempo de interrupcion y calculo de la velocidad del motor derecho
volatile unsigned muestreoAnteriorInterrupcionR = 0;
volatile unsigned deltaMuestreoInterrupcionR = 0;

int encoderI = 3; //pin sensor velocidad llanta derecha
int encoderR = 2; //pin sensor velocidad llanta derecha

int llantaI = 6; //control de velocidad llanta izq
int entrada1I = 13; //direccion 1 llanta izq
int entrada2I = 12; //dirreccion 2 llanta izq

int llantaR = 8; //control de velocidad llanta derecha
int entrada1R = 5; //direccion 1 llanta derecha
int entrada2R = 11; //dirreccion 2 llanta derecha

double frecuenciaI = 0; //frecuencia de interrupcion llanta izquierda
double frecuenciaR = 0; //frecuencia de interrupcion llanta derecha

int Tx = 18;
int Rx = 19;

double Wr = 0; //velocidad angular derecha
double Vr = 0; //velocidad lineal derecha
double Wi = 0; //velocidad lineal Izquierda
double Vi = 0; //Velocidad lineal Izquierda
double diametro = 6.5; //cm

int N = 20; //numero de ranuras en encoder

const int sensorCenter=A2;
const int sensorIzq=A0; 
const int sensorDer=A1;
int receivedNumber = 0;

float retraso = 3.0; // seg

double pwmI = 0;
double pwmR = 0;
double pwmRR = 0;
double pwmII = 0;
int pwm = 0;
int dt = 0;
int cont = 0;

float before = 0;
float time = 0;
float nowPWM = 0;
float beforePWM = 0;


double Setpoint, InputIzq, InputDer; 
double OutputIzq, OutputDer;

bool flag = true;
//funcion de interrupcion del encoder llanda izq
void IEncoder() {
  deltaMuestreoInterrupcionI = muestreoActualInterrupcionI - muestreoAnteriorInterrupcionI; //dif tiempos de interrupciones de ticks del motor
  muestreoAnteriorInterrupcionI = muestreoActualInterrupcionI; //se actualiza el tiempo de interrupcion anterior
  if(deltaMuestreoInterrupcionI == 0){
    deltaMuestreoInterrupcionI = deltaMuestreoInterrupcionI;
  }else{
    frecuenciaI = (1000) / (double) deltaMuestreoInterrupcionI; //frecuencia de interrupcion
  }
  Wi = ((2*3.141592)/N)*frecuenciaI; // velocidad angular llanta izquierda
  if(pwmI < 1){
    Vi = 0;
  }else{
    Vi = Wi*(diametro/2); // velocidad lineal llanta izquierda
  }
  

}
  
 
void REncoder() {
  deltaMuestreoInterrupcionR = muestreoActualInterrupcionR - muestreoAnteriorInterrupcionR; //dif tiempos de interrupciones de ticks del motor
  muestreoAnteriorInterrupcionR = muestreoActualInterrupcionR; //se actualiza el tiempo de interrupcion anterior
  if(deltaMuestreoInterrupcionR == 0){
    deltaMuestreoInterrupcionR = deltaMuestreoInterrupcionR;
  }else{
    frecuenciaR = (1000) / (double) deltaMuestreoInterrupcionR; //frecuencia de interrupcion
  }
  Wr = ((2*3.141592)/N)*frecuenciaR; // velocidad angular llanta derecha
  if(pwmR < 1){
    Vr = 0;
  }else{
    Vr = Wr*(diametro/2); // velocidad lineal llanta derecha
  }
  
}
 
void setup() {

  attachInterrupt(digitalPinToInterrupt(encoderI), IEncoder, FALLING); //añadir una interrupcion a pin
  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING); //añadir una interrupcion a pin

  Serial.begin(115200);

  pinMode(entrada1I, OUTPUT);
  pinMode(entrada2I, OUTPUT);
  pinMode(llantaI, OUTPUT);

  pinMode(entrada1R, OUTPUT);
  pinMode(entrada2R, OUTPUT);
  pinMode(llantaR, OUTPUT);

  pinMode(sensorCenter, INPUT);
  pinMode(sensorIzq, INPUT);
  pinMode(sensorDer, INPUT);


  if (CAN.begin(CAN_500KBPS) == CAN_OK) {
    Serial.println("CAN Bus Init OK");
  } else {
    Serial.println("CAN Bus Init Failed");
  }
}

void loop() {
  muestreoActualInterrupcionI = millis(); //se asigna el tiempo de ejecucion al muestreo actual
  muestreoActualInterrupcionR = millis(); //se asigna el tiempo de ejecucion al muestreo actual
  int valueCenter = analogRead(sensorCenter);
  int valueDer = analogRead(sensorDer);
  int valueIzq = analogRead(sensorIzq);

  unsigned char stmp[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; //los datos a mandar. falta comunicarlo con el encoder que no se como jale xd
  CAN.sendMsgBuf(0x123, 0, 8, stmp);
  delay(1000);
  

  nowPWM = millis();
  InputIzq = Vi;
  InputDer = Vr;
  
  Serial.print("Vi:");
  Serial.print(Vi);
  Serial.print(",");
  Serial.print("Vr:");
  Serial.println(Vr);


}