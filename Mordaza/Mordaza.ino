volatile int  contador = 0;   // Variable entera que se almacena en la RAM del Micro
int           velpot =0;
const int     ledcal=13;
int           senalto=1023;
int           senbajo=0;

 
void setup() {
  Serial.begin(57600);
  pinMode(ledcal, OUTPUT);
  digitalWrite(ledcal,HIGH);
  attachInterrupt(0,interrupcion0,RISING);  // Interrupcion 0 (pin2) LOW, CHANGE, RISING, FALLING
  while (millis()<5000){
    velpot = analogRead (A0);
    if (velpot<senalto){
      senalto=velpot;
    }
     if (velpot>senbajo){
      senbajo=velpot;   
   }
 }
digitalWrite(ledcal,LOW);
}         
 
void loop() {
  velpot=analogRead(A0);
  int pitch = map(velpot,senbajo, senalto, 80, 1000);
 // analogWrite(9, pitch);
  
  delay(999);               // retardo de casi 1 segundo
  Serial.print(contador*(60/8)); // Como son 8 interrupciones por vuelta (contador * (60/8))
  Serial.println(" RPM");    //  El numero 8 depende del numero aspas de la helise del motor en prueba
  Serial.println(velpot);
  contador = 0;
}
 
void interrupcion0()    // Funcion que se ejecuta durante cada interrupion
{
  contador++;           // Se incrementa en uno el contador
}
////////////////////////////////////////////********
#include "BluetoothSerial.h"                                                     //Envio por BLE de RPM --> Km/h 
#define vel 32                                                                   //D2 Wroom =IO32 mini Wroom sensor detector de las interrupciones D2 con Res=1k
#define pi 0.37699111843077518861551720599354                                    //pi*3/25
#define radio 0.072                                                             //radio de circunferencia
                                  

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define led        33                                                           //GPIO33 Control led
Adafruit_NeoPixel pixel(1, led, NEO_GRB + NEO_KHZ800);

BluetoothSerial SerialBT;

//Variables
const float volt25=25; //GPIO25 Lectura de voltaje
float       volt=0;    //numero de vueltas
float       volt2=0;

byte        rojo=0;
byte        verde=0;
byte        azul=0;

double      velo=0;
volatile int  cont = 0;                                                         
String dato = "";  
bool findato = false;                                                           // si el string esta completo

int a=0;                                                                        //Seleccion de opcion

void setup() {
  SerialBT.begin(115200);   Serial.begin(115200);                               // inicio bluetooth // inicio serial
  SerialBT.begin("EntrenadorBMX");                                              // Nombre dispositivo
  SerialBT.println("Conexion disponible");                                      //Serial.println("Conexion disponible");
  //dato2.reserve(200);
  dato.reserve(200);                                                            //Guardo 200 bytes para datos de llegada
  pinMode(vel, INPUT);                                                          //RPM
  attachInterrupt(digitalPinToInterrupt(vel), inter, RISING);
}
void loop() {
  pixel.setBrightness(10);
  SerialBT.println("Durante cuanto tiempo se va a realizar la medición ");      //Pregunta por tiempo
  //Serial.println("Durante cuanto tiempo se va a realizar la medición ");
  SerialBT.setTimeout(100);
  
  volt=analogRead(volt25);                                                      //Medicion de voltaje
  volt2=map(volt, 0, 4095, 0, 33);
  volt2/=10;
  Serial.print(volt);Serial.print('\t'); Serial.println(volt2);
 
  if(volt2<2.5) {
  rojo=255;
  pixel.setPixelColor(0, rojo, verde,  azul);//rojo,verde,azul 0-255
  pixel.show();
  }
   if((volt2>=2.6)&&(volt2<=2.8)) {
  rojo=255;
  verde=128;
  pixel.setPixelColor(0, rojo, verde,  azul);//rojo,verde,azul 0-255
  pixel.show();
  }
   if(volt2>=2.9) {
  verde=255;
  pixel.setPixelColor(0, rojo, verde,  azul);//rojo,verde,azul 0-255
  pixel.show();
  } 
       
  delay(500); 
  //serialEvent();
 if(SerialBT.available()){
  dato = SerialBT.readString();
  a=dato.toInt();
//  Serial.println(a);
//  Serial.println(dato);
    if(a>0){
    SerialBT.println("Inicia en: " );//Serial.println("Inicia en: " );
    for (int i=10;i>=0;i--){                                                  //Cuenta atras
                SerialBT.println(i);//Serial.println(i);
                delay(999);
            }
    for (int i=0;i<=a;i++){                                                   //AQUI ------>>>>>>>>>Hago monitoreo por a segundos<<<<<<---------72mm*0.072m
//--------------------------------------------------------------------------
            delay(999);
            cont*=7.5;                                                        // Como son 8 interrupciones por vuelta (contador * (60/8)=7.5)
            velo=pi*radio*cont;                                               // Conversion de rpm a km/h
            velo;
            SerialBT.print(velo);    //Serial.print(velo);                    //velocidad km/h BLE
            SerialBT.println(" Km/h");  //Serial.println(" Km/h");
            cont=0;
//-----------------------------------------------------------------------------          
            }
          SerialBT.println("Fin de medición.");
          a=0;
          dato="";          //limpiar el dato
          findato = false;
       }
 } 
}
//---------------------------------------------------------------------------------
void inter(){                                                                 // Funcion que se ejecuta durante cada interrupion
  cont ++;
}
//------------------------------------------------------------------------------------
