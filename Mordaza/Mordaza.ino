////#include "BluetoothSerial.h"                                                     //Envio por BLE de RPM --> Km/h 
////#define vel 32                                                                   //D2 Wroom =IO32 mini Wroom sensor detector de las interrupciones D2 con Res=1k
//#define pi 0.37699111843077518861551720599354                                    //pi*3/25
//#define radio 0.074                                                             //radio de circunferencia
//                                  
////#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
////#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
////#endif
////
////BluetoothSerial SerialBT;
//
////Variables
//
//int           encoder=2;
//int           rpm=0;
//volatile int  cont = 0;
//const int     intervalo = 900;                                                     
////String dato = "";  
////bool findato = false;                                                           // si el string esta completo
////int a=0;                                                                        //Seleccion de opcion
//
//void setup() {
//  /*SerialBT.begin(9600);*/                     Serial.begin(9600);                               // inicio bluetooth // inicio serial
//  /*SerialBT.begin("Don_Barril");*/             Serial.println("Don_Barril");                                             // Nombre dispositivo
//  /*SerialBT.println("Conexion disponible")*/;  Serial.println("Conexion disponible");                                    //Serial.println("Conexion disponible");
////  dato.reserve(200);                                                            //Guardo 200 bytes para datos de llegada
//  pinMode(encoder, INPUT);                                                          //RPM
//  attachInterrupt(digitalPinToInterrupt(encoder), inter, RISING);
//  Serial.print("Seg ");Serial.print('\t'); Serial.print("RPM ");Serial.print('\t'); Serial.println("Pulsos ");
//}
//void loop() {
//  delay(intervalo);
//  //SerialBT.setTimeout(100); //esperar para tomar datos
//                                                  //AQUI ------>>>>>>>>>Hago monitoreo por a segundos<<<<<<---------74mm*0.074m
////--------------------------------------------------------------------------
//            cont*=10;                                                        // Como son 6 interrupciones por vuelta (contador * (60/6)=10)
//            rpm=pi*radio*cont;                                               // Conversion de rpm a km/h
//            rpm;     
////-----------------------------------------------------------------------------          
////          a=0;
////          dato="";          //limpiar el dato
////          findato = false;
//      Serial.print(millis()/1000);Serial.print('\t');Serial.print(rpm);Serial.print('\t');Serial.print(cont,DEC);Serial.println('\t');
//      cont=0;
//      //Serial.print(pi);Serial.print(radio);
//}
////---------------------------------------------------------------------------------
//void inter(){                                                                 // Funcion que se ejecuta durante cada interrupion
//  cont ++;
//}
////------------------------------------------------------------------------------------


float Sensibilidad=0.11; //sensibilidad en V/A para nuestro sensor 0.194
float offset=0.092; // Equivale a la amplitud del ruido
void setup() {
  
  Serial.begin(9600);
}

void loop() {
  
  float Ip=get_corriente();//obtenemos la corriente pico
  float Irms=Ip*0.707; //Intensidad RMS = Ipico/(2^1/2)
  float P=Irms*220.0; // P=IV watts
  Serial.print("Ip: ");
  Serial.print(Ip,3);
  Serial.print("A , Irms: ");
  Serial.print(Irms,3);
  Serial.print("A, Potencia: ");
  Serial.print(P,3);  
  Serial.println("W");
  delay(500);     
}

float get_corriente()
{
  float voltajeSensor;
  float corriente=0;
  long tiempo=millis();
  float Imax=0;
  float Imin=0;
  while(millis()-tiempo<500)//realizamos mediciones durante 0.5 segundos
  { 
    voltajeSensor = analogRead(A0) * (5.0 / 1023.0);//lectura del sensor
    corriente=0.9*corriente+0.1*((voltajeSensor-2.527)/Sensibilidad); //Ecuación  para obtener la corriente
    if(corriente>Imax)Imax=corriente;
    if(corriente<Imin)Imin=corriente;
  }
  return(((Imax-Imin)/2)-offset);
}
