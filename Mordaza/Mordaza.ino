//////////------------------------------------------------- velocidad lineal 31/05/2024+corriente+oled ok
#include <U8g2lib.h>
#include <Arduino.h>
#include <Wire.h>
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=22*/ 13, /* data=21*/ 18, /* reset=*/ U8X8_PIN_NONE); //esp32
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //ardu

float Ip=0;
float sensibilidad=0.105;                       //sensibilidad en V/A para sensor (0.105)  5A 0.185/20A 0.100/30A 0.66
float offset=0.136;                             // Equivale a la amplitud del ruido 0v 0a ---0.165  9v 0.04a ---0.186
                                                // calibracion base 0.176
int encoder = 2;                //Pin 2, donde se conecta el encoder       
unsigned int rpm = 0;           //Revoluciones por minuto calculadas.
float vel = 0;                  //Velocidad en [Km/h]
volatile int pulsos  = 0;       //Número de pulsos leidos por el Arduino en un segundo
unsigned long tant = 0;         //Tiempo anterior 
unsigned int muescas = 6;       //Número de muescas que tiene el disco del encoder.
const int diam = 23.94;         //Diámetro de la rueda pequeña[mm]
static volatile unsigned long debounce = 0; //Tiempo del rebote.
      
void setup() {
  Serial.begin(9600);
  u8g2.begin();                 //Inicializamos el display
  pinMode(encoder, INPUT); 
  attachInterrupt(digitalPinToInterrupt(encoder), inter, RISING);  
  pulsos  = 0;
  rpm = 0;
  tant = 0;
  Serial.print("Seconds ");
  Serial.print("RPM ");
  Serial.print("Pulsos  ");
  Serial.println("vel[Km/h]");
}

void loop() {
  if (millis() - tant >= 1000){                                //Se actualiza cada segundo
      noInterrupts();                                           //Desconectamos la interrupción 
      rpm = (60 * 1000 / muescas )/ (millis() - tant)* pulsos ; //Calculamos las revoluciones por minuto
      vel = rpm * 3.141592 * diam * 60 / 1000000;               //Cálculo velocidad en [Km/h] 
      tant = millis(); // Almacenamos el tiempo actual.
      Serial.print(millis()/1000); Serial.print("       ");     //Visualizar resultados
      Serial.print(rpm,DEC); Serial.print("   ");
      Serial.print(pulsos ,DEC); Serial.print("     ");
      Serial.println(vel,2); 
      pulsos  = 0;                                            //Inicializamos los pulsos.
      interrupts();                                           //Reiniciamos la interrupción
  Ip=get_corriente();                                         //obtenemos la corriente pico
  Serial.print("Ip: ");                                       //Visualizar resultados
  Serial.print(Ip,3);
  Serial.println("A");
//  float Irms=Ip*0.707;                        //Intensidad RMS = Ipico/(2^1/2)
//  float P=Irms*220.0;                         // P=IV watts
//  Serial.print(" Irms: ");
//  Serial.print(Irms,3);
//  Serial.print("A, Potencia: ");
//  Serial.print(P,3);  
//  Serial.println("W");
    oled();                                   //Función para mostrar los datos
  }     
}

float get_corriente() {     //-------------------------------Funcion leo corriente
  float voltajesensor;
  float corriente=0;
  long tiempo=millis();
  float Imax=0; 
  float Imaxt=0;
  float Imin=0; 
  float Imint=0;
  while(millis()-tiempo<500)  {                  //realizamos mediciones durante 0.5 segundos
    for(int i=0;i<500;i++); {                    //se promedian 500 mediciones
    voltajesensor = analogRead(A0) * (5.0 / 1023.0);                          //lectura del sensor
    corriente=0.9*corriente+0.1*((voltajesensor-2.527)/sensibilidad);         //Ecuación  para obtener la corriente
      if(corriente>Imax){Imax=corriente;
      Imaxt+=Imax;}
      if(corriente<Imin){Imin=corriente;
      Imint+=Imin;}
    }
    Imaxt/=500;
    Imint/=500;
  }
  return(((Imax-Imin)/2)-offset);
}                         //----------------------------------Fin leo corriente

void inter(){  //------------------------------------------Funcion interrupcion
  if( digitalRead (encoder) && (micros()-debounce > 500) && digitalRead (encoder) ) { 
  //comprueba encoder tiempo es superior a 1000 microsegundos
  debounce = micros(); // Almacena el tiempo para comprobar de rebote
  pulsos ++;}  
}             //------------------------------------------Fin interrupcion

void oled(){  //------------------------------------------Funcion diaplay
  u8g2.clearBuffer();                           // Borra el contenido anterior del buffer de pantalla
  u8g2.setFont( u8g2_font_6x13_tf/*u8g2_font_t0_11_mn*/);             // Selecciona fuente
  u8g2.setCursor(2, 10);                        // Establece la posición de la posición del texto
  u8g2.print("I:");                             //Se muestra I
  u8g2.setCursor(70, 10); u8g2.print(Ip);                               //Se muestra el valor de corriente obtenido
    u8g2.setCursor(2, 20);                       //Posición de texto
    u8g2.print("Seconds ");
    u8g2.setCursor(70, 20); u8g2.print(millis()/1000);
    u8g2.setCursor(2, 40);
    u8g2.print("RPM ");
    u8g2.setCursor(70, 40); u8g2.print(rpm);
    u8g2.setCursor(2, 50);
    u8g2.print("Vel[Km/h]");
    u8g2.setCursor(70, 50); u8g2.print(vel,2);
  u8g2.sendBuffer();                            //Envia contenido   
}             //------------------------------------------Fin interrupcion
