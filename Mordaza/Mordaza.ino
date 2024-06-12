//////////------------------------------------------------- velocidad lineal 31/05/2024+corriente+oled ok
#include <U8g2lib.h>
#include <Arduino.h>
#include <Wire.h>
#define seni 33
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/18 /*13*/, /* data=*/21 /*18*/, /* reset=*/ U8X8_PIN_NONE); //esp32
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //ardu

float Ip=0;
float sensibilidad=0.7;                       //sensibilidad en V/A para sensor 2.9(0.105)  (5A 0.185)/(20A 0.100)/(30A 0.66)
float offset=0.148;                             // Equivale a la amplitud del ruido 0v 0a ---0.165  9v 0.04a ---0.186
                                                // calibracion base 0.176 calibracion calculada 0.136
byte encoder = 34;               //Pin 2ardu , donde se conecta el encoder
//float seni=32;                   //Pin A0, sensor de corriente
unsigned int rpm = 0;           //Revoluciones por minuto calculadas.
float vel = 0;                  //Velocidad en [Km/h]
volatile int pulsos  = 0;       //Número de pulsos leidos por el Arduino en un segundo
unsigned long tant = 0;         //Tiempo anterior 
unsigned int muescas = 6;       //Número de muescas que tiene el disco del encoder.
const int diam = 23.94;         //Diámetro de la rueda pequeña[mm]
static volatile unsigned long debounce = 0; //Tiempo del rebote.
float T=0;
float P=0;
float Irms=0;
      
void setup() {
  Serial.begin(9600);
  u8g2.begin();                 //Inicializamos el display
  pinMode(encoder, INPUT); 
  attachInterrupt(digitalPinToInterrupt(encoder), inter, RISING);  
  pulsos  = 0;
  rpm = 0;
  tant = 0;
  Serial.print("Seconds ");
  Serial.print("Corriente ");
  Serial.print("RPM ");
  //Serial.print("Pulsos  ");
  Serial.print("Potencia: ");
  Serial.print("Torque: ");
  Serial.println("vel[Km/h]");
}

void loop() {
  if (millis() - tant >= 1000){                                //Se actualiza cada segundo
      noInterrupts();                                           //Desconectamos la interrupción 
      rpm = (60 * 1000 / muescas )/ (millis() - tant)* pulsos ; //Calculamos las revoluciones por minuto
      vel = rpm * 3.141592 * diam * 60 / 1000000;               //Cálculo velocidad en [Km/h] 
      tant = millis(); // Almacenamos el tiempo actual.
      Serial.print(millis()/1000); Serial.print("       ");     //Visualizar resultados
      Serial.print(Ip,3); Serial.print("A"); Serial.print("     ");
      Serial.print(rpm,DEC); Serial.print("   ");
      //Serial.print(pulsos ,DEC); Serial.print("     ");
      Serial.print(P,3); Serial.print("W") ;Serial.print("   ");
      Serial.print(T,3); Serial.print("Nm");Serial.print("   ");
      Serial.println(vel,2); 
      pulsos  = 0;                                            //Inicializamos los pulsos.
      interrupts();                                           //Reiniciamos la interrupción
  if (get_corriente()<=0){Ip=0.00;}
      else {Ip=get_corriente();}                                         //obtenemos la corriente pico
  Irms=Ip*0.707;                              //Intensidad RMS = Ipico/(2^1/2)
  P=Irms*120;                                 //P=IV watts Voltaje utilizado por motor cambiar dependiendo del motor
  float velang=rpm*0.10471975;                //Velocidad Angular(rad/s)=Velocidad(rpm)*2π/60
  T=P/velang;                                 //Torque(Nm)=Potencia(W)/VelocidadAngular(rad/s)
//  Serial.print(" Irms: ");                  //Para motores de CA se calcula la potencia mecanica Pm=√3*V*I*cos(φ)
//  Serial.print(Irms,3);                     //cos(φ) Factor de potencia del motor, representa la eficiencia del motor en la conversión de energía eléctrica en energía mecánica.
//  Serial.print("A");                        //Velocidad angular en rad ω (rad/s)=2π*N/60 N vel en rpm
                                              //Tca(Nm)=Pm(W)/ω(rad/s)
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
    voltajesensor = analogRead(seni)*(5.0/4096.0);                            //lectura del sensor (voltaje de trabajo del sensor/resolucion adc 1023Ardu/4096esp)
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

void oled(){  //------------------------------------------Funcion display
  u8g2.clearBuffer();                           // Borra el contenido anterior del buffer de pantalla
  u8g2.setFont( u8g2_font_6x13_tf/*u8g2_font_t0_11_mn*/);             // Selecciona fuente
  u8g2.setCursor(2, 10);                        // Establece la posición de la posición del texto
  u8g2.print("I:");                             //Se muestra I
  u8g2.setCursor(70, 10); u8g2.print(Ip);                               //Se muestra el valor de corriente obtenido
    u8g2.setCursor(2, 20);                       //Posición de texto
    u8g2.print("Seconds ");
    u8g2.setCursor(70, 20); u8g2.print(millis()/1000);
    u8g2.setCursor(2, 30);
    u8g2.print("RPM ");
    u8g2.setCursor(70, 30); u8g2.print(rpm);
    u8g2.setCursor(2, 40);
    u8g2.print("Vel[Km/h]");
    u8g2.setCursor(70, 40); u8g2.print(vel,2);
        u8g2.setCursor(2, 50);
    u8g2.print("Torque:");
    u8g2.setCursor(70, 50); u8g2.print(T,2);
  u8g2.sendBuffer();                            //Envia contenido   
}             //------------------------------------------Fin display
