//#include <Arduino.h>
//#include <U8g2lib.h>
//#include <Wire.h>
//
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
//
//void setup(void) {
//
//  u8g2.begin();
//}
//
//void drawLogo(void){
//    u8g2.setFontMode(1);  // Transparent
//    u8g2.setFontDirection(0);
//    u8g2.setFont(u8g2_font_inb16_mf);
//    u8g2.drawStr(2, 22, "U");
//    u8g2.setFontDirection(1);
//    u8g2.setFont(u8g2_font_inb19_mn);
//    u8g2.drawStr(14,8,"8");
//    u8g2.setFontDirection(0);
//    u8g2.setFont(u8g2_font_inb16_mf);
//    u8g2.drawStr(36,22,"g");
//    u8g2.drawStr(48,22,"\xb2");
//}
//
//void loop(void) {
//  u8g2.clearBuffer();
//  drawLogo();
//  u8g2.sendBuffer();
//  delay(1000);
//}
////------------------------------------------------------------------------------------//27/05/2024

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);

//float sensibilidad=0.185;   //sensibilidad para sensor 5 Amperes
float sensibilidad=0.100;   //sensibilidad para sensor 20 Amperes
//float sensibilidad=0.66;    //sensibilidad para sensor 30 Amperes

//float sensibilidad=0.11;      //sensibilidad en V/A para nuestro sensor 0.194 sensor de 20A
float offset=0.092;           // Equivale a la amplitud del ruido
float iPin = 34;        //Sensor IO34
//float seni=0;
float Ip =0;                  //corriente pico
float Irms =0;               //Intensidad RMS = Ipico/(2^1/2)
float P =0;                 // P=IV watts

void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.clearDisplay();
}

void loop() {
  
  float Ip =get_corriente();//obtenemos la corriente pico
  float Irms =Ip*0.707; //Intensidad RMS = Ipico/(2^1/2)
  float P =Irms*220.0; // P=IV watts
  Serial.print("Ip: ");
  Serial.print(Ip,3);
  Serial.print("A , I-Rms: ");
  Serial.print(Irms,3);
  Serial.print("A, Potencia: ");
  Serial.print(P,3);  
  Serial.println("W");
//  oled();             //mostrar en display
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_13_tf);
  u8g2.setCursor(2, 10);
  u8g2.print("Ip: ");
  u8g2.print(Ip,3);
  u8g2.print("A");

  u8g2.setCursor(2, 20);
  u8g2.print("I-Rms: ");
  u8g2.print(Irms,3);
  u8g2.print("A");
  
  u8g2.setCursor(2, 30);
  u8g2.print("Potencia: ");
  u8g2.print(P,2);  
  u8g2.print("W");
  u8g2.sendBuffer();
  delay(800);     
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
    voltajeSensor = analogRead(iPin) * (5.0 / 1023.0);//lectura del sensor
    corriente=0.9*corriente+0.1*((voltajeSensor-2.527)/sensibilidad); //Ecuación  para obtener la corriente
    if(corriente>Imax)Imax=corriente;
    if(corriente<Imin)Imin=corriente;
  }
  return(((Imax-Imin)/2)-offset);
}

//void oled(){
//  u8g2.clearBuffer();
//  u8g2.setFont(u8g2_font_t0_13_tf);
//  u8g2.setCursor(2, 10);
//  u8g2.print("Ip: ");
//  u8g2.print(Ip,3);
//  u8g2.print("A");
//
//  u8g2.setCursor(2, 20);
//  u8g2.print("I-Rms: ");
//  u8g2.print(Irms,3);
//  u8g2.print("A");
//  
//  u8g2.setCursor(2, 30);
//  u8g2.print("Potencia: ");
//  u8g2.print(P,3);  
//  u8g2.print("W");
//  u8g2.sendBuffer();
//}
//----------------------
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
//Descomentar según el sensor usado ya sea de 5, 20 0 30 Amperes
//float sensibilidad=0.185; //Ajuste de la sensibilidad para sensor de 5 Amperes
//float sensibilidad=0.100; //Ajuste de la sensibilidad para sensor de 20 Amperes
float sensibilidad=0.66; //Ajuste de la sensibilidad para sensor de 30 Amperes
float I=0.00;
float ajuste = 0.05;//Sólo si es necesario, se usa para corregir posibles desvios en la lectura que se comparará con un amperímetro
float iPin = 34; 
void setup() {
  Serial.begin(9600);//Velocidad del puerto serial
  u8g2.begin(); //Inicializamos el display
}
void loop() { 
  I=promedio_I(500);//Promedio de 500 muestras para mejorar la presición(llamamos a la función promedio_I()
  Serial.print("Intencidad: ");
  if(I>=0.01){//Filtro para eliminar pequeñas oscilasciones(ruido)
  I= ((I*10)+ajuste),2;
  Serial.println(I);//Imprime el valor de la corriente consumida
oled();//Función para mostrar los datos obtenidos en el display  
  delay(100); 
  }else{
  Serial.println("0.00");
I = 0.00; 
oled();//Función para mostrar en el display 0.00
  delay(100);   
  }
}
//Función para generar 500 muestas
float promedio_I(int muestras_I)
{
  float sensorA0;
  float intencidad=0;
  for(int i=0;i<muestras_I;i++)
  {
    sensorA0 = analogRead(iPin) * (5.0 / 1023.0);//Leemos el sensor de corriente
    intencidad=(0.8*intencidad)+(sensorA0-2.5)/sensibilidad; //Calculo para obtener el valor de la corriente
  }
  intencidad=intencidad/muestras_I;//dividimos por 500 
  return(intencidad);
}
//Función para mostrar datos en el display oled
void oled(){
  u8g2.clearBuffer(); // Borra el contenido anterior del buffer de pantalla
  u8g2.setFont(u8g2_font_t0_13_tf); // Selecciona una fuente grande
  u8g2.setCursor(0, 40); // Establece la posición de la posición del texto
  u8g2.print("I:");//Se muestra I: en el display
  u8g2.print(I);//Se muestra el valor de corriente obtenido
  u8g2.setCursor(60, 40); // Establece la posición de la posición del texto
  u8g2.print("A");//Se muestra A en el display
  u8g2.sendBuffer(); // Envia el contenido del buffer a la pantalla   
  }


  float Sensibilidad=0.105; //sensibilidad en Voltios 0.100 para 20A

void setup() {
  
  Serial.begin(9600);
}

void loop() {
  
  float I=get_corriente(500);//obtenemos la corriente promedio de 500 muestras 
  Serial.print("Corriente: ");
  Serial.println(I,3); 
  delay(100);     
}

float get_corriente(int n_muestras)
{
  float voltajeSensor;
  float corriente=0;
  for(int i=0;i<n_muestras;i++)
  {
    voltajeSensor = analogRead(A0) * (5.0 / 1023.0);////lectura del sensor
    corriente=corriente+(voltajeSensor-2.5)/Sensibilidad; //Ecuación  para obtener la corriente
  }
  corriente=corriente/n_muestras;
  return(corriente); -0.03
} //calibracion 2.496   1a 2.391 1.05 2.388   2.391-2.496 =-0.105

//void setup() {
//  
//  Serial.begin(9600);
//}
//
//void loop() {
//  
//  float voltajeSensor =get_voltage(10000);//obtenemos voltaje del sensor(10000 muestras) 
//  Serial.print("Voltaje del sensor: ");
//  Serial.println(voltajeSensor ,3);     
//}
//
//
//float get_voltage(int n_muestras)
//{
//  float voltage=0;
//  
//  for(int i=0;i<n_muestras;i++)
//  {
//    voltage =voltage+analogRead(A0) * (5.0 / 1023.0);    
//  }
//  voltage=voltage/n_muestras;
//  return(voltage);
//}
//calibracion 29/05/2024

//sensor corriente

float sensibilidad=0.105;                       //sensibilidad en V/A para nuestro sensor
float offset=0.176;                             // Equivale a la amplitud del ruido 0v 0a ---0.165  9v 0.04a ---0.186

void setup() {
  
  Serial.begin(9600);
}

void loop() {
  
  float Ip=get_corriente();                     //obtenemos la corriente pico
//  float Irms=Ip*0.707;                        //Intensidad RMS = Ipico/(2^1/2)
//  float P=Irms*220.0;                         // P=IV watts
  Serial.print("Ip: ");
  Serial.print(Ip,3);
  Serial.println("A");
//  Serial.print(" Irms: ");
//  Serial.print(Irms,3);
//  Serial.print("A, Potencia: ");
//  Serial.print(P,3);  
//  Serial.println("W");
  delay(500);     
}

float get_corriente()
{
  float voltajesensor;
  float corriente=0;
  long tiempo=millis();
  float Imax=0;
  float Imin=0;
  while(millis()-tiempo<500)                  //realizamos mediciones durante 0.5 segundos
  { 
    voltajesensor = analogRead(A0) * (5.0 / 1023.0);                          //lectura del sensor
    corriente=0.9*corriente+0.1*((voltajesensor-2.527)/sensibilidad);         //Ecuación  para obtener la corriente
    if(corriente>Imax)Imax=corriente;
    if(corriente<Imin)Imin=corriente;
  }
  return(((Imax-Imin)/2)-offset);
}

////////// velocidad lineal 31/05/2024

int encoder = 2;                //Pin 2, donde se conecta el encoder       
unsigned int rpm = 0;           //Revoluciones por minuto calculadas.
float vel = 0;                  //Velocidad en [Km/h]
volatile int pulsos  = 0;       //Número de pulsos leidos por el Arduino en un segundo
unsigned long tant = 0;         //Tiempo anterior 
unsigned int muescas = 6;       //Número de muescas que tiene el disco del encoder.
const int diam = 23.94;         //Diámetro de la rueda pequeña[mm]
static volatile unsigned long debounce = 0; //Tiempo del rebote.

void setup(){
   Serial.begin(9600);   
   pinMode(encoder, INPUT); 
   attachInterrupt(digitalPinToInterrupt(encoder), inter, RISING);  
   pulsos  = 0;
   rpm = 0;
   tant = 0;
  Serial.print("Seconds ");
  Serial.print("RPM ");
  Serial.print("Pulsos  ");
  Serial.println("vel[Km/h]");}

void loop(){
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
   }
} //Fin loop


void inter(){  //Funcin interrupcion
  if( digitalRead (encoder) && (micros()-debounce > 500) && digitalRead (encoder) ) { 
        //comprueba encoder tiempo es superior a 1000 microsegundos
      debounce = micros(); // Almacena el tiempo para comprobar de rebote
      pulsos ++;}  
}
