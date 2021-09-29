//******************************************************************************************
//Universidad del Valle de Guatemala
//BE3015: Electronica Digital 2
//Estefany Eleuteria Batz Cantor
//Laboratorio 4. Pantalla LCD
//******************************************************************************************

//******************************************************************************************
//Librerias
//******************************************************************************************
#include "esp_adc_cal.h"
#include <LiquidCrystal.h>


//******************************************************************************************
//Definición de pines
//******************************************************************************************
//Potenciometros
#define Poten1 34
#define Poten2 35

//Botones
#define BotonA 32
#define BotonD 36

#define RS 13
#define E 12
#define D4 14
#define D5 27
#define D6 26
#define D7 25

//******************************************************************************************
//Prototipo de funciones
//******************************************************************************************
void Potenciometros(void);
float ReadVoltaje(int ADC_Raw);
void CPU(void);
void ValoresPantalla(void);

//******************************************************************************************
//Variables Globales
//******************************************************************************************
//Potenciometros
float ADCPoten1 = 0.0;    //Valor ADC del potenciometro
double VoltajePoten1 = 0; //Para el filtro

float ADCPoten2 = 0.0;
double VoltajePoten2 = 0;

double alpha = 0.06;
//Botones
int ValorContador = 0;

//Para la pantalla LCD
LiquidCrystal LCD(RS, E, D4, D5, D6, D7);
uint8_t P1Decena, P1Unidad, P1Decimal; //Para el primer potenciometro
uint8_t P2Decena, P2Unidad, P2Decimal; //Para el primer potenciometro
uint8_t Decena, Unidad, Decimal; //Para el primer potenciometro  
//******************************************************************************************
//Configuracion
//******************************************************************************************
void setup()
{
  Serial.begin(9600);
  pinMode(BotonA, INPUT_PULLDOWN);
  pinMode(BotonD, INPUT_PULLDOWN);

  Potenciometros();
  CPU();

  LCD.begin(16, 2);
}

//******************************************************************************************
//Loop principal
//******************************************************************************************
void loop()
{
  Potenciometros();
  CPU();
  ValoresPantalla();
  Serial.println(ValorContador);
  LCD.setCursor(0,0);
  LCD.print("P1");
  LCD.setCursor(0,1);
  LCD.print(P1Decena);
  LCD.print(".");
  LCD.print(P1Unidad);
  LCD.print(P1Decena);
  LCD.print("V");

  LCD.setCursor(7,0);
  LCD.print("P2");
  LCD.setCursor(7,1);
  LCD.print(P2Decena);
  LCD.print(".");
  LCD.print(P2Unidad);
  LCD.print(P2Decena);
  LCD.print("V");

  LCD.setCursor(14, 0);
  LCD.print("CPU: ");
  LCD.setCursor(14, 1);
  LCD.print(Decena);
  LCD.print(Unidad);
  LCD.print(Decimal);
  
}

//******************************************************************************************
//Funcion del contador
//******************************************************************************************
void CPU(void)
{
  //Loop del contador************
  int EstadoBotonD = digitalRead(BotonD); //Se lee el estado actual del boton de decremento
  int EstadoBotonI = digitalRead(BotonA);
  //Incremento
  if (EstadoBotonI == 1)
  {
    ValorContador++;
    delay(300);
    if (ValorContador > 15)
    {
      ValorContador = 0;
    }
  }
  //Decremento
  if (EstadoBotonD == 1)
  {
    ValorContador--;
    delay(300);
    if (ValorContador < 1)
    {
      ValorContador = 15;
    }
  }
}

//******************************************************************************************
//Configuracion de la senal ADC
//******************************************************************************************
float ReadVoltage(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//******************************************************************************************
//ADC potenciometros
//******************************************************************************************
void Potenciometros(void)
{
  ADCPoten1 = ((analogReadMilliVolts(Poten1)*5)/3150)*10;
  VoltajePoten1 = (alpha * ADCPoten1) +((1.0 - alpha) * VoltajePoten1);
  Serial.print("Valor P1  ");
  Serial.println(VoltajePoten1);

  ADCPoten2 = ((analogReadMilliVolts(Poten2)*5)/3150)*10;
  VoltajePoten2 = (alpha * ADCPoten2) +((1.0 - alpha) * VoltajePoten2);
  Serial.print("Valor P2  ");
  Serial.println(VoltajePoten2);
}

//******************************************************************************************
//Valores para la pantalla
//******************************************************************************************
void ValoresPantalla(void)
{
  P1Decena = VoltajePoten1/10;
  Serial.println(P1Decena);
  //Pot1 = Pot1 - (P1Decena * 10.0);
  P1Unidad = VoltajePoten1 - (P1Decena*10);
  Serial.println(P1Unidad);
  //Pot1 = Pot1 - P1Unidad * 10.0;
  P1Decimal = (VoltajePoten1*10)-(P1Decena*100)-(P1Unidad*10);
  Serial.println(P1Decimal);

  P2Decena = VoltajePoten2/10;
  P2Unidad = VoltajePoten2 - (P2Decena*10);
  P2Decimal = (VoltajePoten2*10)-(P2Decena*100)-(P2Unidad*10);

  Decena = ValorContador/100;
  Unidad = ValorContador - (Decena*10);
  Decimal = (ValorContador*10)-(Decena*100)-(Unidad*10);
  
  }
