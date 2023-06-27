/*
Projeto de Introdução à Engenharia de Computação 2023 (PCS3100)
Professor: Edson Satoshi Gomi
Turma: 01
SALA INTELIGENTE: SENSOR DE PRESENÇA, UMIDADE E TEMPERATURA
Grupo 11:
Henrique Maruiti          12610243
Larissa Tieri Shinohara   11213360

Pinout do Circuito com ESP32
  13 -> LED (Lâmpada)
  26 -> DHT11 Data
   2 -> Módulo Sensor de Distância Ultrassônico HC-SR04 (1) (Entrada) - Trigger
   4 -> Módulo Sensor de Distância Ultrassônico HC-SR04 (1) (Entrada) - Echo
   5 -> Módulo Sensor de Distância Ultrassônico HC-SR04 (2) (Saída) - Trigger
  18 -> Módulo Sensor de Distância Ultrassônico HC-SR04 (2) (Saída) - Echo
  25 -> Módulo De Relé De 2 Canais Com Optoacoplador 5v (Ventilador) - IN1 
  19 -> Módulo De Relé De 2 Canais Com Optoacoplador 5v (Umidificador) - IN2 
  23 -> Buzzer
  22 -> Display OLED 128x64 0.96" I2C - SCL
  21 -> Display OLED 128x64 0.96" I2C - SDA 

Pinout Virtual com o Blynk
  V0 -> DHT11 (Umidade)
  V1 -> DHT11 (Temperatura)
  V2 -> LED (Lâmpada)
  V3 -> Número de pessoas
  V4 -> Módulo De Relé De 2 Canais Com Optoacoplador 5v (Ventilador)
  V5 -> Módulo De Relé De 2 Canais Com Optoacoplador 5v (Umidificador)
  V6 -> Buzzer
*/
//Credenciais do Blynk
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL2V20vPSHg"
#define BLYNK_TEMPLATE_NAME "PCS final"
#define BLYNK_AUTH_TOKEN "97kcGlEl00Z2JVJancAkjjifNHK0wjoX"

//Bibliotecas externas
#include <WiFi.h>                 //Blynk
#include <WiFiClient.h>           //Blynk
#include <BlynkSimpleEsp32.h>     //Blynk
#include "DHT.h"                  //Dht11
#include <NewPing.h>              //Sensor de distância ultrassônico
#include <Wire.h>                 //i2C
#include <Adafruit_GFX.h>         //Display
#include <Adafruit_SSD1306.h>     //Display

/* Variáveis dos Pinos Digitais de Entrada ou Leitura */
/* Led que estaria simulando uma Lampada */
#define Led 13

/* DHT11 Sensor Temperatura e Umidade Ar - Configurações */
#define DHTPIN 26                 //Dht
#define DHTTYPE DHT11             //Tipo de Dht
DHT dht(DHTPIN, DHTTYPE);

/* HC-SR04 Módulo Sensor de Distância Ultrassônico - Configurações */
#define Sonar_Trigger_1 2         //Sonar (Entrada)
#define Sonar_Echo_1 4 
#define Sonar_Trigger_2 5         //Sonar (Saida)
#define Sonar_Echo_2 18
#define MAX_DISTANCE 20           //Distância máxima de captação
NewPing sonar1(Sonar_Trigger_1, Sonar_Echo_1, MAX_DISTANCE); // Comando para o Sensor de Distância Ultrassônico 1
NewPing sonar2(Sonar_Trigger_2, Sonar_Echo_2, MAX_DISTANCE); // Comando para o Sensor de Distância Ultrassônico 2

/* Módulo Relé 5V 2 Canais - Configurações */
#define Rele_1 25                 //Ventilador
#define Rele_2 19                 //Umidificador

/* Buzzer - Alarme */
#define Buzzer 23
#define limite 5                  //Limite de pessoas para o alarme

/* Display OLED 128x64 0.96" I2C - Configurações */
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// Wifi Settings (SSID)
char ssid[] = "Maruiti";          //Nome da rede
char pass[] = "senha123";         //Senha da rede

// Variaveis para contagem de pessoas que entram/saem do ambiente
int pessoas = 0;
bool entrada = false;
bool saida = false;
bool prevsonar1 = false;
bool prevsonar2 = false;

//Funções do Blynk
BlynkTimer timer; // Função para controlar o tempo de atualização dos dados

// Envio de informações do Sensor Dht11
void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

/*  if (isnan(h) || isnan(t)) { 
    Serial.println("Failed to read from DHT sensor!");
    return;
  }*/
  Blynk.virtualWrite(V0, h); 
  Blynk.virtualWrite(V1, t);
}

//Controle do Led
int Led_control;
BLYNK_WRITE(V2)
{   
  int Led_state = param.asInt();
  if(Led_state == 1){
    Led_control = 1;
  }
  else{
    Led_control = 0;
  }
}

// Envio de informações do Número de pessoas
void num_pessoas(){
  int num_pessoas = pessoas;
  Blynk.virtualWrite(V3, num_pessoas); 
}

//Controle do Módulo De Relé De 2 Canais Com Optoacoplador 5v (1)
int ventilador;

BLYNK_WRITE(V4)
{   
  int Rele1_state = param.asInt();
  if(Rele1_state == 1){
    ventilador = 1;
  }
  else{
    ventilador = 0;
  }
}

//Controle do Módulo De Relé De 2 Canais Com Optoacoplador 5v (1)
int umidificador;

BLYNK_WRITE(V5)
{   
  int Rele2_state = param.asInt();
  if(Rele2_state == 1){
    umidificador = 1;
  }
  else{
    umidificador = 0;
  }
}

//Buzzer
int buzzer_control;

BLYNK_WRITE(V6)
{   
  int buzzer_state = param.asInt();
  if(buzzer_state == 1){
    buzzer_control = 1;
  }
  else{
    buzzer_control = 0;
  }
}

void setup()
{
  pinMode(Led, OUTPUT);                                   // LED
  pinMode(Rele_1, OUTPUT);                                // Módulo De Relé De 2 Canais Com Optoacoplador 5v (Ventilador)
  pinMode(Rele_2, OUTPUT);                                // Módulo De Relé De 2 Canais Com Optoacoplador 5v (Umidificador)
  pinMode(Buzzer, OUTPUT);                                // Buzzer
  Serial.begin(9600);                                     // Monitor Serial
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);              // Iniciar conexão com o Display OLED 128x64 0.96" I2C
  display.clearDisplay();                                 // Limpar a tela do Display OLED 128x64 0.96" I2C
  display.display();                                      // Enviar os comandos para o Display OLED 128x64 0.96" I2C
  dht.begin();                                            // Iniciar o Dht11
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);              // Conexão com o Blynk
  timer.setInterval(500L, sendSensor);                    // Tempo de Atualização das informações do sensor de Temperatura e Umidade Dht11
  timer.setInterval(200L, num_pessoas);                   // Tempo de Atualização das informações do número de pessoas na sala
}

void loop(){
  bool entrada = false;
  bool saida = false;

  // LED - Controle das condições de iluminação
  if(pessoas > 0){
    if(Led_control == 1){
      digitalWrite(Led, HIGH);
    }
    if(Led_control == 0){
      digitalWrite(Led, LOW);
    }
  }
  else{
    digitalWrite(Led, LOW);
  }

  // Dht11 - Leitura de Dados
  float temperatura = dht.readTemperature();  // Lê a temperatura em Celsius
  float umidade = dht.readHumidity();        // Lê a umidade relativa

  //Rele
  if(ventilador == 1){
    digitalWrite(Rele_1, LOW);
    if(temperatura < 17){
      digitalWrite(Rele_1, HIGH);
    }
  }
  else{
    digitalWrite(Rele_1, HIGH);
  }

  if(umidificador == 1){
    digitalWrite(Rele_2, LOW);
    if(umidade > 80){
    digitalWrite(Rele_2, HIGH);
  }
  }
  else{
    digitalWrite(Rele_2, HIGH);
  }                                                                                                       

  //Módulo Sensor de Distância Ultrassônico HC-SR04 (1) (Entrada)
  if(prevsonar1 == false){
    if(sonar1.ping_cm() > 0){
      prevsonar1 = true;
    }
    }
  if(prevsonar1 == true && prevsonar2 == false){
    entrada = true;
    saida = false;
  }

  //Módulo Sensor de Distância Ultrassônico HC-SR04 (2) (Saída)
  if(prevsonar2 == false){
    if(sonar2.ping_cm() > 0){
      prevsonar2 = true;
    }
    }
  if(prevsonar1 == false && prevsonar2 == true){
    entrada = false;
    saida = true;
  }

  //Contagem de Pessoas
  if(prevsonar1 == true && prevsonar2 == true){
    if(entrada == true && saida == false){
      pessoas++;
    }
    else{
      pessoas = pessoas - 1;
    }
    prevsonar1 = false;
    prevsonar2 = false;
    delay(1000);
  }

  //Buzzer
  if(buzzer_control == 1){
    if(pessoas > limite){
      digitalWrite(Buzzer,HIGH);
    }
  }
  else{
    digitalWrite(Buzzer,LOW);
  }

  //Display
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Sala Inteligente");
  display.print("Temp.: ");
  display.print(temperatura);
  display.println(" C");
  display.print("Umidade: ");
  display.print(umidade);
  display.println(" %");
  display.print("Pessoas: ");
  display.print(pessoas);
  display.setCursor(0,0);
  display.display();

  // Blynk
  Blynk.run();
  timer.run();
}
