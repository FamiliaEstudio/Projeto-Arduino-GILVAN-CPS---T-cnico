// Inclusão das bibliotecas necessárias
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Servo.h>

// Definição dos pinos para o display TFT
#define TFT_CS   10
#define TFT_RST  8
#define TFT_DC   9

// Definição dos pinos para o sensor ultrassônico
#define TRIG_PIN 7
#define ECHO_PIN 6

// Definição do pino para o servo motor
#define SERVO_PIN 11

// Criação dos objetos para o display e o servo
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Servo radarServo;

// Variáveis para o cálculo da distância
long duration;
int distance;

void setup() {
  // Inicializa a comunicação serial para depuração
  Serial.begin(9600);

  // Inicializa o display TFT
  tft.begin();
  tft.setRotation(3); // Ajusta a rotação da tela conforme a montagem

  // Configura os pinos do sensor ultrassônico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Anexa o servo ao pino definido
  radarServo.attach(SERVO_PIN);

  // Desenha a interface estática do radar
  drawRadarInterface();
}

void loop() {
  // Varredura de 15 a 165 graus (evita forçar o servo nos limites)
  for (int angle = 15; angle <= 165; angle++) {
    radarServo.write(angle);
    updateRadar(angle);
    delay(30); // Pausa para o servo se mover
  }

  // Varredura de volta de 165 a 15 graus
  for (int angle = 165; angle >= 15; angle--) {
    radarServo.write(angle);
    updateRadar(angle);
    delay(30); // Pausa para o servo se mover
  }
}

// Função para medir a distância
int measureDistance() {
  // Limpa o pino Trig
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Envia um pulso de 10us para disparar o sensor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Lê o tempo de retorno do pulso no pino Echo
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calcula a distância em cm
  distance = duration * 0.034 / 2;
  return distance;
}

// Função para atualizar a tela do radar
void updateRadar(int angle) {
  int distance = measureDistance();
  
  // Limpa a área de varredura (apaga objetos antigos)
  tft.fillScreen(ILI9341_BLACK);
  drawRadarInterface();

  // Desenha a linha de varredura
  drawRadarLine(angle);

  // Se um objeto for detectado dentro do alcance (ex: 40 cm)
  if (distance > 0 && distance < 40) {
    drawObject(angle, distance);
  }
}

// Função para desenhar a linha de varredura
void drawRadarLine(int angle) {
  int centerX = tft.width() / 2;
  int centerY = tft.height() - 10;
  float angleRad = radians(angle); // Converte ângulo para radianos
  int lineLength = 100; // Comprimento da linha

  int endX = centerX + lineLength * cos(angleRad);
  int endY = centerY - lineLength * sin(angleRad);

  tft.drawLine(centerX, centerY, endX, endY, ILI9341_GREEN);
}

// Função para desenhar o objeto detectado
void drawObject(int angle, int distance) {
  int centerX = tft.width() / 2;
  int centerY = tft.height() - 10;
  float angleRad = radians(angle);
  
  // Mapeia a distância para a escala da tela
  int mappedDist = map(distance, 0, 40, 0, 100);

  int objX = centerX + mappedDist * cos(angleRad);
  int objY = centerY - mappedDist * sin(angleRad);

  tft.fillCircle(objX, objY, 3, ILI9341_RED); // Desenha um círculo vermelho
}

// Função para desenhar a interface estática do radar
void drawRadarInterface() {
  int centerX = tft.width() / 2;
  int centerY = tft.height() - 10;
  
  // Desenha os arcos concêntricos
  tft.drawCircle(centerX, centerY, 40, ILI9341_DARKGREEN);
  tft.drawCircle(centerX, centerY, 80, ILI9341_DARKGREEN);
  tft.drawCircle(centerX, centerY, 120, ILI9341_DARKGREEN);

  // Desenha as linhas de ângulo
  tft.drawLine(centerX, centerY, centerX + 120 * cos(radians(30)), centerY - 120 * sin(radians(30)), ILI9341_DARKGREEN);
  tft.drawLine(centerX, centerY, centerX + 120 * cos(radians(60)), centerY - 120 * sin(radians(60)), ILI9341_DARKGREEN);
  tft.drawLine(centerX, centerY, centerX, centerY - 120, ILI9341_DARKGREEN);
  tft.drawLine(centerX, centerY, centerX + 120 * cos(radians(120)), centerY - 120 * sin(radians(120)), ILI9341_DARKGREEN);
  tft.drawLine(centerX, centerY, centerX + 120 * cos(radians(150)), centerY - 120 * sin(radians(150)), ILI9341_DARKGREEN);
}