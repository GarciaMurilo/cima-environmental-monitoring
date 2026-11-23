/*
  Calibração de Precisão do Sensor de Turbidez
  
  Descrição:
  Este código usa uma equação de calibração para converter a leitura bruta do
  sensor de turbidez em um valor preciso de NTU.
  
  Instruções:
  - Substitua os valores das variáveis de calibração abaixo
    com base nas suas leituras no laboratório.
  - Ponto 1: Referência para água limpa.
  - Ponto 2: Referência para água turva.
  - Faça o upload deste código para o seu Arduino.
  - Abra o "Monitor Serial" na IDE do Arduino e configure a taxa de comunicação
    para 9600 bps para visualizar as leituras precisas.
*/

// Define o pino de entrada analógica para o sensor de turbidez
#define PIN_TURBIDEZ A0

// -----------------------------------------------------------------------------
// AJUSTE RÁPIDO PARA AS OFICINAS
// -----------------------------------------------------------------------------
// Coloque o sensor na água de referência de 0 NTU, observe a "Leitura Bruta"
// no Monitor Serial e substitua APENAS o valor abaixo por essa leitura.
const int leituraBrutaZero = 780;

// Variáveis para os pontos de calibração.
// Substitua estes valores pelas suas leituras de laboratório.
const int leituraBrutaPonto1 = 780; // Leitura bruta para a água limpa
const int leituraBrutaPonto2 = 631; // Leitura bruta para a água turva

const float ntuPonto1 = 0.0;    // NTU correspondente à água limpa
const float ntuPonto2 = 810.0; // NTU correspondente à água turva

void setup() {
  // Inicia a comunicação serial para a depuração
  Serial.begin(9600);
  Serial.println("Sensor de turbidez pronto para leituras precisas.");
  Serial.println("Aguarde as leituras...");
}

void loop() {
  // --- Leitura do Sensor de Turbidez ---
  
  // Variáveis para a média
  long totalLeituraBruta = 0;
  int numLeituras = 20; // Número de leituras para a média
  
  // Lê o sensor várias vezes e soma os valores
  for (int i = 0; i < numLeituras; i++) {
    totalLeituraBruta += analogRead(PIN_TURBIDEZ);
    delay(10); // Pequeno atraso para leituras mais estáveis
  }
  
  // Calcula a média das leituras
  int valorBrutoTurbidez = totalLeituraBruta / numLeituras;
  
  // Calcula a inclinação (coeficiente angular) e a interceptação (coeficiente linear)
  // da linha de calibração com base nos seus dois pontos de referência.
  float coeficiente_angular = (ntuPonto2 - ntuPonto1) / (float)(leituraBrutaPonto2 - leituraBrutaPonto1);
  float coeficiente_linear = ntuPonto1 - coeficiente_angular * leituraBrutaPonto1;

  // Converte a leitura bruta para NTU usando a sua equação de calibração
  // NTU = (coeficiente_angular * LeituraBruta) + coeficiente_linear
  float valorTurbidezCalibrado = coeficiente_angular * valorBrutoTurbidez + coeficiente_linear;
  float valorReferenciaZero = coeficiente_angular * leituraBrutaZero + coeficiente_linear;
  float valorTurbidezNTU = valorTurbidezCalibrado - valorReferenciaZero;
  
  // --- Exibição no Monitor Serial ---
  
  Serial.print("Leitura Bruta: ");
  Serial.print(valorBrutoTurbidez);
  Serial.print(" | Turbidez: ");
  Serial.print(valorTurbidezNTU, 2); 
  Serial.println(" NTU");

  // Aguarda 2 segundos antes de fazer a próxima leitura
  delay(2000); 
}
