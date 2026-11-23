/*
  =============================================================================
  MEDIDOR DE pH COM COMPENSAÇÃO TÉRMICA AUTOMÁTICA (ATC)
  =============================================================================

  Descrição:
  Este firmware realiza a leitura de um sensor de pH analógico e de um sensor
  de temperatura DS18B20.

  A calibração do pH é feita em dois pontos e a variação da sensibilidade do
  eletrodo com a temperatura é compensada com base na Equação de Nernst.

  Calibração atual:
    pH 7,0 -> ADC 755
    pH 4,0 -> ADC 857
    Temperatura de calibração -> 25,0 °C
  =============================================================================
*/

#include <OneWire.h>
#include <DallasTemperature.h>

// -----------------------------------------------------------------------------
// AJUSTE RÁPIDO PARA AS OFICINAS
// -----------------------------------------------------------------------------
// Coloque o eletrodo na referência de pH 7, observe o valor "ADC pH" no
// Monitor Serial e substitua APENAS o valor abaixo por essa leitura.
const int LEITURA_BRUTA_ZERO_PH = 755;

// -----------------------------------------------------------------------------
// PINOS
// -----------------------------------------------------------------------------
#define PIN_PH A1
#define PIN_TEMPERATURA 2

// -----------------------------------------------------------------------------
// CONFIGURAÇÕES
// -----------------------------------------------------------------------------

// Temperatura em que os pontos de calibração do pH foram obtidos.
const float TEMP_CALIBRACAO_C = 25.0;

// Correção opcional do DS18B20.
// Exemplo: se ele medir 0,5 °C acima do valor de referência, usar -0.5.
const float CORRECAO_TEMPERATURA_C = 0.0;

// Quantidade de amostras usadas na média da leitura analógica do pH.
const int NUM_LEITURAS_PH = 10;

// Intervalo entre as amostras do ADC, em milissegundos.
const unsigned long INTERVALO_AMOSTRAS_MS = 15;

// Intervalo entre ciclos completos de medição.
const unsigned long INTERVALO_MEDICOES_MS = 2000;

// -----------------------------------------------------------------------------
// CALIBRAÇÃO DO SENSOR DE pH
// -----------------------------------------------------------------------------

// Ponto 1
const float PH_PONTO_1 = 7.0;
const int LEITURA_BRUTA_PONTO_1 = 755;

// Ponto 2
const float PH_PONTO_2 = 4.0;
const int LEITURA_BRUTA_PONTO_2 = 857;

// -----------------------------------------------------------------------------
// SENSOR DE TEMPERATURA
// -----------------------------------------------------------------------------

OneWire oneWireBus(PIN_TEMPERATURA);
DallasTemperature sensors(&oneWireBus);

// -----------------------------------------------------------------------------
// FUNÇÕES
// -----------------------------------------------------------------------------

// Faz a média de várias leituras analógicas para reduzir ruído.
int lerPHBruto() {
  long soma = 0;

  for (int i = 0; i < NUM_LEITURAS_PH; i++) {
    soma += analogRead(PIN_PH);
    delay(INTERVALO_AMOSTRAS_MS);
  }

  return (int)(soma / NUM_LEITURAS_PH);
}


// Lê o DS18B20 e verifica erros básicos de comunicação/faixa.
bool lerTemperatura(float &temperaturaC) {
  sensors.requestTemperatures();

  float leitura = sensors.getTempCByIndex(0);

  // DEVICE_DISCONNECTED_C normalmente corresponde a -127 °C.
  if (leitura == DEVICE_DISCONNECTED_C || leitura < -55.0 || leitura > 125.0) {
    return false;
  }

  temperaturaC = leitura + CORRECAO_TEMPERATURA_C;
  return true;
}


// Calcula o pH usando a calibração de dois pontos e corrige a sensibilidade
// do eletrodo de acordo com a temperatura absoluta (Equação de Nernst).
float calcularPHComATC(int leituraBruta, float temperaturaC) {

  // Inclinação da reta obtida na temperatura de calibração.
  // Unidade aproximada: pH / unidade de ADC.
  const float coefAngularCalibracao =
      (PH_PONTO_2 - PH_PONTO_1) /
      (float)(LEITURA_BRUTA_PONTO_2 - LEITURA_BRUTA_PONTO_1);

  // A relação de Nernst usa temperatura absoluta (Kelvin).
  const float tempCalibracaoK = TEMP_CALIBRACAO_C + 273.15;
  const float tempAtualK = temperaturaC + 273.15;

  /*
    A sensibilidade elétrica do eletrodo em mV/pH é proporcional à temperatura
    absoluta T.

    Como aqui usamos a relação inversa, pH/ADC, a inclinação deve variar como
    1/T. Portanto:

      m(T) = m(Tcal) * Tcal / T
  */
  const float coefAngularCorrigido =
      coefAngularCalibracao * (tempCalibracaoK / tempAtualK);

  /*
    Mantém o ponto de pH 7 como referência da reta e recalcula o intercepto
    para a inclinação correspondente à temperatura atual.
  */
  const float coefLinearCorrigido =
      PH_PONTO_1 - coefAngularCorrigido * LEITURA_BRUTA_PONTO_1;

  const float valorCalibrado =
      coefAngularCorrigido * leituraBruta + coefLinearCorrigido;

  const float valorReferenciaZero =
      coefAngularCorrigido * LEITURA_BRUTA_ZERO_PH + coefLinearCorrigido;

  return valorCalibrado + (PH_PONTO_1 - valorReferenciaZero);
}


// Calcula o pH usando somente a reta original de calibração, sem aplicar
// compensação de temperatura à sensibilidade do eletrodo.
float calcularPHSemCorrecao(int leituraBruta) {

  const float coefAngular =
      (PH_PONTO_2 - PH_PONTO_1) /
      (float)(LEITURA_BRUTA_PONTO_2 - LEITURA_BRUTA_PONTO_1);

  const float coefLinear =
      PH_PONTO_1 - coefAngular * LEITURA_BRUTA_PONTO_1;

  const float valorCalibrado = coefAngular * leituraBruta + coefLinear;
  const float valorReferenciaZero =
      coefAngular * LEITURA_BRUTA_ZERO_PH + coefLinear;

  return valorCalibrado + (PH_PONTO_1 - valorReferenciaZero);
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  sensors.begin();
  sensors.setResolution(12);

  Serial.println(F("=============================================="));
  Serial.println(F(" MEDIDOR DE pH COM ATC - INICIALIZADO"));
  Serial.println(F("=============================================="));
  Serial.println(F("Temperatura de calibracao: 25.0°C"));
  Serial.println();
}

// -----------------------------------------------------------------------------
// LOOP PRINCIPAL
// -----------------------------------------------------------------------------

void loop() {

  const unsigned long inicioCiclo = millis();

  // 1. Leitura da temperatura
  float temperaturaC;

  if (!lerTemperatura(temperaturaC)) {
    Serial.println(F("ERRO: nao foi possivel obter uma leitura valida do DS18B20."));
    Serial.println(F("Verifique alimentacao, conexoes e resistor pull-up do sensor."));
    Serial.println();

    const unsigned long duracaoCiclo = millis() - inicioCiclo;
    if (duracaoCiclo < INTERVALO_MEDICOES_MS) {
      delay(INTERVALO_MEDICOES_MS - duracaoCiclo);
    }
    return;
  }

  // 2. Leitura média do sensor de pH
  int valorBrutoPH = lerPHBruto();

  // 3. Cálculo do pH sem compensação de temperatura
  float valorPHSemCorrecao = calcularPHSemCorrecao(valorBrutoPH);

  // 4. Cálculo do pH com compensação térmica da resposta do eletrodo
  float valorPHCorrigido = calcularPHComATC(valorBrutoPH, temperaturaC);

  // 5. Exibição dos resultados
  // O horário é adicionado automaticamente pelo Serial Monitor da Arduino IDE.
  Serial.print(F("pH corrigido: "));
  Serial.print(valorPHCorrigido, 2);
  Serial.print(F(" | pH: "));
  Serial.print(valorPHSemCorrecao, 2);
  Serial.print(F(" | Temperatura: "));
  Serial.print(temperaturaC, 2);
  Serial.print(F("°C | ADC pH: "));
  Serial.println(valorBrutoPH);

  // Garante aproximadamente uma linha de medicao por segundo.
  // O ciclo inclui o tempo gasto na leitura do DS18B20 e na media do pH.
  const unsigned long duracaoCiclo = millis() - inicioCiclo;
  if (duracaoCiclo < INTERVALO_MEDICOES_MS) {
    delay(INTERVALO_MEDICOES_MS - duracaoCiclo);
  }
}
