# Global_Solution-Edge

## 👥 Integrantes:

| Nome            | RM       |
|-----------------|----------|
| Eduardo Francisco Mauro Gonçalves | RM561969 |
| Gabriel Luchetta dos Santos | RM561861 |

## 🔗 Link do Vídeo: [(https://youtu.be/SUhQ9RGlrMo)]
## 🔗 Wokwi Project Link: https://wokwi.com/projects/447457884276495361

# Smart Work Assistant — IoT para Bem-Estar e Produtividade

## Resumo Detalhado:
Projeto interdisciplinar inspirado no tema “O Futuro do Trabalho”, que propõe o desenvolvimento de um dispositivo inteligente baseado em ESP32 voltado ao bem-estar e à produtividade no ambiente laboral.

A solução realiza o monitoramento das condições ambientais — como temperatura, umidade, luminosidade e presença — e utiliza esses dados para sugerir pausas estratégicas e emitir alertas de ergonomia, promovendo um equilíbrio saudável entre desempenho e qualidade de vida no trabalho.

O sistema integra-se a dashboards e aplicativos externos por meio de protocolos MQTT e HTTP, possibilitando a análise em tempo real, a visualização remota e a automação de decisões relacionadas ao conforto e à eficiência no espaço de trabalho do futuro.

## Funcionalidades:
- Leitura de DHT22 (temperatura/umidade), LDR (luminosidade) e PIR (presença).
- Lógica de decisão para alertas:
  - Temperatura alta (> 28°C)
  - Humidade alta (> 70%)
  - Baixa luminosidade (LDR acima do threshold)
  - Sem pausas por 45 minutos continuous
- Alertas locais (LED + buzzer) e envio de telemetria/alerta via MQTT e/ou HTTP.
- Configurável via constantes no código.

## Arquivos:
- `SmartWorkAssistant.ino` — código principal (ESP32, Arduino)
- `README.md` — esta documentação
- `wokwi/` — (opcional) link para projeto Wokwi / imagens do circuito

## Como rodar (Wokwi):
1. Abra Wokwi (https://wokwi.com) e crie novo projeto com ESP32.
2. Adicione os componentes: DHT22, PIR, LDR (com resistor), LED, Buzzer.
3. Cole o código `SmartWorkAssistant.ino` no editor.
4. Ajuste `WIFI_SSID`, `WIFI_PASS`, `MQTT_BROKER` e `HTTP_ENDPOINT` se desejar testar rede.
5. Start → observe logs no Serial e interaja com sensores virtuais.

## MQTT / HTTP
- MQTT Telemetry topic: `smartworkassistant/telemetry`
- MQTT Alert topic: `smartworkassistant/alerts`
- HTTP endpoint: `POST /api/telemetry` (ex.: https://meuservidor/telemetry)

Payload JSON:
```json
{
  "device_id": "esp32_swa_001",
  "timestamp": "boot_secs_12345",
  "temp_c": 27.8,
  "hum_pct": 55.2,
  "ldr_raw": 512,
  "presence": 1,
  "timeActiveSec": 2700,
  "alert": null
}
