#define BLYNK_TEMPLATE_ID "TMPL6XvmEvtrk"
#define BLYNK_TEMPLATE_NAME "Gas Detector"
#define BLYNK_AUTH_TOKEN "1yEVhZIHkQ1cZFt6J4IIiDkCyNU9e0g1"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char ssid[] = "KuongTiFu";          //Tên WiFi thật
char pass[] = "77777777";           //Mật khẩu WiFi thật


BlynkTimer timer;

String uartBuffer = "";
int lastSentPPM = -1;

void setup()
{
  Serial.begin(9600);       // UART to STM32
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(500L, processUartData); // Kiểm tra UART mỗi 0.5s
}

void loop()
{
  Blynk.run();
  timer.run();
}

void processUartData()
{
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      uartBuffer.trim();

      if (uartBuffer.startsWith("ALERT")) {
        Blynk.logEvent("gas_warning", "Gas leak detected!");
      }
      else if (uartBuffer.startsWith("PPM:")) {
        int ppm = uartBuffer.substring(4).toInt();
        if (ppm != lastSentPPM) {
          Blynk.virtualWrite(V0, ppm);
          lastSentPPM = ppm;
        }
      }
      uartBuffer = "";
    } else {
      uartBuffer += c;
    }
  }
}

