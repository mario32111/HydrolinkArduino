#include <WiFi.h>
#include <HTTPClient.h> // Librería para manejar solicitudes HTTP

// Credenciales WiFi
const char* ssid = "INFINITUM7z8t";       // Nombre de tu red WiFi
const char* password = "963ffc39a2";      // Contraseña de tu red WiFi

// URL de la API
const char* apiUrl = "http://shielded-meadow-09877-903acd4fabd4.herokuapp.com/api/v1/products";

void setup() {
  Serial.begin(115200);
  pinMode(5, OUTPUT); // Configurar el pin 5 como salida (para el LED)

  // Conexión a la red WiFi
  Serial.println("\nConectando a WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado.");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Verificar conexión WiFi
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Iniciar la conexión HTTP
    http.begin(apiUrl); // Configurar la URL de la API
    Serial.println("\nEnviando solicitud GET a la API...");

    // Hacer la solicitud GET
    int httpResponseCode = http.GET();

    // Procesar la respuesta
    if (httpResponseCode > 0) {
      Serial.print("Código de respuesta HTTP: ");
      Serial.println(httpResponseCode);

      // Leer el contenido de la respuesta
      String payload = http.getString();
      Serial.println("Respuesta de la API:");
      Serial.println(payload);

      // Control del LED en función de la respuesta
      if (payload.indexOf("ledOn") != -1) {
        digitalWrite(5, HIGH); // Encender el LED si la respuesta contiene "ledOn"
        Serial.println("LED encendido.");
      } else if (payload.indexOf("ledOff") != -1) {
        digitalWrite(5, LOW); // Apagar el LED si la respuesta contiene "ledOff"
        Serial.println("LED apagado.");
      }
    } else {
      Serial.print("Error en la solicitud HTTP: ");
      Serial.println(httpResponseCode);
    }

    // Finalizar conexión HTTP
    http.end();
  } else {
    Serial.println("No hay conexión WiFi.");
  }

  delay(10000); // Esperar 10 segundos antes de enviar otra solicitud
}
