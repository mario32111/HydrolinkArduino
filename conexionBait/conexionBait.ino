#include "HardwareSerial.h"

// Configuración del UART1 para el módulo SIM A7670
HardwareSerial SIM7670(1); // UART1

// Definición de pines
#define RX_PIN 27       // RX del ESP32 (GPIO27)
#define TX_PIN 26       // TX del ESP32 (GPIO26)
#define PWRKEY 4        // Pin de encendido PWRKEY conectado al GPIO4 del ESP32

// Función para enviar comandos AT
String enviarComandoAT(String comando, int tiempoEspera = 2000) {
  Serial.print("Enviando comando: ");
  Serial.println(comando);

  SIM7670.println(comando); // Envía el comando AT al módulo
  delay(100); // Breve pausa antes de leer respuesta

  long tiempoInicio = millis();
  String respuesta = "";

  // Leer la respuesta durante el tiempo de espera
  while (millis() - tiempoInicio < tiempoEspera) {
    while (SIM7670.available()) {
      char c = SIM7670.read();
      respuesta += c;
    }
  }

  // Imprimir la respuesta recibida
  if (respuesta.length() > 0) {
    Serial.println("Respuesta:");
    Serial.println(respuesta);

    // Verifica si la respuesta contiene "ERROR"
    if (respuesta.indexOf("ERROR") != -1) {
      Serial.println("¡Error detectado en el comando AT!");
    }
  } else {
    Serial.println("No se recibió respuesta.");
  }
  return respuesta;
}

// Función para encender el módulo SIM usando PWRKEY
void encenderModuloSIM() {
  Serial.println("Encendiendo módulo SIM A7670...");
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(1000); // Mantén PWRKEY en LOW por 1 segundo
  digitalWrite(PWRKEY, HIGH);
  delay(3000); // Espera a que el módulo inicie
  Serial.println("Módulo encendido.");
}

// Configuración inicial
void setup() {
  Serial.begin(115200);  // Comunicación con el monitor serie
  SIM7670.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // UART1 a 115200 baudios

  encenderModuloSIM(); // Encender el módulo SIM

  Serial.println("Inicializando módulo SIM A7670...");
  delay(3000);

  // Comandos AT básicos
  enviarComandoAT("AT");                               // Verifica comunicación
  enviarComandoAT("AT+CSQ");                           // Verifica el nivel de señal
  enviarComandoAT("AT+CREG?");                         // Estado de registro en la red
  enviarComandoAT("AT+CGDCONT=1,\"IP\",\"internet.wm.com\"", 5000); // Configura el APN
  enviarComandoAT("AT+CGATT=1", 5000);                 // Adjuntar a la red de datos
  enviarComandoAT("AT+CGPADDR=1", 2000);               // Verificar dirección IP asignada

}

// Bucle principal
void loop() {
  enviarComandoAT("AT");  // Enviar comando AT periódicamente para verificar actividad
  delay(5000);            // Espera 5 segundos entre comandos
}
