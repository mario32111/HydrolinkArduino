#include "HardwareSerial.h"

// Configuración del UART1 para el módulo SIM A7670
HardwareSerial SIM7670(1); // UART1

// Definición de pines
#define RX_PIN 27       // RX del ESP32 (GPIO27)
#define TX_PIN 26       // TX del ESP32 (GPIO26)
#define PWRKEY 4        // Pin de encendido PWRKEY conectado al GPIO4 del ESP32

// Función para enviar comandos AT
void enviarComandoAT(String comando, int tiempoEspera = 2000) {
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
  } else {
    Serial.println("No se recibió respuesta.");
  }
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
  //enviarComandoAT("AT");                               // Verifica comunicación
  //enviarComandoAT("AT+CSQ");                           // Nivel de señal
  //enviarComandoAT("AT+CREG?");                         // Estado del registro en la red
  //enviarComandoAT("AT+CGDCONT=1,\"IP\",\"internet.wm.com\"", 5000); // Configura APN
  //enviarComandoAT("AT+CGATT=1", 5000);                 // Adjuntar a la red de datos
  //enviarComandoAT("AT+SAPBR=1,1", 7000);               // Habilita conexión de datos

  //Escanear redes disponibles
  enviarComandoAT("AT+CNMI=0,0,0,0,0"); // Desactiva notificaciones
  enviarComandoAT("AT+COPS=?", 360000);  // Escaneo de redes

    // Comandos para registrar en la red y configurar datos
  //enviarComandoAT("AT");                              // Verifica comunicación
  //enviarComandoAT("AT+COPS=1,0,\"BAIT R\"", 10000);   // Registro en la red BAIT
  //enviarComandoAT("AT+CGDCONT=1,\"IP\",\"internet.wm.com\"", 5000); // APN configurado correctamente
  //enviarComandoAT("AT+CGATT=1", 10000);              // Adjuntar a la red de datos
  //enviarComandoAT("AT+SAPBR=1,1", 15000);             // Habilita conexión de datos
  //enviarComandoAT("AT+SAPBR=2,1", 5000);             // Verificar conexión
  //enviarComandoAT("AT+CSQ", 5000); // Verificar la intensidad de la señal
  //enviarComandoAT("AT+CREG?", 5000); // Estado del registro en la red (para redes GSM)


}

// Bucle principal
void loop() {
  enviarComandoAT("AT");  // Enviar comando AT periódicamente para ver si está activo
  delay(5000);            // Espera 5 segundos entre comandos
}
