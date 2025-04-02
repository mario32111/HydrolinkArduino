#include <Arduino.h>
#include <HardwareSerial.h>

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

  // Configuración inicial del módulo SIM
  enviarComandoAT("AT");                               // Verifica comunicación
  enviarComandoAT("AT+CSQ");                           // Verifica el nivel de señal
  enviarComandoAT("AT+CREG?");                         // Estado de registro en la red
  enviarComandoAT("AT+CGDCONT=1,\"IP\",\"internet.wm.com\"", 5000); // Configura el APN
  enviarComandoAT("AT+CGATT=1", 5000);                 // Adjuntar a la red de datos
  enviarComandoAT("AT+CGPADDR=1", 2000);               // Verificar dirección IP asignada
}

// Función para manejar redirecciones
String manejarRedireccion(String url) {
  while (true) {
    // Inicializa el servicio HTTP
    if (enviarComandoAT("AT+HTTPINIT", 2000).indexOf("OK") != -1) {
      // Configura la URL
      String comandoURL = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
      if (enviarComandoAT(comandoURL, 2000).indexOf("OK") != -1) {
        // Realiza la solicitud GET
        String respuesta = enviarComandoAT("AT+HTTPACTION=0", 10000); // Espera hasta 10 segundos
        if (respuesta.indexOf("+HTTPACTION: 0,200") != -1) {
          Serial.println("Petición GET exitosa.");
          // Lee la respuesta del servidor (incluyendo cabeceras y cuerpo)
          String contenido = enviarComandoAT("AT+HTTPREAD", 5000);
          Serial.println("Contenido de la respuesta completa:");
          Serial.println(contenido);
          
          // Verifica si la redirección está presente en los encabezados HTTP
          int locIndex = contenido.indexOf("Location: ");
          if (locIndex != -1) {
            int locEnd = contenido.indexOf("\r\n", locIndex);
            String nuevaURL = contenido.substring(locIndex + 10, locEnd);
            Serial.println("Nueva URL de redirección:");
            Serial.println(nuevaURL);
            url = nuevaURL; // Actualiza la URL para seguir la redirección
          } else {
            Serial.println("No se encontró la URL de redirección.");
            break;
          }
        } else if (respuesta.indexOf("+HTTPACTION: 0,3") != -1) {
          Serial.println("Redirección detectada, pero no se pudo leer la URL.");
          // Intentar leer la respuesta completa para ver los encabezados
          String encabezados = enviarComandoAT("AT+HTTPREAD", 5000);
          Serial.println("Encabezados HTTP:");
          Serial.println(encabezados);
          break;
        } else {
          Serial.println("Error en la petición GET.");
          break;
        }
      } else {
        Serial.println("Error al configurar la URL.");
        break;
      }

      // Termina el servicio HTTP
      enviarComandoAT("AT+HTTPTERM", 2000);
    } else {
      Serial.println("Error al inicializar el servicio HTTP.");
      break;
    }
  }
  return "";
}

// Bucle principal
void loop() {
  String url = "http://example.com"; // URL inicial
  manejarRedireccion(url);
  delay(30000); // Espera 30 segundos antes de la siguiente petición
}
