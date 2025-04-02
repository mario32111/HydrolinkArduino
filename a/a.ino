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

// Función para verificar el estado de la red y asegurarse de que la SIM esté registrada
void verificarConexion() {
  // Verificar el estado de registro en la red
  String estadoRed = enviarComandoAT("AT+CREG?", 5000);
  Serial.println("Estado de registro en la red: " + estadoRed);
  if (estadoRed.indexOf("+CREG: 0,1") != -1 || estadoRed.indexOf("+CREG: 0,5") != -1) {
    Serial.println("Módulo registrado en la red.");
  } else {
    Serial.println("Intentando registro manual...");
    enviarComandoAT("AT+COPS=0", 10000); // Registro automático en la red
    estadoRed = enviarComandoAT("AT+CREG?", 5000);
    if (estadoRed.indexOf("+CREG: 0,1") != -1 || estadoRed.indexOf("+CREG: 0,5") != -1) {
      Serial.println("Registro manual exitoso.");
    } else {
      Serial.println("No se pudo registrar en la red.");
    }
  }

  // Verificar nivel de señal
  String nivelSenal = enviarComandoAT("AT+CSQ", 2000);
  Serial.println("Nivel de señal: " + nivelSenal);

  // Intentar adjuntarse a la red
  String adjuntarRed = enviarComandoAT("AT+CGATT=1", 5000);
  if (adjuntarRed.indexOf("OK") != -1) {
    Serial.println("Adjuntado a la red con éxito.");
  } else {
    Serial.println("Error al adjuntarse a la red.");
  }
}

// Función para manejar la asignación de la dirección IP
void verificarIP() {
  // Verificar si se asignó una dirección IP
  String direccionIP = enviarComandoAT("AT+CGPADDR=1", 5000);
  Serial.println("Dirección IP: " + direccionIP);
  if (direccionIP.indexOf("0.0.0.0") != -1) {
    Serial.println("No se asignó una dirección IP.");
  } else {
    Serial.println("Dirección IP asignada: " + direccionIP);
  }
}

// Función para realizar la solicitud HTTP y verificar errores
void realizarSolicitudHTTP(String url) {
  // Inicializa el servicio HTTP
  String respuesta = enviarComandoAT("AT+HTTPINIT", 2000);  // Inicia el servicio HTTP
  if (respuesta.indexOf("OK") != -1) {
    // Configura la URL
    String comandoURL = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
    respuesta = enviarComandoAT(comandoURL, 2000);
    if (respuesta.indexOf("OK") != -1) {
      // Realiza la solicitud GET
      respuesta = enviarComandoAT("AT+HTTPACTION=0", 10000); // Espera hasta 10 segundos
      if (respuesta.indexOf("+HTTPACTION: 0,200") != -1) {
        Serial.println("Petición GET exitosa.");
        
        // Leer la cabecera de la respuesta
        String cabecera = enviarComandoAT("AT+HTTPHEAD", 5000);
        Serial.println("Cabecera de la respuesta:");
        Serial.println(cabecera);

        // Leer la respuesta del servidor en bloques
        int bytesLeidos = 0;
        int totalBytes = 52060;  // Según la longitud de la respuesta en el comando HTTPACTION (ajustar si es necesario)
        while (bytesLeidos < totalBytes) {
          String contenido = enviarComandoAT("AT+HTTPREAD=" + String(bytesLeidos) + ",500", 5000); // Leer en bloques de 500 bytes
          Serial.println("Contenido parcial de la respuesta:");
          Serial.println(contenido);
          
          bytesLeidos += contenido.length();
          delay(100); // Breve espera para no sobrecargar el módulo
        }

        // Terminar el servicio HTTP
        enviarComandoAT("AT+HTTPTERM", 2000); // Detener el servicio HTTP
      } else {
        Serial.println("Error en la petición GET. Código de respuesta: " + respuesta);
      }
    } else {
      Serial.println("Error al configurar la URL. Respuesta: " + respuesta);
    }
  } else {
    Serial.println("Error al inicializar el servicio HTTP.");
  }
}


// Configuración inicial
void setup() {
  Serial.begin(115200);
  SIM7670.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  encenderModuloSIM();

  Serial.println("Inicializando módulo SIM A7670...");
  delay(3000);

  // Configuración inicial del módulo SIM
  enviarComandoAT("AT");                               // Verifica comunicación
  enviarComandoAT("AT+CSQ");                           // Verifica el nivel de señal
  enviarComandoAT("AT+CREG?");                         // Estado de registro en la red
  enviarComandoAT("AT+CGDCONT=1,\"IP\",\"eapn1.net\"", 5000); // Configura el APN
  verificarConexion();  // Verificar la conexión y registro

  // Verificar dirección IP asignada
  verificarIP();
}

// Bucle principal
void loop() {
  String direccionIP = enviarComandoAT("AT+CGPADDR=1", 5000);
  if (direccionIP.indexOf("0.0.0.0") == -1) {
    Serial.println("Conexión activa. Realizando solicitud HTTP.");
    String url = "http://189.155.87.106:3000/simulate-sensor"; // URL inicial
    realizarSolicitudHTTP(url);
  } else {
    Serial.println("No hay conexión. Reintentando registro...");
    verificarConexion();  // Reintenta el registro
  }
  delay(30000); // Espera 30 segundos antes de la siguiente iteración
}
