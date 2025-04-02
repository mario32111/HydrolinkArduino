#include <DHT.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// *********************** CONFIGURACIÓN SIM A7670 ***********************
HardwareSerial SIM7670(1); // UART1

#define RX_PIN 27       // RX del ESP32 (GPIO27)
#define TX_PIN 26       // TX del ESP32 (GPIO26)
#define PWRKEY 4        // Pin PWRKEY conectado al GPIO4

const String brokerHost = "189.155.29.154";
const int brokerPort = 1883;
const String clientID = "ESP32_SIM_A7670";
const String topicPub = "llave/datos";
const String topicSub = "llave/config";

bool simInitialized = false;
bool mqttConnected = false;

// *********************** CONFIGURACIÓN SENSORES ***********************
#define HUMIDITY_SENSOR_PIN 33  // GPIO33 para sensor de humedad del suelo
#define DHTPIN 32               // GPIO32 para el sensor DHT11
#define DHTTYPE DHT11           // Tipo de sensor DHT11

int humedadDelSuelo = 0;
bool soilSensorInitialized = false;

float temperaturaAmbiente = 0.0;
float humedadAmbiente = 0.0;
bool dhtInitialized = false;

DHT dht(DHTPIN, DHTTYPE);

// *********************** FUNCIONES SIM A7670 ***********************
void encenderModuloSIM() {
  Serial.println("Encendiendo módulo SIM A7670...");
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(1000); // Mínimo requerido por el SIM A7670
  digitalWrite(PWRKEY, HIGH);
  delay(2000); // Reducido de 3000 ms, suficiente para iniciar
  Serial.println("Módulo encendido.");
}

String enviarComandoAT(String comando, int tiempoEspera = 500) {
  Serial.print("Enviando comando: ");
  Serial.println(comando);

  SIM7670.println(comando);
  delay(20); // Reducido de 100 ms, suficiente para que el comando se envíe

  unsigned long tiempoInicio = millis();
  String respuesta = "";

  while (millis() - tiempoInicio < tiempoEspera) {
    while (SIM7670.available()) {
      char c = SIM7670.read();
      respuesta += c;
    }
  }

  if (respuesta.length() > 0) {
    Serial.println("Respuesta:");
    Serial.println(respuesta);
  } else {
    Serial.println("No se recibió respuesta.");
  }
  return respuesta;
}

void verificarConexion() {
  String estadoRed = enviarComandoAT("AT+CREG?", 1000); // Reducido de 5000 ms
  Serial.println("Estado de registro en la red: " + estadoRed);
  if (estadoRed.indexOf("+CREG: 0,1") != -1 || estadoRed.indexOf("+CREG: 0,5") != -1) {
    Serial.println("Módulo registrado en la red.");
  } else {
    Serial.println("Intentando registro manual...");
    enviarComandoAT("AT+COPS=0", 5000); // Mantengo 5000 ms aquí por si la red es lenta
    estadoRed = enviarComandoAT("AT+CREG?", 1000);
    if (estadoRed.indexOf("+CREG: 0,1") != -1 || estadoRed.indexOf("+CREG: 0,5") != -1) {
      Serial.println("Registro manual exitoso.");
    } else {
      Serial.println("No se pudo registrar en la red.");
    }
  }

  String nivelSenal = enviarComandoAT("AT+CSQ", 500); // Reducido de 2000 ms
  Serial.println("Nivel de señal: " + nivelSenal);

  String adjuntarRed = enviarComandoAT("AT+CGATT=1", 1000); // Reducido de 5000 ms
  if (adjuntarRed.indexOf("OK") != -1) {
    Serial.println("Adjuntado a la red con éxito.");
  } else {
    Serial.println("Error al adjuntarse a la red.");
  }
}

void verificarIP() {
  String direccionIP = enviarComandoAT("AT+CGPADDR=1", 1000); // Reducido de 5000 ms
  Serial.println("Dirección IP: " + direccionIP);
  if (direccionIP.indexOf("0.0.0.0") != -1) {
    Serial.println("No se asignó una dirección IP.");
  } else {
    Serial.println("Dirección IP asignada: " + direccionIP);
  }
}

void mqttSub() {
  String respuesta;

  respuesta = enviarComandoAT("AT+CMQTTSTART", 1000); // Reducido de 5000 ms
  if (respuesta.indexOf("OK") != -1) {
    Serial.println("Servicio MQTT iniciado.");
  } else {
    Serial.println("Error al iniciar el servicio MQTT.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTACCQ=0,\"" + clientID + "\"", 1000); // Reducido de 5000 ms
  if (respuesta.indexOf("OK") != -1) {
    Serial.println("Cliente MQTT creado.");
  } else {
    Serial.println("Error al crear el cliente MQTT.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTWILLTOPIC=0,10", 500); // Reducido de 5000 ms
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println("will_topic");
    respuesta = enviarComandoAT("", 500); // Reducido de 2000 ms
    if (respuesta.indexOf("OK") != -1) {
      Serial.println("Tema 'will' configurado.");
    } else {
      Serial.println("Error al configurar el tema 'will'.");
      return;
    }
  } else {
    Serial.println("Error al iniciar la configuración del tema 'will'.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTWILLMSG=0,6,1", 500); // Reducido de 5000 ms
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println("will_msg");
    respuesta = enviarComandoAT("", 500); // Reducido de 2000 ms
    if (respuesta.indexOf("OK") != -1) {
      Serial.println("Mensaje 'will' configurado.");
    } else {
      Serial.println("Error al configurar el mensaje 'will'.");
      return;
    }
  } else {
    Serial.println("Error al iniciar la configuración del mensaje 'will'.");
    return;
  }

  String comando = "AT+CMQTTCONNECT=0,\"tcp://" + brokerHost + ":" + String(brokerPort) + "\",60,1";
  respuesta = enviarComandoAT(comando, 2000); // Reducido de 5000 ms, puede requerir más tiempo por la red
  if (respuesta.indexOf("+CMQTTCONNECT: 0,0") != -1) {
    Serial.println("Conexión establecida con el broker MQTT.");
    mqttConnected = true;
  } else {
    Serial.println("Error al conectar al broker MQTT.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTSUBTOPIC=0," + String(topicSub.length()) + ",1", 500); // Reducido de 5000 ms
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println(topicSub);
    respuesta = enviarComandoAT("", 500); // Reducido de 2000 ms
    if (respuesta.indexOf("OK") != -1) {
      Serial.println("Tema configurado para suscripción.");
    } else {
      Serial.println("Error al configurar el tema para suscripción.");
      return;
    }
  } else {
    Serial.println("Error al iniciar la configuración del tema para suscripción.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTSUB=0", 1000); // Reducido de 5000 ms
  if (respuesta.indexOf("+CMQTTSUB: 0,0") != -1) {
    Serial.println("Suscripción exitosa al tema de configuración.");
  } else {
    Serial.println("Error al suscribirse al tema de configuración. Respuesta: " + respuesta);
    return;
  }
}

void mqttPub(String mensaje) {
  if (!mqttConnected) {
    Serial.println("MQTT no conectado, no se puede publicar.");
    return;
  }

  String respuesta;

  respuesta = enviarComandoAT("AT+CMQTTTOPIC=0," + String(topicPub.length()), 500); // Reducido de 5000 ms
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println(topicPub);
    respuesta = enviarComandoAT("", 500); // Reducido de 2000 ms
    if (respuesta.indexOf("OK") != -1) {
      Serial.println("Tema configurado para publicación.");
    } else {
      Serial.println("Error al configurar el tema para publicación.");
      return;
    }
  } else {
    Serial.println("Error al iniciar la configuración del tema para publicación.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTPAYLOAD=0," + String(mensaje.length()), 500); // Reducido de 5000 ms
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println(mensaje);
    respuesta = enviarComandoAT("", 500); // Reducido de 2000 ms
    if (respuesta.indexOf("OK") != -1) {
      Serial.println("Payload configurado para publicación.");
    } else {
      Serial.println("Error al configurar el payload para publicación.");
      return;
    }
  } else {
    Serial.println("Error al iniciar la configuración del payload para publicación.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTPUB=0,1,60", 1000); // Reducido de 5000 ms
  if (respuesta.indexOf("+CMQTTPUB: 0,0") != -1) {
    Serial.println("Mensaje publicado correctamente en " + topicPub);
  } else {
    Serial.println("Error al publicar el mensaje. Respuesta: " + respuesta);
    return;
  }
}

void recibirMensajes() {
  static String bufferParcial = "";
  const unsigned int maxTamanoBuffer = 512;

  unsigned long inicio = millis();
  while (SIM7670.available() && (millis() - inicio < 20)) { // Reducido de 50 ms
    if (bufferParcial.length() < maxTamanoBuffer) {
      char c = SIM7670.read();
      bufferParcial += c;

      if (c == '\n') {
        bufferParcial.trim();
        if (bufferParcial.length() > 0) {
          procesarMensajeMQTT(bufferParcial);
        }
        bufferParcial = "";
      }
    } else {
      Serial.println("¡Buffer MQTT lleno! Limpiando...");
      bufferParcial = "";
    }
  }
}

void procesarMensajeMQTT(const String& mensaje) {
  if (mensaje.indexOf("+CMQTTRXSTART") != -1) {
    Serial.print("[MQTT RX] ");
    Serial.println(mensaje);

    if (mensaje.indexOf("reset=all") != -1) {
      Serial.println("Reiniciando todos los sensores...");
      // Lógica de reinicio aquí si es necesario
    }
  }
}

// *********************** FUNCIONES SENSORES ***********************
void leerDHT11() {
  if (!dhtInitialized) return;

  humedadAmbiente = dht.readHumidity();
  temperaturaAmbiente = dht.readTemperature();

  if (isnan(humedadAmbiente)) {
    Serial.println("Error al leer la humedad del sensor DHT11!");
    humedadAmbiente = 0.0;
  }
  if (isnan(temperaturaAmbiente)) {
    Serial.println("Error al leer la temperatura del sensor DHT11!");
    temperaturaAmbiente = 0.0;
  }
}

String createSensorJSON() {
  int humidityPercentage = map(humedadDelSuelo, 0, 4095, 100, 0);

  String json = "{";
  json += "\"environment\":{\"temperature\":" + String(temperaturaAmbiente, 1) + ",\"humidity\":" + String(humedadAmbiente, 1) + "},";
  json += "\"soil\":{\"raw_value\":" + String(humedadDelSuelo) + ",\"percentage\":" + String(humidityPercentage) + "}";
  json += "}";
  return json;
}

void displaySensorData(int humidity) {
  Serial.println("\n=== DATOS DEL SISTEMA ===");
  Serial.print("Humedad del suelo: ");
  Serial.print(humidity);
  Serial.print(" (");
  Serial.print(map(humidity, 0, 4095, 100, 0));
  Serial.println("%)");
  Serial.println("--- Datos Ambientales ---");
  Serial.print("Temperatura ambiente: ");
  Serial.print(temperaturaAmbiente);
  Serial.print("°C | Humedad ambiente: ");
  Serial.print(humedadAmbiente);
  Serial.println("%");
  Serial.println("=================");
}

// *********************** SETUP Y LOOP PRINCIPAL ***********************
void setup() {
  Serial.begin(115200);

  Serial.println("\nIniciando inicialización del módulo SIM...");
  SIM7670.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  encenderModuloSIM();

  enviarComandoAT("AT");
  enviarComandoAT("AT+CSQ");
  enviarComandoAT("AT+CREG?");
  enviarComandoAT("AT+CGDCONT=1,\"IP\",\"eapn1.net\"", 1000); // Reducido de 5000 ms
  verificarConexion();
  verificarIP();
  mqttSub();

  simInitialized = true;
  Serial.println("Módulo SIM inicializado correctamente.");

  Serial.println("\nIniciando inicialización de sensores...");
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);
  soilSensorInitialized = true;
  Serial.println("Sensor de humedad del suelo inicializado.");

  dht.begin();
  dhtInitialized = true;
  Serial.println("Sensor DHT11 inicializado.");

  Serial.println("\nTodos los sistemas inicializados.");
}

void loop() {
  static unsigned long ultimaPublicacion = 0;
  static unsigned long ultimaLectura = 0;
  static unsigned long ultimoCheckMensajes = 0;

  if (mqttConnected && millis() - ultimoCheckMensajes >= 50) { // Reducido de 100 ms
    ultimoCheckMensajes = millis();
    recibirMensajes();
  }

  if (millis() - ultimaLectura >= 1000) { // Reducido de 2000 ms
    ultimaLectura = millis();

    if (soilSensorInitialized) {
      humedadDelSuelo = analogRead(HUMIDITY_SENSOR_PIN);
    }

    if (dhtInitialized) {
      leerDHT11();
    }

    displaySensorData(humedadDelSuelo);
  }

  if (millis() - ultimaPublicacion >= 1000 && // Reducido de 4000 ms
      simInitialized && mqttConnected && 
      soilSensorInitialized && dhtInitialized) {
    ultimaPublicacion = millis();

    String jsonData = createSensorJSON();
    mqttPub(jsonData);
  }
}
