#include <DHT.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// *********************** CONFIGURACIÓN SIM A7670 ***********************
HardwareSerial SIM7670(1); // UART1
#define RX_PIN 27         // RX del ESP32 (GPIO27)
#define TX_PIN 26         // TX del ESP32 (GPIO26)
#define PWRKEY 4          // Pin PWRKEY conectado al GPIO4
const String brokerHost = "107.21.152.83";
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

// *********************** CONFIGURACIÓN LLAVE LATCHING ***********************
const int enAPin = 15;
const int in1Pin = 14;
const int in2Pin = 25;
const int pulseDuration = 200;
enum LlaveState {
  CERRADA,
  ABIERTA
};
LlaveState estadoLlave = CERRADA;

// *********************** FUNCIONES SIM A7670 ***********************
void encenderModuloSIM() {
  Serial.println("Encendiendo módulo SIM A7670...");
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(1000);
  digitalWrite(PWRKEY, HIGH);
  delay(2000);
  Serial.println("Módulo encendido.");
}

String enviarComandoAT(String comando, int tiempoEspera = 500) {
  Serial.print("Enviando comando: ");
  Serial.println(comando);

  SIM7670.println(comando);
  delay(20);

  unsigned long tiempoInicio = millis();
  String respuesta = "";

  while (millis() - tiempoInicio < tiempoEspera) {
    while (SIM7670.available()) {
      char c = SIM7670.read();
      respuesta += c;
    }
  }

  if (respuesta.length() > 0) {
    Serial.print("Respuesta: ");
    Serial.println(respuesta);
  } else {
    Serial.println("No se recibió respuesta.");
  }
  return respuesta;
}

void verificarConexion() {
  String estadoRed = enviarComandoAT("AT+CREG?", 1000);
  Serial.println("Estado de registro en la red: " + estadoRed);
  if (estadoRed.indexOf("+CREG: 0,1") != -1 || estadoRed.indexOf("+CREG: 0,5") != -1) {
    Serial.println("Módulo registrado en la red.");
  } else {
    Serial.println("Intentando registro manual...");
    enviarComandoAT("AT+COPS=0", 5000);
    estadoRed = enviarComandoAT("AT+CREG?", 1000);
    if (estadoRed.indexOf("+CREG: 0,1") != -1 || estadoRed.indexOf("+CREG: 0,5") != -1) {
      Serial.println("Registro manual exitoso.");
    } else {
      Serial.println("No se pudo registrar en la red.");
    }
  }

  String nivelSenal = enviarComandoAT("AT+CSQ", 500);
  Serial.println("Nivel de señal: " + nivelSenal);

  String adjuntarRed = enviarComandoAT("AT+CGATT=1", 1000);
  if (adjuntarRed.indexOf("OK") != -1) {
    Serial.println("Adjuntado a la red con éxito.");
  } else {
    Serial.println("Error al adjuntarse a la red.");
  }
}

void verificarIP() {
  String direccionIP = enviarComandoAT("AT+CGPADDR=1", 1000);
  Serial.println("Dirección IP: " + direccionIP);
  if (direccionIP.indexOf("0.0.0.0") != -1) {
    Serial.println("No se asignó una dirección IP.");
  } else {
    Serial.println("Dirección IP asignada: " + direccionIP);
  }
}

void mqttSub() {
  String respuesta;

  respuesta = enviarComandoAT("AT+CMQTTSTART", 1000);
  if (respuesta.indexOf("OK") != -1) {
    Serial.println("Servicio MQTT iniciado.");
  } else {
    Serial.println("Error al iniciar el servicio MQTT.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTACCQ=0,\"" + clientID + "\"", 1000);
  if (respuesta.indexOf("OK") != -1) {
    Serial.println("Cliente MQTT creado.");
  } else {
    Serial.println("Error al crear el cliente MQTT.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTWILLTOPIC=0,10", 500);
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println("will_topic");
    respuesta = enviarComandoAT("", 500);
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

  respuesta = enviarComandoAT("AT+CMQTTWILLMSG=0,6,1", 500);
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println("will_msg");
    respuesta = enviarComandoAT("", 500);
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
  respuesta = enviarComandoAT(comando, 2000);
  if (respuesta.indexOf("+CMQTTCONNECT: 0,0") != -1) {
    Serial.println("Conexión establecida con el broker MQTT.");
    mqttConnected = true;
  } else {
    Serial.println("Error al conectar al broker MQTT.");
    return;
  }

  respuesta = enviarComandoAT("AT+CMQTTSUBTOPIC=0," + String(topicSub.length()) + ",1", 500);
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println(topicSub);
    respuesta = enviarComandoAT("", 500);
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

  respuesta = enviarComandoAT("AT+CMQTTSUB=0", 1000);
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

  respuesta = enviarComandoAT("AT+CMQTTTOPIC=0," + String(topicPub.length()), 500);
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println(topicPub);
    respuesta = enviarComandoAT("", 500);
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

  respuesta = enviarComandoAT("AT+CMQTTPAYLOAD=0," + String(mensaje.length()), 500);
  if (respuesta.indexOf(">") != -1) {
    SIM7670.println(mensaje);
    respuesta = enviarComandoAT("", 500);
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

  respuesta = enviarComandoAT("AT+CMQTTPUB=0,1,60", 1000);
  if (respuesta.indexOf("+CMQTTPUB: 0,0") != -1) {
    Serial.println("Mensaje publicado correctamente en " + topicPub);
  } else {
    Serial.println("Error al publicar el mensaje. Respuesta: " + respuesta);
    return;
  }
}

void recibirMensajes() {
  static String bufferCompleto = "";
  static bool capturandoMensaje = false;
  
  while (SIM7670.available()) {
    String linea = SIM7670.readStringUntil('\n');
    linea.trim();
    
    if (linea.length() > 0) {
      // Limpiar respuestas AT no deseadas
      if (linea.startsWith("AT+") || linea == "OK" || linea == ">") {
        continue;
      }

      Serial.print("[SIM RX] ");
      Serial.println(linea);

      if (linea.startsWith("+CMQTTRXSTART")) {
        capturandoMensaje = true;
        bufferCompleto = linea + "\n";
      } 
      else if (capturandoMensaje) {
        bufferCompleto += linea + "\n";
        
        if (linea.startsWith("+CMQTTRXEND")) {
          // Limpiar buffer antes de procesar
          while(SIM7670.available()) SIM7670.read();
          
          procesarMensajeMQTT(bufferCompleto);
          capturandoMensaje = false;
          bufferCompleto = "";
        }
      }
    }
  }
}

void procesarMensajeMQTT(const String& mensaje) {
  // Limpiar buffer serial antes de procesar
  while(SIM7670.available()) SIM7670.read();

  Serial.println("[MQTT] Mensaje completo recibido:");
  Serial.println(mensaje);

  int startPayload = mensaje.indexOf("+CMQTTRXPAYLOAD:");
  if (startPayload == -1) {
    Serial.println("Error: No se encontró payload");
    return;
  }

  int payloadStart = mensaje.indexOf('\n', startPayload) + 1;
  int payloadEnd = mensaje.indexOf('\n', payloadStart);
  String payload = mensaje.substring(payloadStart, payloadEnd);
  payload.trim();

  Serial.print("[MQTT] Payload extraído: '");
  Serial.print(payload);
  Serial.println("'");

  if (payload == "abrir" && estadoLlave != ABIERTA) {
    Serial.println("Ejecutando ABRIR...");
    abrirLlave();
    estadoLlave = ABIERTA;
    Serial.println("Estado: ABIERTA");
  } 
  else if (payload == "cerrar" && estadoLlave != CERRADA) {
    Serial.println("Ejecutando CERRAR...");
    cerrarLlave();
    estadoLlave = CERRADA;
    Serial.println("Estado: CERRADA");
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
  String valveState = (estadoLlave == ABIERTA ? "open" : "closed");

  String json = "{";
  json += "\"environment\":{\"temperature\":" + String(temperaturaAmbiente, 1) + ",\"humidity\":" + String(humedadAmbiente, 1) + "},";
  json += "\"soil\":{\"raw_value\":" + String(humedadDelSuelo) + ",\"percentage\":" + String(humidityPercentage) + "},";
  json += "\"valve\":\"" + valveState + "\"";
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
  Serial.println("--- Estado de la Válvula ---");
  Serial.print("Estado: ");
  Serial.println(estadoLlave == ABIERTA ? "ABIERTA" : "CERRADA");
  Serial.println("=================");
}

// *********************** FUNCIONES ELECTROVÁLVULA ***********************
void abrirLlave() {
  Serial.println("--- INICIO abrirLlave() ---");
  digitalWrite(enAPin, HIGH);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  delay(pulseDuration);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(enAPin, LOW);
  Serial.println("--- FIN abrirLlave() ---");
}

void cerrarLlave() {
  Serial.println("--- INICIO cerrarLlave() ---");
  digitalWrite(enAPin, HIGH);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  delay(pulseDuration);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(enAPin, LOW);
  Serial.println("--- FIN cerrarLlave() ---");
}

void testLlaveManual() {
  static unsigned long lastTest = 0;
  if (millis() - lastTest > 10000) { // Cada 10 segundos
    lastTest = millis();
    Serial.println("Probando llave manualmente...");
    abrirLlave();
    delay(2000);
    cerrarLlave();
  }
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
  enviarComandoAT("AT+CGDCONT=1,\"IP\",\"eapn1.net\"", 1000);
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

  Serial.println("\nIniciando inicialización de la llave...");
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(enAPin, LOW);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  
  // Prueba inicial de la llave
  Serial.println("Prueba inicial de la llave...");
  abrirLlave();
  delay(1000);
  cerrarLlave();
  Serial.println("Llave inicializada.");

  Serial.println("\nTodos los sistemas inicializados.");
}

void loop() {
  static unsigned long ultimaPublicacion = 0;
  
  // 1. Máxima prioridad: Recepción MQTT
  if (mqttConnected) {
    recibirMensajes();
  }

  // 2. Lectura de sensores (cada 2 segundos)
  static unsigned long ultimaLectura = 0;
  if (millis() - ultimaLectura >= 2000) {
    ultimaLectura = millis();
    
    if (soilSensorInitialized) {
      humedadDelSuelo = analogRead(HUMIDITY_SENSOR_PIN);
    }
    
    if (dhtInitialized) {
      leerDHT11();
    }
    
    displaySensorData(humedadDelSuelo);
  }

  // 3. Publicación MQTT (cada 10 segundos)
  if (millis() - ultimaPublicacion >= 10000 && mqttConnected) {
    ultimaPublicacion = millis();
    
    // Limpiar buffer antes de publicar
    while(SIM7670.available()) SIM7670.read();
    
    String jsonData = createSensorJSON();
    mqttPub(jsonData);
  }

  delay(20);
}
