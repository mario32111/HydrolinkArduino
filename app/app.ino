#include <Arduino.h>
#include <HardwareSerial.h>
#include <DHT.h>

// Configuración del UART
HardwareSerial SIM7670(1);  // UART1 para SIM A7670
HardwareSerial GPS(2);      // UART2 para GPS (evita conflicto con UART1)

// Definición de pines
#define RX_PIN 27       // RX del ESP32 (GPIO27)
#define TX_PIN 26       // TX del ESP32 (GPIO26)    
#define PWRKEY 4        // Pin de encendido PWRKEY conectado al GPIO4 del ESP32
const String brokerHost = "189.155.29.154";
const int brokerPort = 1883;
const String clientID = "ESP32_SIM_A7670";
const String topic = "llave/datos";


#define GPS_RX_PIN 13      // GPS TX -> ESP32 RX (GPIO26)
#define GPS_TX_PIN 12      // GPS RX -> ESP32 TX (GPIO27)
#define HUMIDITY_SENSOR_PIN 33  // GPIO33 para sensor de humedad del suelo
#define DHTPIN 32           // GPIO4 para el sensor DHT11
#define DHTTYPE DHT11      // Tipo de sensor DHT11

// Variables para el GPS
String gpsData = "";
float latitude = 0.0;
float longitude = 0.0;
bool gpsDataValid = false;

// Variables para la humedad del suelo
int humedadDelSuelo = 0;

// Variables para los datos del GPS
String gpsTime = "";
String gpsDate = "";
float gpsHDOP = 0.0;
int gpsQuality = 0;
int gpsSatellites = 0;
float gpsAltitude = 0.0;

// Variables para el DHT11
float temperaturaAmbiente = 0.0;
float humedadAmbiente = 0.0;

// Inicializar sensor DHT
DHT dht(DHTPIN, DHTTYPE);

//FUNCIONES DE LOS SENSORES
void leerDHT11() {
    // Leer humedad ambiente
    humedadAmbiente = dht.readHumidity();
    // Leer temperatura ambiente
    temperaturaAmbiente = dht.readTemperature();
    
    // Verificar si las lecturas fallaron
    if (isnan(humedadAmbiente) || isnan(temperaturaAmbiente)) {
      Serial.println("Error al leer el sensor DHT11!");
      return;
    }
    else{
      //mqttPub(humedadAmbiente)
      //mqttPub(temperaturaAmbiente)
    }
  }
  
void readGPSData() {
    while (GPS.available()) {
        char c = GPS.read();
        Serial.write(c);  // Muestra los datos crudos en el Monitor Serial
        gpsData += c;
        
        if (c == '\n') {
        processGPSData(gpsData);
        gpsData = "";
        }
    }
    mqttPub(gpsData);
}
  
void processGPSData(String data) {
    if (data.startsWith("$GPGGA")) {
        Serial.println("\nProcesando GPGGA: " + data);  // Depuración
        
        // Dividir el mensaje en partes
        int parts[15] = {0};
        int partIndex = 0;
        
        for (int i = 0; i < data.length() && partIndex < 14; i++) {
        if (data.charAt(i) == ',') {
            parts[++partIndex] = i + 1;
        } else if (data.charAt(i) == '*') {
            parts[++partIndex] = i + 1;
            break;
        }
        }

        // Extraer tiempo (hhmmss.ss)
        if (parts[1] > 0 && parts[2] > 0) {
        gpsTime = data.substring(parts[1], parts[2] - 1);
        }

        // Extraer latitud (llll.ll) y dirección (N/S)
        if (parts[2] > 0 && parts[4] > 0) {
        String latStr = data.substring(parts[2], parts[3] - 1);
        char latDir = data.charAt(parts[3] - 1);
        latitude = parseCoordinate(latStr, latDir);
        }

        // Extraer longitud (yyyyy.yy) y dirección (E/W)
        if (parts[4] > 0 && parts[6] > 0) {
        String lonStr = data.substring(parts[4], parts[5] - 1);
        char lonDir = data.charAt(parts[5] - 1);
        longitude = parseCoordinate(lonStr, lonDir);
        }

        // Calidad GPS (0-2)
        if (parts[6] > 0 && parts[7] > 0) {
        gpsQuality = data.substring(parts[6], parts[7] - 1).toInt();
        }

        // Número de satélites
        if (parts[7] > 0 && parts[8] > 0) {
        gpsSatellites = data.substring(parts[7], parts[8] - 1).toInt();
        }

        // HDOP
        if (parts[8] > 0 && parts[9] > 0) {
        gpsHDOP = data.substring(parts[8], parts[9] - 1).toFloat();
        }

        // Altitud
        if (parts[9] > 0 && parts[10] > 0) {
        gpsAltitude = data.substring(parts[9], parts[10] - 1).toFloat();
        }
        
        gpsDataValid = (gpsQuality > 0);
        
        // Procesar también GPRMC para obtener fecha
        if (data.startsWith("$GPRMC") && parts[9] > 0) {
        gpsDate = data.substring(parts[9], parts[9] + 6);  // Fecha en formato DDMMYY
        }
    }
}
  
float parseCoordinate(String coord, char direction) {
    if (coord.length() == 0 || coord.indexOf('.') == -1) {
        return 0.0;
    }

    int pointPos = coord.indexOf('.');
    if (pointPos < 2) return 0.0;

    // Grados son los primeros dígitos (2 para latitud, 3 para longitud)
    float degrees = coord.substring(0, pointPos - 2).toFloat();
    // Minutos son el resto
    float minutes = coord.substring(pointPos - 2).toFloat();

    float result = degrees + (minutes / 60.0);

    // Ajustar dirección
    if (direction == 'S' || direction == 'W') {
        result = -result;
    }

    return result;
}
  
void displaySensorData(int humidity) {
    Serial.println("\n=== DATOS DEL SISTEMA ===");

    // Muestra datos de humedad del suelo
    Serial.print("Humedad del suelo: ");
    Serial.print(humidity);
    Serial.print(" (");
    int humidityPercentage = map(humidity, 0, 4095, 100, 0);  // Convertir a porcentaje
    Serial.print(humidityPercentage);
    Serial.println("%)");

    // Muestra datos del DHT11
    Serial.println("--- Datos Ambientales ---");
    Serial.print("Temperatura ambiente: ");
    Serial.print(temperaturaAmbiente);
    Serial.print("°C | Humedad ambiente: ");
    Serial.print(humedadAmbiente);
    Serial.println("%");

    // Muestra datos del GPS
    Serial.println("--- Datos GPS ---");
    if (gpsDataValid) {
    Serial.print("Hora UTC: ");
    if (gpsTime.length() >= 6) {
    Serial.print(gpsTime.substring(0, 2)); // Horas
    Serial.print(":");
    Serial.print(gpsTime.substring(2, 4)); // Minutos
    Serial.print(":");
    Serial.println(gpsTime.substring(4));   // Segundos
    } else {
    Serial.println(gpsTime);
    }

    Serial.print("Fecha: ");
    if (gpsDate.length() == 6) {
    Serial.print(gpsDate.substring(0, 2)); // Día
    Serial.print("/");
    Serial.print(gpsDate.substring(2, 4)); // Mes
    Serial.print("/20");
    Serial.println(gpsDate.substring(4));   // Año
    } else {
    Serial.println(gpsDate);
    }

    Serial.print("Latitud: ");
    Serial.print(latitude, 6);
    Serial.print("  Longitud: ");
    Serial.println(longitude, 6);

    Serial.print("Calidad GPS: ");
    Serial.println(gpsQuality);

    Serial.print("Satélites: ");
    Serial.println(gpsSatellites);

    Serial.print("Precisión (HDOP): ");
    Serial.println(gpsHDOP, 1);

    Serial.print("Altitud: ");
    Serial.print(gpsAltitude, 1);
    Serial.println(" m");
    } else {
    Serial.println("Buscando señal GPS...");
    Serial.println("(Coloca el GPS al aire libre)");
    }
    Serial.println("=================");
}






//FUNCIONES PARA LA SIM
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
  
  String enviarComandoAT(String comando, int tiempoEspera = 2000) {
    Serial.print("Enviando comando: ");
    Serial.println(comando);
  
    SIM7670.println(comando);
    delay(100);
  
    long tiempoInicio = millis();
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
void mqttSub() {
    String respuesta;

    // 1. Iniciar el servicio MQTT
    respuesta = enviarComandoAT("AT+CMQTTSTART", 5000);
    if (respuesta.indexOf("OK") != -1) {
        Serial.println("Servicio MQTT iniciado.");
    } else {
        Serial.println("Error al iniciar el servicio MQTT.");
        return;
    }

    // 2. Adquirir un cliente MQTT
    respuesta = enviarComandoAT("AT+CMQTTACCQ=0,\"" + clientID + "\"", 5000);
    if (respuesta.indexOf("OK") != -1) {
        Serial.println("Cliente MQTT creado.");
    } else {
        Serial.println("Error al crear el cliente MQTT.");
        return;
    }

    // 3. Configurar el tema "will" (opcional)
    respuesta = enviarComandoAT("AT+CMQTTWILLTOPIC=0,10", 5000);
    if (respuesta.indexOf(">") != -1) {
        SIM7670.println("will_topic"); // Envía el tema "will"
        respuesta = enviarComandoAT("", 2000); // Espera la respuesta
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

    // 4. Configurar el mensaje "will" (opcional)
    respuesta = enviarComandoAT("AT+CMQTTWILLMSG=0,6,1", 5000);
    if (respuesta.indexOf(">") != -1) {
        SIM7670.println("will_msg"); // Envía el mensaje "will"
        respuesta = enviarComandoAT("", 2000); // Espera la respuesta
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

    // 5. Conectar al broker MQTT
    String comando = "AT+CMQTTCONNECT=0,\"tcp://" + brokerHost + ":" + String(brokerPort) + "\",60,1";
    respuesta = enviarComandoAT(comando, 5000); // Conectar al broker MQTT
    if (respuesta.indexOf("+CMQTTCONNECT: 0,0") != -1) {
        Serial.println("Conexión establecida con el broker MQTT.");
    } else {
        Serial.println("Error al conectar al broker MQTT.");
        return;
    }

    // 6. Suscribirse al tema
    respuesta = enviarComandoAT("AT+CMQTTSUBTOPIC=0," + String(topic.length()) + ",1", 5000);
    if (respuesta.indexOf(">") != -1) {
        SIM7670.println(topic); // Envía el tema
        respuesta = enviarComandoAT("", 2000); // Espera la respuesta
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

    // 7. Suscribirse al tema
    respuesta = enviarComandoAT("AT+CMQTTSUB=0", 5000);
    if (respuesta.indexOf("+CMQTTSUB: 0,0") != -1) {
        Serial.println("Suscripción exitosa al tema.");
    } else {
        Serial.println("Error al suscribirse al tema. Respuesta: " + respuesta);
        return;
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
  
void recibirMensajes() {
    while (SIM7670.available()) {
        String respuesta = SIM7670.readString();
        if (respuesta.indexOf("+CMQTTRXSTART") != -1) {
        Serial.println("Mensaje recibido:");
        Serial.println(respuesta);
        }
    }
}
  
void mqttPub(String mensaje) {
    String respuesta;

    // Configurar el tema para publicación
    respuesta = enviarComandoAT("AT+CMQTTTOPIC=0," + String(topic.length()), 5000);
    if (respuesta.indexOf(">") != -1) {
        SIM7670.println(topic); // Envía el tema
        respuesta = enviarComandoAT("", 2000); // Espera la respuesta
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

    // Configurar el payload (mensaje) para publicación
    respuesta = enviarComandoAT("AT+CMQTTPAYLOAD=0," + String(mensaje.length()), 5000);
    if (respuesta.indexOf(">") != -1) {
        SIM7670.println(mensaje); // Envía el mensaje
        respuesta = enviarComandoAT("", 2000); // Espera la respuesta
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

    // Publicar el mensaje
    respuesta = enviarComandoAT("AT+CMQTTPUB=0,1,60", 5000);
    if (respuesta.indexOf("+CMQTTPUB: 0,0") != -1) {
        Serial.println("Mensaje publicado correctamente.");
    } else {
        Serial.println("Error al publicar el mensaje. Respuesta: " + respuesta);
        return;
    }
}


// Configuración inicial
void setup() {
    //setup sim
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
    
    // Suscribirse al tema MQTT
    mqttSub();
    mqttPub("mensaje enviado desde la esp32");





    //setup gps

  
  // Configura el puerto serial para el GPS
  GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Configura el pin del sensor de humedad del suelo
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);
  
  // Inicializa el sensor DHT11
  dht.begin();
  
  Serial.println("Sistema iniciado - Esperando datos...");
  }





  void loop() {
    // Aquí puedes agregar lógica adicional si es necesario
      recibirMensajes();
        // Procesamiento del GPS




        
    readGPSData();
    
    // Lectura de humedad del suelo
    humedadDelSuelo = analogRead(HUMIDITY_SENSOR_PIN);    
    // Lectura del sensor DHT11
    leerDHT11();
    
    // Mostrar datos en el monitor serial
    displaySensorData(humedadDelSuelo);

    delay(2000);  // Espera 2 segundos entre lecturas (necesario para DHT11)
  }
  
