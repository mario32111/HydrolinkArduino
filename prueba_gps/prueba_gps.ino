#define GPS_RX_PIN 26  // GPS TX -> ESP32 RX (GPIO26)
#define GPS_TX_PIN 27  // GPS RX -> ESP32 TX (GPIO27)
#define HUMIDITY_SENSOR_PIN 33  // GPIO33 para sensor de humedad

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

void setup() {
  // Inicializa la comunicación serial para el monitor
  Serial.begin(115200);
  
  // Configura el puerto serial para el GPS
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Configura el pin del sensor de humedad
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);
  
  Serial.println("Sistema iniciado - Esperando datos...");
}

void loop() {
  // Procesamiento del GPS
  readGPSData();
  
  // Lectura de humedad
  humedadDelSuelo = analogRead(HUMIDITY_SENSOR_PIN);
  
  // Mostrar datos en el monitor serial
  displaySensorData(humedadDelSuelo);

  delay(1000);  // Espera 1 segundo entre lecturas
}

void readGPSData() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);  // Muestra los datos crudos en el Monitor Serial
    gpsData += c;
    
    if (c == '\n') {
      processGPSData(gpsData);
      gpsData = "";
    }
  }
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

  // Muestra datos de humedad
  Serial.print("Humedad del suelo: ");
  Serial.print(humidity);
  Serial.print(" (");
  int humidityPercentage = map(humidity, 0, 4095, 100, 0);  // Convertir a porcentaje
  Serial.print(humidityPercentage);
  Serial.println("%)");
  
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
