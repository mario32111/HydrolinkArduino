#include <Arduino.h>
#include <HardwareSerial.h>

// Configuración del UART1 para el módulo SIM A7670
HardwareSerial SIM7670(1); // UART1

// Definición de pines
#define RX_PIN 27       // RX del ESP32 (GPIO27)
#define TX_PIN 26       // TX del ESP32 (GPIO26)
#define PWRKEY 4        // Pin de encendido PWRKEY conectado al GPIO4 del ESP32
const String brokerHost = "189.155.74.226";
const int brokerPort = 1883;
const String clientID = "ESP32_SIM_A7670";
const String topic = "test";

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
}

// Bucle principal
void loop() {
  // Aquí puedes agregar lógica adicional si es necesario
    recibirMensajes();
    
}
