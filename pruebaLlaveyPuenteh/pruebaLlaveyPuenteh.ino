
// Definición de pines del puente H
const int enAPin = 15;
const int in1Pin = 14;
const int in2Pin = 25;

// Definición de la duración del pulso para la llave latching (en milisegundos)
const int pulseDuration = 200; // Ajusta este valor si es necesario

// Estados para controlar la llave
enum LlaveState {
  CERRADA,
  ABIERTA
};

LlaveState estadoLlave = CERRADA; // Estado inicial de la llave

void setup() {
  // Configuración de los pines del puente H como salidas
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  // Inicialmente deshabilitar el puente H
  digitalWrite(enAPin, LOW);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  Serial.begin(115200); // Prueba con una velocidad común como 115200
  Serial.println("Control de Llave DC Latching");
  Serial.println("Escribe 'abrir' o 'cerrar' en el monitor serial.");
}



void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim(); // Eliminar espacios en blanco al principio y al final
    if (comando.equalsIgnoreCase("abrir") && estadoLlave == CERRADA) {
      abrirLlave();
      estadoLlave = ABIERTA;
      Serial.println("Llave ABIERTA");
    } else if (comando.equalsIgnoreCase("cerrar") && estadoLlave == ABIERTA) {
      cerrarLlave();
      estadoLlave = CERRADA;
      Serial.println("Llave CERRADA");
    } else if (comando.equalsIgnoreCase("abrir") && estadoLlave == ABIERTA) {
      Serial.println("La llave ya está ABIERTA.");
    } else if (comando.equalsIgnoreCase("cerrar") && estadoLlave == CERRADA) {
      Serial.println("La llave ya está CERRADA.");
    } else {
      Serial.println("Comando inválido. Escribe 'abrir' o 'cerrar'.");
    }
  }
  // Aquí puedes agregar otra lógica de tu programa si es necesario
}

void abrirLlave() {
  // Habilitar el puente H
  digitalWrite(enAPin, HIGH);
  // Establecer la dirección para abrir (ajusta según tu llave)
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  // Mantener el pulso durante un corto tiempo
  delay(pulseDuration);
  // Detener el flujo de corriente
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  // Deshabilitar el puente H (opcional para ahorrar energía)
  digitalWrite(enAPin, LOW);
}
void cerrarLlave() {
  // Habilitar el puente H
  digitalWrite(enAPin, HIGH);
  // Establecer la dirección para cerrar (la opuesta a la de abrir)
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  // Mantener el pulso durante un corto tiempo
  delay(pulseDuration);
  // Detener el flujo de corriente
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  // Deshabilitar el puente H (opcional para ahorrar energía)
  digitalWrite(enAPin, LOW);
}
