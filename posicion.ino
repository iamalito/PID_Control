#include <LiquidCrystal.h>
#include <Keypad.h>
#include <SoftwareSerial.h>

// Configuración del LCD: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

// Configuración del teclado matricial 3x4
const byte FILAS = 4;
const byte COLUMNAS = 3;

// Configura SoftwareSerial en pines 10 (RX) y 11 (TX)
SoftwareSerial espSerial(15, 14);

// Definimos las teclas del teclado (en su disposición)
char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// Pines conectados a las filas y columnas del teclado
byte pinesFilas[FILAS] = {A0, A1, A2, A3};
byte pinesColumnas[COLUMNAS] = {A4, A5, A6};

// Crear el objeto Keypad
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

// Cadena para almacenar las teclas presionadas
String cadenaTeclas = "";

// Variables para el motor
char SalidaPWM = 10; // Pin de salida para PWM motor
char SentGiroPos = 9;  // Pin de sentido de giro
char SentGiroNeg = 8; // Pin de sentido de giro
char EncA = 2; // PIN ENCODER A
char EncB = 3; // PIN ENCODER B
volatile long posicionActual = 0; // Posición actual del encoder
long SetPoint = 180; // Posición objetivo

// Constantes PID
float Kp = 0.45;
float Ki = 0;
float Kd = 0;
float error1 = 0;
float error2 = 0;
float error3 = 0;
float salidaActual = 0;
float salidaPrevia = 0;

// Funciones del encoder
void InterrupcionA() {
  if (digitalRead(EncA) == digitalRead(EncB)) {
    posicionActual++;
  } else {
    posicionActual--;
  }
}

void InterrupcionB() {
  if (digitalRead(EncA) != digitalRead(EncB)) {
    posicionActual++;
  } else {
    posicionActual--;
  }
}

void LCD() {
  lcd.setCursor(0, 1);
  lcd.print("Pos: ");
  lcd.print(posicionActual);
}

void setup() {
  // Inicializamos el LCD
  lcd.begin(8, 2);
  lcd.setCursor(0, 0);
  lcd.print("U.CUENCA");
  lcd.setCursor(0, 1);
  lcd.print(" P.I.D. ");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("POSICION:");

  // Inicializar comunicación serial
  Serial.begin(9600);

  // Configuración de pines del motor
  pinMode(SalidaPWM, OUTPUT);
  pinMode(SentGiroPos, OUTPUT);
  pinMode(SentGiroNeg, OUTPUT);
  pinMode(EncA, INPUT);
  pinMode(EncB, INPUT);

  // Configuración de interrupciones para el encoder
  attachInterrupt(digitalPinToInterrupt(EncA), InterrupcionA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncB), InterrupcionB, CHANGE);

  // Inicializar el motor apagado
  digitalWrite(SentGiroPos, LOW);
  digitalWrite(SentGiroNeg, LOW);
  analogWrite(SalidaPWM, 0);

  espSerial.begin(9600);
}

void loop() {
  // Leer una tecla del teclado
  char teclaPresionada = teclado.getKey();

  // Si se presiona una tecla y no es '*', y la cadena tiene menos de 6 caracteres
  if (teclaPresionada && teclaPresionada != '*' && teclaPresionada != '#' && cadenaTeclas.length() < 6) {
    cadenaTeclas += teclaPresionada;

    // Mostrar la cadena completa en el LCD
    lcd.setCursor(0, 1);
    lcd.print("        ");
    lcd.setCursor(0, 1);
    lcd.print(cadenaTeclas);
  }

  // Si se presiona '*', usar el valor ingresado como SetPoint
  if (teclaPresionada == '*') {
    SetPoint = cadenaTeclas.toInt(); // Convertir la cadena a entero
    lcd.setCursor(0, 0);
    lcd.print("Set: ");
    lcd.print(SetPoint);

    // Reiniciar la cadena
    cadenaTeclas = "";
  }

  // Control del motor basado en posición
  error3 = error2;
  error2 = error1;
  error1 = SetPoint - posicionActual;

  salidaActual = salidaPrevia + Kp * (error1 - error2) + Ki * error1 + Kd * (error1 - 2 * error2 + error3);
  salidaPrevia = salidaActual;

  salidaActual = constrain(salidaActual, -255, 255);

  if (salidaActual > 0) {
    digitalWrite(SentGiroPos, HIGH);
    digitalWrite(SentGiroNeg, LOW);
    analogWrite(SalidaPWM, salidaActual);
  } else {
    digitalWrite(SentGiroPos, LOW);
    digitalWrite(SentGiroNeg, HIGH);
    analogWrite(SalidaPWM, -salidaActual);
  }

  LCD();
  delay(50);
}
