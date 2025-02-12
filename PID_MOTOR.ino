#include <LiquidCrystal.h>
#include <Keypad.h>

// Configuración del LCD: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

// Configuración del teclado matricial 3x4
const byte FILAS = 4; // Número de filas
const byte COLUMNAS = 3; // Número de columnas
#include <SoftwareSerial.h>

// Configura SoftwareSerial en pines 10 (RX) y 11 (TX)
SoftwareSerial espSerial(15, 14);
char conta = 0,conta2=0, bandera=0, conta3=0; // Contador de caracteres

// Definimos las teclas del teclado (en su disposición)
char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// Pines conectados a las filas y columnas del teclado
byte pinesFilas[FILAS] = {A0, A1, A2, A3}; // Pines de las filas
byte pinesColumnas[COLUMNAS] = {A4, A5, A6}; // Pines de las columnas

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
volatile unsigned long tiempoInicial = 0; // Tiempo inicial para cálculo RPM
volatile unsigned long tiempoFinal = 0; // Tiempo Final para cálculo RPM
volatile unsigned long tiempoPulso = 0; // Tiempo entre flancos del encoder
float w = 0; // Velocidad angular
float RPM = 0;
volatile int cuentaPulsos = 0; // Cuenta los pulsos del encoder

// Constantes PID
float SetPoint = 0; // Variable que se actualizará con el valor ingresado
float Kp = 10; // Ganancia proporcional
float Ki = 0.7; // Ganancia Integral 
float Kd = 0.65555; // Ganancia Derivativa
float Ts = 0.25; // Tiempo de establecimiento
float MedidaActual = 0; // Valor de entrada al controlador
float error1 = 0;
float error2 = 0;
float error3 = 0;
float salidaActual = 0; // Salida actual del controlador
float salidaPrevia = 0; // Salida previa del controlador 

// Funciones del motor
float CalculaRPM(void);
void Interrupcion() {
  cuentaPulsos++;
}

void LCD(){
    lcd.setCursor(0, 1);
    lcd.print(RPM);
    lcd.print("RPM");
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
  lcd.print("R.P.M:  ");

  // Inicializar comunicación serial
  Serial.begin(9600);

  // Configuración de pines del motor
  pinMode(SalidaPWM, OUTPUT);
  pinMode(SentGiroPos, OUTPUT);
  pinMode(SentGiroNeg, OUTPUT);
  pinMode(EncA, INPUT);
  pinMode(EncB, INPUT);

  // Configuración de interrupción para el encoder
  attachInterrupt(digitalPinToInterrupt(EncA), Interrupcion, RISING);

  // Activar el motor inicialmente apagado
  digitalWrite(SentGiroPos, LOW); 
  digitalWrite(SentGiroNeg, HIGH);
  analogWrite(SalidaPWM, 0); // Motor apagado
  espSerial.begin(9600);  
}

void loop() {
  // Leer una tecla del teclado
  char teclaPresionada = teclado.getKey();

  // Si se presiona una tecla y no es '*', y la cadena tiene menos de 3 caracteres
  if (teclaPresionada && teclaPresionada != '*' && teclaPresionada != '#' && conta < 3) {
    // Agregar la tecla presionada a la cadena
    cadenaTeclas += teclaPresionada;
    conta++; // Incrementar el contador

    // Mostrar la cadena completa en el LCD
    lcd.setCursor(0, 1); // Posición: columna 0, fila 1
    lcd.print("        "); // Limpiar la línea
    lcd.setCursor(0, 1);
    lcd.print(cadenaTeclas); // Mostrar la cadena de teclas
  }

  // Si se presiona '*', usar el valor ingresado como SetPoint
  if (teclaPresionada == '*') {
    // Convertir la cadena a un número y usarlo como SetPoint
    SetPoint = cadenaTeclas.toFloat();

    // Mostrar SetPoint en el LCD
    lcd.setCursor(0, 0);
    lcd.print("Set: ");
    lcd.print(SetPoint);

    // Reiniciar la cadena
    cadenaTeclas = "";
    conta = 0;
    bandera=0;
  }

  if (teclaPresionada == '#') {
    // Convertir la cadena a un número y usarlo como SetPoint
    bandera=1;
    conta2=8;
    lcd.setCursor(0, 1);
    lcd.print("        ");
  }

  // Control del motor
  if (SetPoint > 0) {
    conta2=conta2+1;

    if (conta2==10 && bandera==0){
       LCD();
       conta2=0;
    }

   

    MedidaActual = CalculaRPM();

    error3 = error2;
    error2 = error1;
    error1 = SetPoint - MedidaActual;

    salidaActual = salidaPrevia + (Kp + (Kd / Ts)) * error1 +
                   (-Kp + Ki * Ts - 2 * (Kd / Ts)) * error2 +
                   (Kd / Ts) * error3;

    salidaPrevia = salidaActual;

    salidaActual = constrain(salidaActual, 0, 255);

    analogWrite(SalidaPWM, (int)salidaActual);
  } else {
    analogWrite(SalidaPWM, 0); // Apagar motor si no hay SetPoint
  }

}

float CalculaRPM(void) {
  float t = 50; // Tiempo de verificación
  tiempoFinal = millis();

  if ((tiempoFinal - tiempoInicial) >= t) {
    tiempoPulso = tiempoFinal - tiempoInicial;
    w = (cuentaPulsos * ((2 * 3.141592) / 240)) / ((float)tiempoPulso / 1000);
    cuentaPulsos = 0;
    tiempoInicial = tiempoFinal;
  }

  RPM = w * 9.55;
  String marca = String(SetPoint) ;
  Serial.print("0 ");  
  Serial.print(marca);   // Valor mínimo
  Serial.print("300 ");
  
  //delay(50);
  Serial.println(RPM);
    
    conta3=conta3+1;
    if (conta3==15){
       String num = String(RPM, 2) + " RPM";
      espSerial.println(num);
       conta3=0;
    }

  
  return RPM;
}
