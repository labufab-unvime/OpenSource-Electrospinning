// Código Electrohilado
/*-----------------------------------------------------------------------------------------------------------------------------------------------
 * Proyecto del Club Estudiantil de Bioingeniería, realizado en el año 2025 por los integrantes Lorenzo Tell, Carolina Peñeñory, Lourdes Casale,
 * Gamaliel Soria, Carla Moran, con la compañia de las profesoras Dra. Helga Blanco y Dra. Tania Rodriguez
*
*
 */
 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

#define EN_PIN      6
#define STEP_PIN    7
#define DIR_PIN     4
#define MOS_PIN     9
#define ENC_A       2
#define ENC_B       3
#define ENC_SW      8
#define DOOR_SW     0
#define RELAY1_PIN  5
#define RELAY2_PIN 10

#define LED_GREEN   11
#define LED_RED     12
#define LED_YELLOW  13

#define CORRIENTE_PIN A1

#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

volatile int encoderDelta = 0;
volatile unsigned long lastInterruptTime = 0;
const unsigned int debounceTime = 2000;

bool lastButton = HIGH;
unsigned long lastDebounce = 0;

int estado = 0;  
int subMenuIndex = 0;
float flujoValor = 1.0;   // en ml/h (se muestra asi en pantalla)
bool flujoOn = false;
float tensionValor = 1.0; // en kV
bool tensionOn = false;
float tempFlujoValor;
bool editing = false;
float tempTensionValor;

// maxOptions por estado: 0=Menu principal (4 items), 1=Flujo (3 items), 2=Tension (3 items), 3=Ejecucion(1), 4=Jeringa(1)
const int maxOptions[] = {4, 3, 3, 1, 1};

// --- Control de flujo ---
unsigned long lastStepTime = 0;
float pasosPorSegundo = 0;
unsigned long intervaloPasoMicros = 0;
bool motorDir = true;

// Datos jeringas
float diametroJeringa = 8.26; // mm (default 3 ml)
int tipoJeringa = 0; // 0 = 3ml (8.26), 1 = 5ml (14.7)

// Variables para calibración
bool calibrando = false;
bool calibracionCompletada = false;
int dutyCalibrado = 0;
const float errorUmbral = 0.05;
float lecturaTension = 0.0;
unsigned long inicioCalibracion = 0;
const unsigned long tiempoMaxCalibracion = 2000;

bool alarmaCorriente = false;
unsigned long ultimaMedicionCorriente = 0;
const unsigned long intervaloMedicionCorriente = 100;
const int umbralCorrienteDC = 100;
int contadorFallo = 0;
const int maxContadorFallo = 3;

void setup() {
  Serial.begin(9600);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOS_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  // Estado inicial relés: RELAY2 = LOW (desactivado) porque en tu HW D10=HIGH activa el relé
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(DOOR_SW, INPUT_PULLUP);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  // PWM configuración original (mantener)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
  TCCR1B |= (1 << CS10);
  ICR1 = 75;
  OCR1A = ICR1;
  
  // Interrupción encoder (A)
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  lcd.begin();
  lcd.backlight();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Electrospinning");
  delay(1200);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CEB - LabµFab");
  delay(1200);
  lcd.clear();
  drawMenu();
  actualizarFlujo(); // calcular intervalos iniciales
}

void loop() {
  bool doorClosed = (digitalRead(DOOR_SW) == LOW);

  // LEDs de estado
  digitalWrite(LED_GREEN, (doorClosed && !alarmaCorriente) ? HIGH : LOW);
  digitalWrite(LED_RED, (!doorClosed || alarmaCorriente) ? HIGH : LOW);
  digitalWrite(LED_YELLOW, (estado != 3 && !alarmaCorriente) ? HIGH : LOW);

  // Si hay alarma, mostramos y salimos rápido
  if (alarmaCorriente) {
    drawMenu();
    delay(50);
    return;
  }

  // Encoder (delta actualizado por ISR)
  if (encoderDelta != 0) {
    handleEncoder();
    encoderDelta = 0;
  }
  
  handleEncoderButton(doorClosed);

  // Seguridad: puerta abierta durante operación
  if ((estado == 3 || tensionOn) && !doorClosed) {
    apagarSistema();
    estado = 0;
  }

  // Control de tensión (aqui se activa RELAY2 y RELAY1 según condiciones)
  controlTensionSegura(doorClosed);

  // Protección corriente sólo si está tensión y en ejecución
  if (tensionOn && estado == 3) {
    verificarCorriente();
  }

  // Movimiento de motor por flujo
  if (flujoOn) {
    moverMotorFlujo();
  }

  drawMenu();
  delay(50);
}

// ---------------- ISR y manejo encoder ----------------
void encoderISR() {
  unsigned long now = micros();
  if (now - lastInterruptTime < debounceTime) return;
  lastInterruptTime = now;
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  if (!a) encoderDelta += (b ? 1 : -1);
}

void handleEncoder() {
  if (alarmaCorriente) return;
  
  if (editing) {
    // Cuando estamos editando, el encoder cambia el valor
    if (estado == 1) {
      tempFlujoValor = constrain(tempFlujoValor + encoderDelta * 0.1, 0.1, 50.0); // amplié tope por si quieres más rango
    } else if (estado == 2) {
      tempTensionValor = constrain(tempTensionValor + encoderDelta * 0.1, 1.0, 30.0);
    }
  } else {
    int opciones = 0;
    if (estado >= 0 && estado <= 4) opciones = maxOptions[estado];
    if (opciones > 0) {
      subMenuIndex = (subMenuIndex + encoderDelta + opciones) % opciones;
    }
  }
}

void handleEncoderButton(bool doorClosed) {
  if (alarmaCorriente) return;
  
  bool reading = digitalRead(ENC_SW);
  if (reading == LOW && lastButton == HIGH && (millis() - lastDebounce) > 200) {
    lastDebounce = millis();
    
    if (editing) {
      // Guardar valor editado
      editing = false;
      if (estado == 1) {
        flujoValor = tempFlujoValor;
        actualizarFlujo();
      } else if (estado == 2) {
        tensionValor = tempTensionValor;
        calibracionCompletada = false;
      }
    } else {
      // Acciones por estado y subMenuIndex
      switch (estado) {
        case 0: // Menú principal: subMenuIndex navega por: 0=Flujo,1=Tension,2=Iniciar,3=Jeringa
          if (subMenuIndex == 3) {
            estado = 4; // ir a selección jeringa
            subMenuIndex = 0;
          } else {
            if (subMenuIndex == 2 && !doorClosed) {
              // intentar Iniciar pero puerta abierta => ignorar
              break;
            }
            estado = (subMenuIndex < 2) ? (subMenuIndex + 1) : 3; // 0->1(Flujo),1->2(Tension),2->3(Iniciar)
            if (subMenuIndex == 2) {
              // Iniciar: activar flujo y tension
              flujoOn = true;
              tensionOn = true;
              calibracionCompletada = false;
              actualizarFlujo();
            }
            subMenuIndex = 0;
          }
          break;
          
        case 1: // Submenu Flujo
          if (subMenuIndex == 0) { // Valor
            editing = true;
            tempFlujoValor = flujoValor;
          }
          else if (subMenuIndex == 1) { // Encendido / Apagado
            flujoOn = !flujoOn;
            if (flujoOn) actualizarFlujo();
          }
          else { // Volver
            estado = 0;
            subMenuIndex = 0;
          }
          break;
          
        case 2: // Submenu Tension
          if (subMenuIndex == 0) { // Valor
            editing = true;
            tempTensionValor = tensionValor;
            calibracionCompletada = false;
          }
          else if (subMenuIndex == 1) { // Estado
            if (doorClosed) {
              tensionOn = !tensionOn;
              if (tensionOn) {
                calibracionCompletada = false;
              } else {
                apagarSistema();
              }
            }
          }
          else { // Volver
            estado = 0;
            subMenuIndex = 0;
          }
          break;
          
        case 3: // Ejecucion - boton Detener
          apagarSistema();
          estado = 0;
          break;

        case 4: // Selección Jeringa: al presionar alterna y vuelve al menú
          tipoJeringa = (tipoJeringa + 1) % 2;
          diametroJeringa = (tipoJeringa == 0) ? 8.26 : 14.7;
          actualizarFlujo();
          estado = 0;
          subMenuIndex = 0;
          break;
      }
    }
  }
  lastButton = reading;
}

// ---------------- Sistema / Relés / Tensión ----------------
void apagarSistema() {
  flujoOn = false;
  tensionOn = false;
  OCR1A = ICR1;
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW); // RELAY2 LOW = desactivado en tu HW (D10 HIGH activa)
  subMenuIndex = 0;
}

void controlTensionSegura(bool doorClosed) {
  // condiciones para operar tensión: tensionOn true, puerta cerrada, sin alarma
  if (!tensionOn || !doorClosed || alarmaCorriente) {
    // aseguramos RELAY2 y RELAY1 apagados si no están las condiciones
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    return;
  }

  // Activar RELAY2 (conecta rama flyback a GND via MOSFET) - tu HW: D10 HIGH activa
  digitalWrite(RELAY2_PIN, HIGH);

  if (!calibracionCompletada) {
    if (!calibrando) {
      calibrando = true;
      // activar RELAY1 para medición durante calibración
      digitalWrite(RELAY1_PIN, HIGH);
      inicioCalibracion = millis();
    } else {
      // lectura y control proporcional simple
      float setpointV = tensionValor * (3.0 / 30.0); // escala original
      lecturaTension = analogRead(A0) * (5.0 / 1023.0);
      float error = setpointV - lecturaTension;
      
      const float Kp = 100.0;
      int duty = constrain((int)(error * Kp), 0, ICR1);
      dutyCalibrado = ICR1 - duty;
      OCR1A = dutyCalibrado;

      if (abs(error) < errorUmbral || (millis() - inicioCalibracion) > tiempoMaxCalibracion) {
        calibrando = false;
        calibracionCompletada = true;
        digitalWrite(RELAY1_PIN, LOW); // desconectar medición
      }
    }
  } else {
    // modo mantenido: aplicar duty calibrado
    OCR1A = dutyCalibrado;
  }
}

void verificarCorriente() {
  unsigned long ahora = millis();
  if (ahora - ultimaMedicionCorriente >= intervaloMedicionCorriente) {
    ultimaMedicionCorriente = ahora;
    
    int valorCorriente = analogRead(CORRIENTE_PIN);
    
    if (valorCorriente > umbralCorrienteDC) {
      contadorFallo++;
      if (contadorFallo >= maxContadorFallo) {
        alarmaCorriente = true;
        apagarSistema();
      }
    } else {
      contadorFallo = 0;
    }
  }
}

// ---------------- Dibujado del menú (UNA SOLA FORMA: encabezado fija, linea2 navegable) ----------------
void drawMenu() {
  lcd.clear();
  
  if (alarmaCorriente) {
    lcd.setCursor(0,0);
    lcd.print("!ALARMA CORRIENTE!");
    lcd.setCursor(0,1);
    lcd.print(" FALLO MOSFET ");
    return;
  }
  
  switch (estado) {
    case 0: // Menu principal: encabezado fijo "Menu", linea2 muestra 1 item (con >)
      lcd.setCursor(0,0);
      lcd.print("Menu:");
      lcd.setCursor(0,1);
      if (subMenuIndex == 0) lcd.print("> Flujo");
      else if (subMenuIndex == 1) lcd.print("> Tension");
      else if (subMenuIndex == 2) lcd.print("> Iniciar");
      else if (subMenuIndex == 3) lcd.print("> Jeringa");
      break;
      
    case 1: // Submenu Flujo -> encabezado "Flujo", linea2 muestra solo la opcion seleccionada
      lcd.setCursor(0,0); lcd.print("Flujo");
      lcd.setCursor(0,1);
      if (editing) {
        // edición: mostrar valor en segunda linea
        lcd.print("Set: ");
        lcd.print(tempFlujoValor,1);
        lcd.print(" ml/h");
      } else {
        if (subMenuIndex == 0) {
          lcd.print("> Valor: ");
          lcd.print(flujoValor,1);
          lcd.print(" ml/h");
        } else if (subMenuIndex == 1) {
          lcd.print("> Estado: ");
          lcd.print(flujoOn ? "ON" : "OFF");
        } else {
          lcd.print("> Volver");
        }
      }
      break;
      
    case 2: // Submenu Tension
      lcd.setCursor(0,0); lcd.print("Tension");
      lcd.setCursor(0,1);
      if (editing) {
        lcd.print("Set: ");
        lcd.print(tempTensionValor,1);
        lcd.print(" kV");
      } else {
        if (calibrando) {
          lcd.print("Calib: ");
          lcd.print(lecturaTension * 10.0, 1);
          lcd.print("kV");
        } else {
          if (subMenuIndex == 0) {
            lcd.print("> Valor: ");
            lcd.print(tensionValor,1);
            lcd.print(" kV");
          } else if (subMenuIndex == 1) {
            lcd.print("> Estado: ");
            lcd.print(tensionOn ? "ON" : "OFF");
          } else {
            lcd.print("> Volver");
          }
        }
      }
      break;
      
    case 3: // Ejecución
      lcd.setCursor(0,0); lcd.print("Ejecucion");
      lcd.setCursor(0,1);
      if (calibrando) lcd.print("Calibrando...");
      else {
        lcd.print("T:");
        lcd.print(tensionValor,1);
        lcd.print("kV F:");
        lcd.print(flujoOn ? "ON" : "OFF");
      }
      break;

    case 4: // Seleccion jeringa
      lcd.setCursor(0,0); lcd.print("Jeringa");
      lcd.setCursor(0,1);
      if (tipoJeringa == 0) lcd.print("> 3ml (8.26mm)");
      else lcd.print("> 5ml (14.7mm)");
      break;
      
    default:
      lcd.setCursor(0,0); lcd.print("Menu:");
      lcd.setCursor(0,1); lcd.print("> Flujo");
      break;
  }
}

// ---------------- Flujo <-> motor ----------------
void actualizarFlujo() {
  // flujoValor en ml/h
  float radio = diametroJeringa / 2.0;
  float area = M_PI * radio * radio; // mm²
  float mmPorMl = 1000.0 / area; // mm de avance por ml
  float pasosPorMm = 200.0 * 16.0; // motor 200 pasos, 16 microsteps
  float pasosPorMl = pasosPorMm * mmPorMl;
  
  pasosPorSegundo = (flujoValor * pasosPorMl) / 3600.0; // ml/h -> pasos/s
  if (pasosPorSegundo > 0.00001) intervaloPasoMicros = (unsigned long)(1000000.0 / pasosPorSegundo);
  else intervaloPasoMicros = 0;
}

void moverMotorFlujo() {
  if (!flujoOn || intervaloPasoMicros == 0) return;
  
  unsigned long ahora = micros();
  if (ahora - lastStepTime >= intervaloPasoMicros) {
    lastStepTime = ahora;
    digitalWrite(EN_PIN, LOW);
    digitalWrite(DIR_PIN, motorDir ? HIGH : LOW);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
  }
}
