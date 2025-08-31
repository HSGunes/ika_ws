// *** MEGA 2560 ZORUNLU ***
#if !defined(__AVR_ATmega2560__)
  #error "Tools > Board: 'Arduino Mega 2560' secilmelidir."
#endif

// ===================== DC MOTOR PINLERI (IKA - 6 Tekerlek) =====================
// Sol taraf tekerlekler
#define LPWM1 53  // Sol ön tekerlek
#define RPWM1 52
#define LEN1  51
#define REN1  50

#define LPWM2 49  // Sol orta tekerlek
#define RPWM2 48
#define LEN2  47
#define REN2  46

#define LPWM3 45  // Sol arka tekerlek
#define RPWM3 44
#define LEN3  43
#define REN3  42

// Sağ taraf tekerlekler
#define LPWM4 41  // Sağ ön tekerlek
#define RPWM4 40
#define LEN4  39
#define REN4  38

#define LPWM5 37  // Sağ orta tekerlek
#define RPWM5 36
#define LEN5  35
#define REN5  34

#define LPWM6 33  // Sağ arka tekerlek
#define RPWM6 32
#define LEN6  31
#define REN6  30

// ===================== STEPPER PINLERI (Direksiyon) ======================
#define STEP1 28  // Sol ön direksiyon
#define DIR1  29
#define STEP2 26  // Sağ ön direksiyon
#define DIR2  27

// =========== STEPPER ENABLE PINLERI (DM860H icin) ==========
#define ENA1_PLUS 12  // Sol ön direksiyon enable
#define ENA2_PLUS 11  // Sağ ön direksiyon enable

// ===================== RC KANAL PINLERI =====================
#define CH2 A2   // ileri/geri (DC)
#define CH1 A1   // sag/sol (Stepper)
#define CH7 A3   // Ateş (opsiyonel)
#define CH6 A4   // Fren (opsiyonel)

// ===================== AYARLAR ==============================
const unsigned long RC_TIMEOUT_US = 40000; // 40ms
const int stepDelayUs = 10;                // Stepper pulse gecikmesi

// PWM rampa
int mevcutPWM = 0;
int hedefPWM  = 0;
const int rampaHizi = 5;

// STEP ayarlari
const int stepsPerRev        = 200;
const int microstep          = 8;
const int stepsFor60Degrees  = (stepsPerRev * microstep) * 60 / 360;

// Durum
unsigned long ileri, stepmotordonus;
unsigned long atis, frenKomutu;
bool frenDurumu = false;

// Serial komut değişkenleri
String serialCommand = "";
bool serialCommandComplete = false;
float targetSteerAngle = 0.0;
int targetSpeed = 0;

// ===================== YARDIMCI =============================
static inline unsigned long readRC(uint8_t pin){
  unsigned long v = pulseIn(pin, HIGH, RC_TIMEOUT_US);
  if (v < 800 || v > 2200) return 0;
  return v;
}

static inline void stepPulse(uint8_t PUL, int delayUs){
  digitalWrite(PUL, HIGH); delayMicroseconds(delayUs);
  digitalWrite(PUL, LOW ); delayMicroseconds(delayUs);
}

static inline void stepMove(uint8_t PUL, uint8_t DIR, bool dirHigh, int steps, int delayUs){
  digitalWrite(DIR, dirHigh ? HIGH : LOW);
  for (int i=0; i<steps; i++) stepPulse(PUL, delayUs);
}

void pulseAllSteppers() {
  digitalWrite(STEP1, HIGH); digitalWrite(STEP2, HIGH);
  delayMicroseconds(stepDelayUs);
  digitalWrite(STEP1, LOW ); digitalWrite(STEP2, LOW );
  delayMicroseconds(stepDelayUs);
}

// =========================== SETUP ==========================
void setup() {
  Serial.begin(115200);

  // DC motor pin modlari
  pinMode(LPWM1, OUTPUT); pinMode(RPWM1, OUTPUT);
  pinMode(LEN1,  OUTPUT); pinMode(REN1,  OUTPUT);
  pinMode(LPWM2, OUTPUT); pinMode(RPWM2, OUTPUT);
  pinMode(LEN2,  OUTPUT); pinMode(REN2,  OUTPUT);
  pinMode(LPWM3, OUTPUT); pinMode(RPWM3, OUTPUT);
  pinMode(LEN3,  OUTPUT); pinMode(REN3,  OUTPUT);
  pinMode(LPWM4, OUTPUT); pinMode(RPWM4, OUTPUT);
  pinMode(LEN4,  OUTPUT); pinMode(REN4,  OUTPUT);
  pinMode(LPWM5, OUTPUT); pinMode(RPWM5, OUTPUT);
  pinMode(LEN5,  OUTPUT); pinMode(REN5,  OUTPUT);
  pinMode(LPWM6, OUTPUT); pinMode(RPWM6, OUTPUT);
  pinMode(LEN6,  OUTPUT); pinMode(REN6,  OUTPUT);

  // Enable pinleri aktif
  digitalWrite(LEN1, HIGH); digitalWrite(REN1, HIGH);
  digitalWrite(LEN2, HIGH); digitalWrite(REN2, HIGH);
  digitalWrite(LEN3, HIGH); digitalWrite(REN3, HIGH);
  digitalWrite(LEN4, HIGH); digitalWrite(REN4, HIGH);
  digitalWrite(LEN5, HIGH); digitalWrite(REN5, HIGH);
  digitalWrite(LEN6, HIGH); digitalWrite(REN6, HIGH);

  // Stepper pinleri
  pinMode(STEP1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(STEP2, OUTPUT); pinMode(DIR2, OUTPUT);

  // Stepper EN (DM860H: LOW=aktif)
  pinMode(ENA1_PLUS, OUTPUT); digitalWrite(ENA1_PLUS, LOW);
  pinMode(ENA2_PLUS, OUTPUT); digitalWrite(ENA2_PLUS, LOW);

  // RC pinleri
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH6, INPUT);
  pinMode(CH7, INPUT);

  Serial.println("IKA Arduino Controller Ready");
}

// ============================ LOOP ==========================
void loop() {
  // Serial komutları kontrol et
  if (serialCommandComplete) {
    parseSerialCommand(serialCommand);
    serialCommand = "";
    serialCommandComplete = false;
  }
  
  // RC okumaları (manuel kontrol için)
  ileri          = readRC(CH2);   // A2
  stepmotordonus = readRC(CH1);   // A1
  atis           = readRC(CH7);   // A3
  frenKomutu     = readRC(CH6);   // A4

  // Debug bilgisi
  Serial.print("RC - CH2(A2) ileri="); Serial.print(ileri);
  Serial.print("  CH1(A1) donus="); Serial.print(stepmotordonus);
  Serial.print("  CH6(A4) fren="); Serial.print(frenKomutu);
  Serial.println();

  // RC kontrolü (serial komut yoksa)
  if (targetSpeed == 0 && targetSteerAngle == 0.0) {
    // RC ile kontrol
    handleRCMovement();
  } else {
    // Serial komut ile kontrol
    handleSerialMovement();
  }

  delay(50);
}

void handleRCMovement() {
  // ---- PWM hedef/rampa (yalniz DC taraf) ----
  if (ileri < 1330 && ileri > 0) {
    hedefPWM = constrain(map((int)ileri, 1380, 970, 0, 170), 0, 170);
  } else if (ileri > 1430) {
    hedefPWM = constrain(map((int)ileri, 1430, 1900, 0, 170), 0, 170);
  } else {
    hedefPWM = 0;
  }
  
  // PWM rampa
  if (mevcutPWM < hedefPWM) {
    mevcutPWM += rampaHizi; if (mevcutPWM > hedefPWM) mevcutPWM = hedefPWM;
  } else if (mevcutPWM > hedefPWM) {
    mevcutPWM -= rampaHizi; if (mevcutPWM < hedefPWM) mevcutPWM = hedefPWM;
  }

  // ---- DC Motor Kontrol (IKA - 6 tekerlek) ----
  if (ileri < 1330 && ileri > 0) { // ileri
    analogWrite(RPWM1, mevcutPWM); analogWrite(LPWM1, 0);  // Sol ön
    analogWrite(RPWM2, mevcutPWM); analogWrite(LPWM2, 0);  // Sol orta
    analogWrite(RPWM3, mevcutPWM); analogWrite(LPWM3, 0);  // Sol arka
    analogWrite(RPWM4, mevcutPWM); analogWrite(LPWM4, 0);  // Sağ ön
    analogWrite(RPWM5, mevcutPWM); analogWrite(LPWM5, 0);  // Sağ orta
    analogWrite(RPWM6, mevcutPWM); analogWrite(LPWM6, 0);  // Sağ arka
  } else if (ileri > 1430) { // geri
    analogWrite(RPWM1, 0); analogWrite(LPWM1, mevcutPWM);  // Sol ön
    analogWrite(RPWM2, 0); analogWrite(LPWM2, mevcutPWM);  // Sol orta
    analogWrite(RPWM3, 0); analogWrite(LPWM3, mevcutPWM);  // Sol arka
    analogWrite(RPWM4, 0); analogWrite(LPWM4, mevcutPWM);  // Sağ ön
    analogWrite(RPWM5, 0); analogWrite(LPWM5, mevcutPWM);  // Sağ orta
    analogWrite(RPWM6, 0); analogWrite(LPWM6, mevcutPWM);  // Sağ arka
  } else { // dur
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
    analogWrite(RPWM4, 0); analogWrite(LPWM4, 0);
    analogWrite(RPWM5, 0); analogWrite(LPWM5, 0);
    analogWrite(RPWM6, 0); analogWrite(LPWM6, 0);
  }

  // ---- STEPPER KONTROL (direksiyon) ----
  if (stepmotordonus > 1500) {
    digitalWrite(DIR1, LOW);  digitalWrite(DIR2, LOW);
    pulseAllSteppers();
  } else if (stepmotordonus < 1400 && stepmotordonus > 0) {
    digitalWrite(DIR1, HIGH); digitalWrite(DIR2, HIGH);
    pulseAllSteppers();
  }
}

void handleSerialMovement() {
  // Serial komut ile hareket
  int speedPWM = abs(targetSpeed);
  if (speedPWM > 170) speedPWM = 170;  // Maksimum sınır
  
  // DC Motor Kontrol
  if (targetSpeed > 0) { // ileri
    analogWrite(RPWM1, speedPWM); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, speedPWM); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, speedPWM); analogWrite(LPWM3, 0);
    analogWrite(RPWM4, speedPWM); analogWrite(LPWM4, 0);
    analogWrite(RPWM5, speedPWM); analogWrite(LPWM5, 0);
    analogWrite(RPWM6, speedPWM); analogWrite(LPWM6, 0);
  } else if (targetSpeed < 0) { // geri
    analogWrite(RPWM1, 0); analogWrite(LPWM1, speedPWM);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, speedPWM);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, speedPWM);
    analogWrite(RPWM4, 0); analogWrite(LPWM4, speedPWM);
    analogWrite(RPWM5, 0); analogWrite(LPWM5, speedPWM);
    analogWrite(RPWM6, 0); analogWrite(LPWM6, speedPWM);
  } else { // dur
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
    analogWrite(RPWM4, 0); analogWrite(LPWM4, 0);
    analogWrite(RPWM5, 0); analogWrite(LPWM5, 0);
    analogWrite(RPWM6, 0); analogWrite(LPWM6, 0);
  }
  
  // Direksiyon kontrolü
  if (abs(targetSteerAngle) > 0.1) {
    if (targetSteerAngle > 0) {
      digitalWrite(DIR1, LOW);  digitalWrite(DIR2, LOW);
    } else {
      digitalWrite(DIR1, HIGH); digitalWrite(DIR2, HIGH);
    }
    pulseAllSteppers();
  }
}

void parseSerialCommand(String command) {
  // Komut formatı: "STEER:angle,SPEED:speed"
  if (command.startsWith("STEER:")) {
    int steerIndex = command.indexOf("STEER:");
    int speedIndex = command.indexOf("SPEED:");
    
    if (steerIndex >= 0 && speedIndex >= 0) {
      // Direksiyon açısını parse et
      String steerStr = command.substring(steerIndex + 6, speedIndex - 1);
      targetSteerAngle = steerStr.toFloat();
      
      // Hızı parse et
      String speedStr = command.substring(speedIndex + 6);
      targetSpeed = speedStr.toInt();
      
      Serial.print("CMD_ACK: STEER="); Serial.print(targetSteerAngle);
      Serial.print(", SPEED="); Serial.println(targetSpeed);
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      serialCommandComplete = true;
    } else {
      serialCommand += inChar;
    }
  }
}
