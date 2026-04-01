
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

#define IR_LEFT   A0   // 좌측 라인센서 (라인 위 = HIGH)
#define IR_RIGHT  A1   // 우측 라인센서

// 장애물 IR 센서 (디지털, Active-LOW 또는 Active-HIGH 모듈에 따라 조정)
#define OBS_LEFT  A2
#define OBS_CENTER A3
#define OBS_RIGHT A4

// 좌측 모터 (L298N IN1/IN2 + ENA)
#define ML_IN1  2
#define ML_IN2  3
#define ML_EN   5   // PWM 핀

// 우측 모터 (L298N IN3/IN4 + ENB)
#define MR_IN1  6
#define MR_IN2  7
#define MR_EN   8   // PWM 핀 (Mega 사용시 8번도 PWM 가능)

// Lifter 서보
#define SERVO_PIN 9

// RFID
#define RFID_SS   10
#define RFID_RST  A5

// ============================================================
//  상수 정의
// ============================================================
// 모터 속도 (0~255)
#define SPEED_NORMAL  160   // 직진 속도
#define SPEED_TURN    110   // 회전 속도
#define SPEED_SLOW    100   // 감속 속도 (격자 진입 시)

// IR 센서 임계값 (아날로그)
// 흰 바닥 ≈ 800~1023, 검은 라인 ≈ 0~400 (모듈 특성에 따라 조정)
#define LINE_THRESHOLD 500

// 라인 상태가 없는 교차점 감지용 카운트
#define GRID_PULSE     30   // 교차점 통과 시 양쪽 센서가 라인을 밟는 횟수 카운트

// Lifter 서보 각도
#define LIFTER_UP   90    // 팔렛을 들어올린 각도
#define LIFTER_DOWN 0     // 팔렛을 내려놓은 각도

// ============================================================
//  전역 객체 및 변수
// ============================================================
MFRC522 rfid(RFID_SS, RFID_RST);
Servo lifterServo;

// -------- 미션 설정 (경기 당일 수정 필요) --------
// 방향: 0=북(위), 1=동(우), 2=남(아래), 3=서(좌)
// 좌표: 행 A=0, B=1, C=2 ... / 열 1=0, 2=1, 3=2 ...

struct Coord {
  int row;  // 0=A, 1=B, 2=C ...
  int col;  // 0=1열, 1=2열 ...
};

// 출발 위치 (경기 당일 확인 후 수정)
Coord START_POS = {3, 0};  // 예: D1

// 물류창고 좌표 목록 (경기 당일 미션 공개 후 수정)
// 팔렛 5개 순서대로 입력
Coord WAREHOUSE[5] = {
  {1, 1},  // 예: B2
  {1, 3},  // 예: B4
  {2, 1},  // 예: C2
  {3, 2},  // 예: D3
  {0, 2}   // 예: A3
};

// 행선지 좌표 (경기 당일 미션 공개 후 수정)
Coord DESTINATION[5] = {
  {0, 4},  // 예: 서울
  {4, 0},  // 예: 부산
  {0, 4},  // 예: 서울
  {2, 4},  // 예: 대전
  {4, 0}   // 예: 부산
};

// 팔렛 RFID UID (경기 당일 공개 후 수정, 16진수 배열)
// 각 팔렛의 UID 4바이트
byte PALLET_UID[5][4] = {
  {0xAA, 0xBB, 0xCC, 0x01},
  {0xAA, 0xBB, 0xCC, 0x02},
  {0xAA, 0xBB, 0xCC, 0x03},
  {0xAA, 0xBB, 0xCC, 0x04},
  {0xAA, 0xBB, 0xCC, 0x05}
};

// 출발 신호용 RFID UID (자신의 로봇 출발 카드)
byte START_UID[4] = {0x11, 0x22, 0x33, 0x44};

// 현재 로봇 위치
Coord currentPos;
int   currentDir = 2;  // 시작 방향: 남(아래) 방향으로 출발 가정

// ============================================================
//  모터 제어 함수
// ============================================================
void motorForward(int speedL, int speedR) {
  digitalWrite(ML_IN1, HIGH); digitalWrite(ML_IN2, LOW);
  digitalWrite(MR_IN1, HIGH); digitalWrite(MR_IN2, LOW);
  analogWrite(ML_EN, speedL);
  analogWrite(MR_EN, speedR);
}

void motorStop() {
  digitalWrite(ML_IN1, LOW); digitalWrite(ML_IN2, LOW);
  digitalWrite(MR_IN1, LOW); digitalWrite(MR_IN2, LOW);
  analogWrite(ML_EN, 0);
  analogWrite(MR_EN, 0);
}

void turnLeft(int ms) {
  // 제자리 좌회전: 좌모터 후진, 우모터 전진
  digitalWrite(ML_IN1, LOW);  digitalWrite(ML_IN2, HIGH);
  digitalWrite(MR_IN1, HIGH); digitalWrite(MR_IN2, LOW);
  analogWrite(ML_EN, SPEED_TURN);
  analogWrite(MR_EN, SPEED_TURN);
  delay(ms);
  motorStop();
}

void turnRight(int ms) {
  // 제자리 우회전
  digitalWrite(ML_IN1, HIGH); digitalWrite(ML_IN2, LOW);
  digitalWrite(MR_IN1, LOW);  digitalWrite(MR_IN2, HIGH);
  analogWrite(ML_EN, SPEED_TURN);
  analogWrite(MR_EN, SPEED_TURN);
  delay(ms);
  motorStop();
}

void turnAround(int ms) {
  // 180도 회전
  turnRight(ms * 2);
}

// ============================================================
//  라인트레이싱 (교차점 1칸 진행)
//  반환값: true = 교차점(격자 중심) 도달
// ============================================================
bool readLine(bool &left, bool &right) {
  left  = (analogRead(IR_LEFT)  < LINE_THRESHOLD);  // true = 라인 위
  right = (analogRead(IR_RIGHT) < LINE_THRESHOLD);
  return (left && right);  // 교차점: 양쪽 모두 라인
}

// 1격자(좌표 한 칸) 전진
void moveOneGrid() {
  bool L, R;
  bool crossDetected = false;
  int  crossCount    = 0;

  // 교차점을 지나칠 때까지 라인트레이싱
  while (true) {
    readLine(L, R);

    if (L && R) {
      // 교차점 진입
      crossCount++;
      if (crossCount >= GRID_PULSE) {
        // 격자 중심 도달 → 잠깐 더 직진 후 정지
        motorForward(SPEED_NORMAL, SPEED_NORMAL);
        delay(80);
        motorStop();
        return;
      }
      motorForward(SPEED_SLOW, SPEED_SLOW);
    } else if (!L && !R) {
      // 라인 없음: 짧게 직진
      motorForward(SPEED_NORMAL, SPEED_NORMAL);
    } else if (L && !R) {
      // 우측 이탈: 좌회전 보정
      motorForward(SPEED_SLOW, SPEED_NORMAL);
    } else {
      // 좌측 이탈: 우회전 보정
      motorForward(SPEED_NORMAL, SPEED_SLOW);
    }
    delay(5);
  }
}

// ============================================================
//  방향 전환 (현재방향 → 목표방향)
//  방향: 0=북, 1=동, 2=남, 3=서
// ============================================================
#define TURN_90_MS  480   // 90도 회전 시간 (ms) - 현장 보정 필요
#define TURN_180_MS 960   // 180도 회전 시간

void faceDirection(int targetDir) {
  int diff = (targetDir - currentDir + 4) % 4;
  switch (diff) {
    case 0: break;
    case 1: turnRight(TURN_90_MS);  break;  // 우회전 90°
    case 2: turnAround(TURN_90_MS); break;  // 180°
    case 3: turnLeft(TURN_90_MS);   break;  // 좌회전 90°
  }
  currentDir = targetDir;
  delay(100);
}

void moveTo(Coord target) {
  // 1. 행(row) 먼저 이동
  while (currentPos.row != target.row) {
    int dir = (target.row > currentPos.row) ? 2 : 0; // 남 or 북
    faceDirection(dir);
    moveOneGrid();
    currentPos.row += (dir == 2) ? 1 : -1;
    delay(50);
  }
  // 2. 열(col) 이동
  while (currentPos.col != target.col) {
    int dir = (target.col > currentPos.col) ? 1 : 3; // 동 or 서
    faceDirection(dir);
    moveOneGrid();
    currentPos.col += (dir == 1) ? 1 : -1;
    delay(50);
  }
}

void lifterUp() {
  lifterServo.write(LIFTER_UP);
  delay(800);  // 리프터 동작 완료 대기
}

void lifterDown() {
  lifterServo.write(LIFTER_DOWN);
  delay(800);
}

bool uidMatch(byte *a, byte *b) {
  for (int i = 0; i < 4; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

// RFID 카드가 태깅될 때까지 대기, 태깅되면 UID 반환
// timeout_ms = 0 이면 무한 대기
bool waitForRFID(byte *outUID, unsigned long timeout_ms = 0) {
  unsigned long start = millis();
  while (true) {
    if (!rfid.PICC_IsNewCardPresent()) {
      if (timeout_ms && (millis() - start > timeout_ms)) return false;
      delay(50);
      continue;
    }
    if (!rfid.PICC_ReadCardSerial()) {
      delay(50);
      continue;
    }
    for (int i = 0; i < 4; i++) outUID[i] = rfid.uid.uidByte[i];
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return true;
  }
}

bool obstacleAhead() {
  // Active-LOW 모듈 가정: LOW = 물체 감지
  bool cL = (digitalRead(OBS_LEFT)   == LOW);
  bool cC = (digitalRead(OBS_CENTER) == LOW);
  bool cR = (digitalRead(OBS_RIGHT)  == LOW);
  return (cL || cC || cR);
}

void setup() {
  Serial.begin(9600);

  // 모터 핀
  pinMode(ML_IN1, OUTPUT); pinMode(ML_IN2, OUTPUT); pinMode(ML_EN, OUTPUT);
  pinMode(MR_IN1, OUTPUT); pinMode(MR_IN2, OUTPUT); pinMode(MR_EN, OUTPUT);
  motorStop();

  // IR 센서 핀
  // 아날로그 핀은 별도 pinMode 불필요
  pinMode(OBS_LEFT,   INPUT);
  pinMode(OBS_CENTER, INPUT);
  pinMode(OBS_RIGHT,  INPUT);

  // Lifter
  lifterServo.attach(SERVO_PIN);
  lifterDown();  // 초기 위치 하강

  // RFID
  SPI.begin();
  rfid.PCD_Init();
  delay(100);

  // 현재 위치 초기화
  currentPos = START_POS;

  Serial.println(F("=== ILRC 물류로봇 초기화 완료 ==="));
  Serial.println(F("출발 RFID 카드를 태깅하세요..."));
}

void loop() {
  // ── 단계 1: 출발 RFID 대기 ──────────────────────────────
  byte uid[4];
  Serial.println(F("[대기] RFID 출발 카드 태깅을 기다리는 중..."));

  while (true) {
    if (waitForRFID(uid, 0)) {
      if (uidMatch(uid, START_UID)) {
        Serial.println(F("[출발] 출발 신호 인식!"));
        break;
      }
    }
  }
  delay(200);

  // ── 단계 2: 미션 수행 (팔렛 5개 운반) ─────────────────
  for (int mission = 0; mission < 5; mission++) {
    Serial.print(F("[미션] "));
    Serial.print(mission + 1);
    Serial.println(F("번째 팔렛 운반 시작"));

    // 2-1. 물류창고로 이동
    Serial.println(F("  → 물류창고 이동 중"));
    moveTo(WAREHOUSE[mission]);
    delay(300);

    // 2-2. Pallet RFID 확인 (선택적: 미확인 시 그냥 상차)
    Serial.println(F("  → Pallet RFID 확인 중"));
    byte palletUid[4];
    bool found = waitForRFID(palletUid, 2000);  // 2초 내 인식
    if (found) {
      Serial.print(F("  Pallet UID: "));
      for (int i = 0; i < 4; i++) {
        Serial.print(palletUid[i], HEX); Serial.print(' ');
      }
      Serial.println();
    } else {
      Serial.println(F("  (Pallet RFID 미인식 - 상차 진행)"));
    }

    // 2-3. Lifter 상승 (팔렛 상차)
    Serial.println(F("  → Lifter UP (상차)"));
    lifterUp();
    delay(300);

    // 2-4. 행선지로 이동
    Serial.println(F("  → 행선지 이동 중"));
    moveTo(DESTINATION[mission]);
    delay(300);

    // 2-5. Lifter 하강 (팔렛 하차)
    Serial.println(F("  → Lifter DOWN (하차)"));
    lifterDown();
    delay(500);

    Serial.print(F("[완료] "));
    Serial.print(mission + 1);
    Serial.println(F("번째 팔렛 배송 완료 (+10점)"));
  }

  // ── 단계 3: 미션 종료 ──────────────────────────────────
  Serial.println(F("=== 모든 미션 완료! ==="));
  motorStop();

  // 다음 루프 방지 (경기 종료)
  while (true) {
    delay(1000);
  }
}

  void waitIfObstacle() {
    while (obstacleAhead()) {
      motorStop();
      delay(200);
    }
  }
