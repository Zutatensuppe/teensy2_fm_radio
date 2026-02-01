#include <Wire.h>

#define BTN_CHANNEL_UP PIN_D2
#define BTN_CHANNEL_DOWN PIN_D3

bool debug_mode = false;

#define DBG(...) \
  do { \
    if (debug_mode) Serial.print(__VA_ARGS__); \
  } while (0)
#define DBGLN(...) \
  do { \
    if (debug_mode) Serial.println(__VA_ARGS__); \
  } while (0)

const byte I2C_ADDR = 0x3E;

const float CHANNEL_STEP = 0.1;

const float CHANNEL_MIN = 70.0;
const float CHANNEL_MAX = 108.0;

float current_channel = 98.6;

enum country_type {
  USA,
  JAPAN,
  EUROPE,
  AUSTRALIA,
  CHINA,
};

// PGA Gain Settings (5-bit values: MSB + LSB)
#define PGA_12DB 0x1F   // Binary 11111 (+12dB Max Volume)
#define PGA_6DB 0x1A    // Binary 11010 (+6dB)
#define PGA_0DB 0x10    // Binary 10000 (0dB Standard)
#define PGA_N6DB 0x06   // Binary 00110 (-6dB)
#define PGA_N15DB 0x0F  // Binary 01111 (-15dB Min Volume)

void setup() {
  pinMode(BTN_CHANNEL_UP, INPUT_PULLUP);
  pinMode(BTN_CHANNEL_DOWN, INPUT_PULLUP);

  Wire.begin();

  // Wait for Serial to initialize so we don't miss messages
  Serial.begin(9600);
  while (!Serial && millis() < 1000)
    ;  // Wait up to 1 second for serial

  DBGLN("\n\n===================================");
  DBGLN("--- KT0803L DEBUG MODE STARTED ---");
  DBGLN("===================================");

  // 1. I2C Scanner to confirm connection
  diag_scan_i2c();

  // 2. Initialize Chip
  DBGLN("\n--- INITIALIZING CONFIGURATION ---");

  DBGLN("-> Setting Frequency...");
  set_chsel(current_channel);

  DBGLN("-> Setting RF Gain...");
  set_rfgain(0x08);

  DBGLN("-> Setting PGA...");
  set_pga(PGA_6DB);

  DBGLN("-> Setting Region...");
  set_phtcnst(EUROPE);

  DBGLN("-> Setting AU Enhance...");
  set_au_enhance(true);

  activate_module();

  DBGLN("\n--- READY ---");
  DBGLN("Commands:");
  DBGLN("  'm' = Toggle Mute");
  DBGLN("  'd' = Dump Registers");
  DBGLN("  'f' = Force Wakeup & Unmute (Panic Button)");
}

void loop() {
  if (digitalRead(BTN_CHANNEL_UP) == LOW) {
    DBGLN(">> Button UP Pressed");
    adjust_channel(+CHANNEL_STEP);
    delay(200);
  }

  if (digitalRead(BTN_CHANNEL_DOWN) == LOW) {
    DBGLN(">> Button DOWN Pressed");
    adjust_channel(-CHANNEL_STEP);
    delay(200);
  }

  // Serial Command Handler
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'm': toggle_mute(); break;
      case 'd': diag_dump_all_registers(); break;
      case 'f': activate_module(); break;
    }
    // Clear buffer
    while (Serial.available()) {
      Serial.read();
    }
  }
}

void activate_module() {
  DBGLN(">>> EXECUTING FORCE ACTIVE SEQUENCE <<<");

  // 1. WAKE UP (Register 0x0B)
  // Bit 7: Standby (0=Active, 1=Sleep)
  // Bit 5: PDPA (0=PA On, 1=PA Off)
  // We write 0x00 to turn everything ON.
  write_to_register(0x0B, 0x00);
  DBGLN("1. Standby Forced OFF (Reg 0x0B set to 0x00)");
  delay(50);

  // 2. DISABLE SILENCE DETECTION (Register 0x12)
  // Bit 7: SLNCDIS (1=Disable Silence detection)
  // If this is 0, the chip auto-mutes when audio is low. We set it to 1.
  uint8_t reg12 = read_from_register(0x12);
  write_to_register(0x12, reg12 | 0x80);
  DBGLN("2. Silence Detection Disabled (Auto-mute OFF)");

  // 3. UNMUTE (Register 0x02)
  set_mute(false)
  DBGLN("3. Software Mute Forced OFF");

  diag_dump_all_registers();
}

bool is_muted() {
  uint8_t register_0x02 = read_from_register(0x02);
  return bool(register_0x02 & 0x08);
}

void toggle_mute() {
  set_mute(!is_muted());
}

void adjust_channel(float adjustment) {
  current_channel += adjustment;
  if (current_channel > CHANNEL_MAX) {
    current_channel = CHANNEL_MAX;
  }
  if (current_channel < CHANNEL_MIN) {
    current_channel = CHANNEL_MIN;
  }
  set_chsel(current_channel);
}

// ----------------------------------------------------------------
//
// --- I2C HELPER FUNCTIONS WITH DEBUGGING ---
//
// ----------------------------------------------------------------

void write_to_register(uint8_t target_register, uint8_t value) {
  DBG("   [I2C WRITE] Reg: 0x");
  if (target_register < 0x10) DBG("0");
  DBG(target_register, HEX);
  DBG(" | Val: 0x");
  if (value < 0x10) DBG("0");
  DBG(value, HEX);
  DBG(" (Bin: ");
  diag_print_binary(value);
  DBG(")");

  // write the value
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(target_register);
  Wire.write(value);
  byte error = Wire.endTransmission();
  // done...

  if (error == 0) {
    DBGLN(" -> OK");
  } else {
    DBG(" -> FAIL! Error Code: ");
    DBGLN(error);
  }
}

uint8_t read_from_register(uint8_t source_register) {
  uint8_t value = 0;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(source_register);
  byte error = Wire.endTransmission(false);

  if (error != 0) {
    DBG("   [I2C READ ERROR] Transmission failed for Reg 0x");
    DBGLN(source_register, HEX);
    return 0;
  }

  Wire.requestFrom((int)I2C_ADDR, 1);

  if (Wire.available()) {
    value = Wire.read();
    DBG("   [I2C READ ] Reg: 0x");
    if (source_register < 0x10) DBG("0");
    DBG(source_register, HEX);
    DBG(" | Val: 0x");
    if (value < 0x10) DBG("0");
    DBG(value, HEX);
    DBG(" (Bin: ");
    diag_print_binary(value);
    DBGLN(")");
  } else {
    DBGLN("   [I2C READ ] No Data Available!");
  }

  return value;
}

// ----------------------------------------------------------------
//
// --- LOGIC FUNCTIONS ---
//
// ----------------------------------------------------------------

/*
This controls the signal strength. 
If on default, some default radio stations cannot be heard anymore,
if that is the case reduce the value!

RFGAIN[3:0] RFOUT
       0000 95.5 dBuV
       0001 96.5 dBuV
       0010 97.5 dBuV
       0011 98.2 dBuV
       0100 98.9 dBuV
       0101 100 dBuV
       0110 101.5 dBuV
       0111 102.8 dBuV
       1000 105.1 dBuV (107.2dBuV PA_BIAS=1)
       1001 105.6 dBuV (108dBuV, PA_BIAS=1)
       1010 106.2 dBuV (108.7dBuV, PA_BIAS=1)
       1011 106.5 dBuV (109.5dBuV, PA_BIAS=1)
       1100 107 dBuV (110.3dBuV, PA_BIAS=1)
       1101 107.4 dBuV (111dBuV, PA_BIAS=1)
       1110 107.7 dBuV (111.7dBuV, PA_BIAS=1)
       1111 (default) 108 dBuV (112.5dBuV, PA_BIAS=1)
*/
void set_rfgain(uint8_t rfgain) {
  DBG("   Setting rfgain to: 0b");
  DBG(rfgain, BIN);
  DBGLN();

  // rfgain value is spread over 3 registers
  uint8_t register_0x01 = read_from_register(0x01);  // Bits 7|6 = RFGAIN[1:0]
  uint8_t register_0x13 = read_from_register(0x13);  // Bit 7 = RFGAIN[2]
  uint8_t register_0x02 = read_from_register(0x02);  // Bit 6 = RFGAIN[3]

  rfgain &= 0x0F;  // keep only last 4 bits of incoming value
  register_0x01 = (register_0x01 & 0x3F) | (rfgain << 6);

  if (rfgain & 0x04) register_0x13 |= 0x80;
  else register_0x13 &= ~0x80;

  if (rfgain & 0x08) register_0x02 |= 0x40;
  else register_0x02 &= ~0x40;

  write_to_register(0x01, register_0x01);
  write_to_register(0x13, register_0x13);
  write_to_register(0x02, register_0x02);
}

/*
CHSEL[11:0] = Dec2Bin (Target frequency in MHz x 20),
where CHSEL[11:0] = register_0x01[2:0]:register_0x00[7:0]:register_0x02[7]
*/
void set_chsel(float f_channel) {
  DBG("   Setting Channel to: ");
  DBG(f_channel);
  DBGLN(" MHz");

  uint16_t chsel;
  uint8_t register_0x02, register_0x00, register_0x01;

  register_0x02 = read_from_register(0x02);
  register_0x01 = read_from_register(0x01);

  // Convert float to uint16_t according to required calculation
  chsel = (uint16_t)(f_channel * 20);
  chsel &= 0x0FFF;

  if (chsel & 0x01) register_0x02 |= 0x80;
  else register_0x02 &= ~0x80;

  register_0x00 = (uint8_t)(chsel >> 1);
  register_0x01 = (register_0x01 & 0xF8) | (uint8_t)(chsel >> 9);

  write_to_register(0x02, register_0x02);
  write_to_register(0x01, register_0x01);
  write_to_register(0x00, register_0x00);
}

/*
Pre-emphasis Time-Constant Set
0: 75 μs (USA, Japan, South Korea)
1: 50 μs (Europe, Australia, China, most everywhere else)
*/
void set_phtcnst(country_type country) {
  uint8_t register_0x02;

  register_0x02 = read_from_register(0x02);

  switch (country) {
    case USA:
    case JAPAN:
      register_0x02 &= ~0x01;
      break;
    case EUROPE:
    case AUSTRALIA:
    case CHINA:
      register_0x02 |= 0x01;
      break;
  }

  write_to_register(0x02, register_0x02);
}

/*
PGA[2:0] PGA_LSB[1:0] PGA Gain
     111           11 12dB
     111           10 11
     111           01 10
     111           00 9
     110           11 8
     110           10 7
     110           01 6
     110           00 5
     101           11 4
     101           10 3
     101           01 2
     101           00 1
     100           11 0
     100           10 0
     100           01 0
     100           00 0
     000           00 0
     000           01 -1
     000           10 -2
     000           11 -3
     001           00 -4
     001           01 -5
     001           10 -6
     001           11 -7
     010           00 -8
     010           01 -9
     010           10 -10
     010           11 -11
     011           00 -12
     011           01 -13
     011           10 -14
     011           11 -15
*/
void set_pga(uint8_t pga) {
  // --- Handle MSB (Register 0x01, Bits 5:3) ---
  uint8_t register_0x01 = read_from_register(0x01);

  // Take top 3 bits of input (e.g., 100xx) -> shift down to 00000100
  uint8_t msb = (pga >> 2) & 0x07;

  // Clear bits 5,4,3 (0xC7 is 11000111) and OR in new value shifted up
  register_0x01 = (register_0x01 & 0xC7) | (msb << 3);
  write_to_register(0x01, register_0x01);

  // --- Handle LSB (Register 0x04, Bits 5:4) ---
  uint8_t register_0x04 = read_from_register(0x04);

  // Take bottom 2 bits of input (e.g., xxx00)
  uint8_t lsb = pga & 0x03;

  // Clear bits 5,4 (0xCF is 11001111) and OR in new value shifted up
  register_0x04 = (register_0x04 & 0xCF) | (lsb << 4);
  write_to_register(0x04, register_0x04);
}

/*
Audio Frequency Response Enhancement Enable
0 = Disable
1 = Enable
*/
void set_au_enhance(bool au_enhance) {
  uint8_t register_0x17 = read_from_register(0x17);

  if (au_enhance) register_0x17 |= 0x20;
  else register_0x17 &= ~0x20;

  write_to_register(0x17, register_0x17);
}

/*
Software Mute
0: MUTE Disabled
1: MUTE Enabled
*/
void set_mute(bool mute) {
  uint8_t register_0x02 = read_from_register(0x02);

  if (mute) register_0x02 |= 0x08;
  else register_0x02 &= ~0x08;

  write_to_register(0x02, register_0x02);
}

// ----------------------------------------------------------------
//
// --- DIAGNOSTIC TOOLS - only useful in debug mode! ---
//
// ----------------------------------------------------------------

void diag_scan_i2c() {
  DBGLN("Scanning I2C bus...");
  byte error, address;
  uint8_t devices_count = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      DBG("I2C device found at address 0x");
      if (address < 16) DBG("0");
      DBG(address, HEX);
      DBGLN("  !");
      devices_count++;
    }
  }

  if (devices_count == 0) DBGLN("No I2C devices found\n");
  else DBGLN("Done\n");
}

void diag_dump_all_registers() {
  DBGLN("--- DUMPING KT0803L REGISTERS ---");
  // Important registers based on datasheet
  uint8_t registers[] = { 0x00, 0x01, 0x02, 0x04, 0x0B, 0x0E, 0x13, 0x17, 0x1E };

  for (unsigned int i = 0; i < sizeof(registers); i++) {
    uint8_t val = read_from_register(registers[i]);
    DBG("Reg 0x");
    if (registers[i] < 0x10) DBG("0");
    DBG(registers[i], HEX);
    DBG(": 0x");
    if (val < 0x10) DBG("0");
    DBG(val, HEX);
    DBG(" (");
    diag_print_binary(val);
    DBG(")");

    // Decode specific important bits
    if (registers[i] == 0x02) {
      if (val & 0x08) DBG(" [Mute is ON]");
      else DBG(" [Mute is OFF]");
    }
    if (registers[i] == 0x0B) {
      if (val & 0x80) DBG(" [Standby is ON]");
      else DBG(" [Standby is OFF]");
    }
    DBGLN();
  }
  DBGLN("---------------------------------");
}

void diag_print_binary(byte value) {
  for (int b = 7; b >= 0; b--) {
    DBG((value >> b) & 1);
  }
}
