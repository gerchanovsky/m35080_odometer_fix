/*
The M35080 device consists of 1024x8 bits of low power EEPROM.
The device is accessed by a simple SPI-compatible serial interface.

The bus signals consist of a serial clock input (C), a serial data
input (D) and aserial data output (Q)

The device is selected when the chip select input (S) is held low.

Data is clocked in during the low to high transition of the clock, C.
Data is clocked out during the high to low transition of the clock.
*/
#include <stdint.h>
#include <SPI.h>
static const SPISettings spiSettings(3000000, MSBFIRST, SPI_MODE0);

#define NEW_VIN       "KP83884" // Put here reqired lst 7 characters of your VIN (must be 2 letters+5 digits)
#define NEW_MILAGE_KM xxxx // Put here required mileage in KILOMETERS
static uint16_t vin_offs = 0x7A;

#define WE      9        // Write Enable (~W)  Orange
#define CS	10//SS   // Chip Select (S)    White
#define DATAOUT 11//MOSI // Serial Input (D)   Green
#define DATAIN  12//MISO // Serial Output (Q)  Blue
#define SCLK    13//SCK  // Serial Clock (C)   Yellow

//Instruction Set
#define WREN  B00000110  // Set Write Enable Latch
#define WRDI  B00000100  // Reset Write Enable Latch
#define RDSR  B00000101  // Read Status Register
#define WRSR  B00000001  // Write Status Register
#define READ  B00000011  // Read Data from Memory Array
#define WRITE B00000010  // Write Data to Memory Array
#define WRINC B00000111  // Write Data to Secure Array

#define STATUS_SRWD (1<<7)
#define STATUS_UV   (1<<6)
#define STATUS_X    (1<<5)
#define STATUS_INC  (1<<4)
#define STATUS_BP1  (1<<3)
#define STATUS_BP0  (1<<2)
#define STATUS_WEL  (1<<1)
#define STATUS_WIP  (1<<0)

static uint8_t eeprom[0x400];
#define CS_ENABLE     digitalWrite(CS, LOW)
#define CS_DISABLE    digitalWrite(CS, HIGH)
#define WRITE_ENABLE  digitalWrite(WE, LOW)
#define WRITE_DISABLE digitalWrite(WE, HIGH)

static void hex_8(byte b, const char *suffix = " ")
{
  if (b<0x10)
    Serial.print('0');
  Serial.print(b, HEX);
  if (suffix && *suffix!=0)
    Serial.print(suffix);
}

static void hex_address(uint16_t addr, const char *suffix = ": ")
{
  Serial.println();
  if (addr==0x000)
    Serial.println("INCREMENTAL REGISTERS:");
  else if (addr==0x020)
    Serial.println("NOT INCREMENTAL REGISTERS:");
  if (addr<0x100) {
    Serial.print('0');
    if (addr<0x10)
      Serial.print('0');
  }
  Serial.print(addr, HEX);
  Serial.print(suffix);
}
/*****************************************************/
static uint8_t read_status() {
  SPI.beginTransaction(spiSettings);
  CS_ENABLE;
  SPI.transfer(RDSR);
  uint8_t value = SPI.transfer(0x00);
  CS_DISABLE;
  SPI.endTransaction();
  return value;
}
static void print_status(uint8_t status) {
  Serial.print("Status:");
  hex_8(status, "\n");
  if (status & STATUS_SRWD) Serial.println("SRWD (Write Disable bit)");
  if (status & STATUS_UV  ) Serial.println("UV (chip has been erased)");
  if (status & STATUS_X   ) Serial.println("x ");
  if (status & STATUS_INC ) Serial.println("INC (incremental write failed)");
  if (status & STATUS_BP1 ) Serial.println("BP1 (Block Protect 1)");
  if (status & STATUS_BP0 ) Serial.println("BP0 (Block Protect 0)");
  if (status & STATUS_WEL ) Serial.println("WEL (Write Enable Latch)");
  if (status & STATUS_WIP ) Serial.println("WIP (Write-In-Progress)");
}
/************************************************/
static uint8_t read_8(int address) {
  SPI.beginTransaction(spiSettings);
  CS_ENABLE;
  SPI.transfer(READ);
  SPI.transfer16(address);
  uint8_t value = SPI.transfer(0x00);
  CS_DISABLE;
  SPI.endTransaction();
  return value;
}
static void read_buf(int address, uint8_t *buffer, uint16_t size) {
  SPI.beginTransaction(spiSettings);
  CS_ENABLE;
  SPI.transfer(READ);
  SPI.transfer16(address);
  while (size-->0)
    *buffer++ = SPI.transfer(0x00);
  CS_DISABLE;
  SPI.endTransaction();
}
/************************************************/
static bool write_enable() {
//return false;
  WRITE_ENABLE;
  SPI.beginTransaction(spiSettings);
  CS_ENABLE;
  SPI.transfer(WREN);
  CS_DISABLE;
  SPI.endTransaction();
  delay(10);
  if ((read_status() & STATUS_WEL)==0) {
    Serial.println("Write disabled");
    WRITE_DISABLE;
    return false;
  }
  SPI.beginTransaction(spiSettings);
  CS_ENABLE;
  return true;
}
static bool write_disable(bool inc = false) {
  bool res = true;
  CS_DISABLE;
  SPI.endTransaction();
  uint8_t status, counter = 0;
  delay(10);
  do {
    if (counter++==1)
      Serial.println("Wait for WIP");
    status = read_status();
  } while (status  & STATUS_WIP);
  if (inc && (status & STATUS_INC)) {
    Serial.println("Incremental write failed");
    print_status(status);
    res = false;
  }
  SPI.beginTransaction(spiSettings);
  CS_ENABLE;
  SPI.transfer(WRDI);
  CS_DISABLE;
  SPI.endTransaction();
  WRITE_DISABLE;
  return res;
}
static bool write_8(uint16_t address, uint8_t data) {
  if (!write_enable()) return false;
  SPI.transfer(WRITE);
  SPI.transfer16(address);
  SPI.transfer(data);
  return write_disable();
}
static bool write_buf(uint16_t address, const uint8_t *buffer, uint16_t size) {
  if (!write_enable()) return false;
  SPI.transfer(WRITE);
  SPI.transfer16(address);
  while (size-->0)
    SPI.transfer(*buffer++);
  return write_disable();
}
static bool write_secure(uint16_t address, uint16_t data) {
  if (!write_enable()) return false;
  SPI.transfer(WRINC);
  SPI.transfer16(address);
  SPI.transfer((uint8_t)(data>>8));
  SPI.transfer((uint8_t)(data&0xFF));
  return write_disable(true);
}
/************************************************/
static bool is_vin(const uint8_t *ptr) {
  return (ptr[0]>='A' && ptr[0]<='Z') &&
         (ptr[1]>='A' && ptr[1]<='Z') &&
         ((ptr[2]>>4)<10) &&
         ((ptr[2]&0xf)<10) &&
         ((ptr[3]>>4)<10) &&
         ((ptr[3]&0xf)<10) &&
         ((ptr[4]>>4)<10) &&
         ((ptr[4]&0xf)==0);
}
static bool find_vin() {
  int offs = -1;
  for (uint16_t i=0;i<(sizeof(eeprom)-5);i++) {
    if (!is_vin(eeprom+i)) continue;
    if (offs<0) offs = i;else {
      Serial.println("More then one potential VIN offsets found");
      hex_address(offs, ",");
      hex_address(i, "\n");
      return false;
    }
  }
  if (offs<0) {
    Serial.println("VIN offset not found");
    return false;
  }
  vin_offs = offs;
  Serial.print("VIN offset:");
  hex_address(offs, "\n");
  return true;
}
static void print_vin()
{
  //Next you want to find your VIN. The problem with that is there is no definite location its stored.
  //So if you remember from earlier the VIN of the replacement cluster was “AW72288” what you want to
  //look for is “AW” on the right side pane of the hex editor. Once you find a possible match look for
  //the rest of the VIN. In mine 41 is ASCII for “A” and 57 is ASCII for “W” the rest is “72 28”.
  //The next two numbers are some check digit. You want to leave that as it is. The last is “8” shown in the pic below.
  Serial.print("VIN:");
  const uint8_t *ptr = eeprom+vin_offs;
  Serial.print((char)*ptr++);
  Serial.print((char)*ptr++);
  hex_8(*ptr++,NULL);
  hex_8(*ptr++,NULL);
  Serial.println(((*ptr++)>>4),HEX);
}
static bool write_vin(const char *vin_new)
{
  uint8_t vin[5];// = {'F','R',0x05,0x67,0x60};
  vin[0] = vin_new[0];
  vin[1] = vin_new[1];
  vin[2] = ((vin_new[2]-'0')<<4)|(vin_new[3]-'0');
  vin[3] = ((vin_new[4]-'0')<<4)|(vin_new[5]-'0');
  vin[4] = ((vin_new[6]-'0')<<4);
  return write_buf(vin_offs,vin,sizeof(vin));
}
static void print_odometer() {
  // Quick and dirty odometer parsing
  struct {
    uint16_t v;
    uint8_t  n;
  } vals[0x10];
  uint8_t i, c = 0xFF;
  for (i=0;i<0x10;i++) {
    uint16_t val = ((uint16_t)eeprom[(i<<1)])<<8 | eeprom[(i<<1)+1];
    if (c==0xff || val!=vals[c].v) {
      c++;
      vals[c].v = val;
      vals[c].n = 1;
    } else {
      vals[c].n++;
    }
  }
  Serial.print("Odometer:");
  if (c==0) {
    Serial.print(((uint32_t)vals[0].v)*16);
    Serial.println(" km");
  } else if (c==1) {
    Serial.print(((uint32_t)vals[1].v)*16+vals[0].n);
    Serial.println(" km");
  } else {
    Serial.println("error");
    for (i=0;i<=c;i++) {
      Serial.print(((uint32_t)vals[i].v)*16);
      Serial.print(' ');
      Serial.println(vals[i].n);
    }
  }
}
static bool write_odometer(uint32_t odometer)
{
  uint16_t v = odometer>>4;
  uint16_t next = odometer&0xF;
  struct {
    uint8_t hi;
    uint8_t lo;
    inline uint16_t get() { return ((uint16_t)hi)<<8|lo;};
  } secure[0x10];
  read_buf(0x000, (uint8_t*)&secure, sizeof(secure));
  Serial.print("New odometer:");
  Serial.print(odometer);
  Serial.println(" km");
  for (uint8_t i=0;i<0x10;i++) {
    uint16_t tmp, old = secure[i].get();
    if (next>0) {
      tmp = v+1;
      next--;
    } else {
      tmp = v;
    }
    if (old>tmp) {
      Serial.print("Can't set odometer (new mileage is lower then previous):");
      Serial.print(tmp,HEX);
      Serial.print('<');
      Serial.println(old,HEX);
      return false;
    } else if (old<tmp) {
      Serial.print("Write@");
      Serial.print(i<<1,HEX);
      Serial.print(":");
      Serial.println(tmp, HEX);
      if (!write_secure(i<<1,tmp))
        return false;
    }
  }
  return true;
}

/************************************************************/
void setup() {
  pinMode(CS, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  digitalWrite(CS, HIGH);
  digitalWrite(SCLK, HIGH);
  pinMode(WE, OUTPUT);
  WRITE_DISABLE;

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  Serial.begin(115200);
  Serial.println();
  Serial.flush();
  Serial.println("Done setting up...");

// Uncomment to write a new odometer
// Please double check new value - you can only write greater odometer value
//  write_odometer(NEW_MILAGE_KM);
  print_status(read_status());

  read_buf(0x000, eeprom, sizeof(eeprom));
  if (find_vin()) {
    write_vin(NEW_VIN);
    read_buf(0x000, eeprom, sizeof(eeprom));
  }

  print_vin();
  print_odometer();
  for (uint16_t index = 0x00; index <= 0x3FF; index++) {
    if (index % 16 == 0)
      hex_address(index);
    hex_8(eeprom[index]);
  }
  Serial.println();
  Serial.println("END");
}

void loop() {
}
