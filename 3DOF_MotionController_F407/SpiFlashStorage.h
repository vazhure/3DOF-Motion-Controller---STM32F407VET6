// SpiFlashStorage.h - Software SPI version for STM32F407VET6
// No hardware SPI conflict with Timer2/Timer3
#pragma once

#include <Arduino.h>

// ============================================================================
// PIN CONFIGURATION - Using board header definitions
// From generic_f407v.h:
//   FLASH_CS_PIN   = PB0
//   FLASH_CLK_PIN  = PB3 (SPI3_SCK)
//   FLASH_DO_PIN   = PB4 (SPI3_MISO)
//   FLASH_DI_PIN   = PB5 (SPI3_MOSI)
// ============================================================================

// ============================================================================
// WINBOND W25Q16JV COMMANDS
// ============================================================================
#define CMD_WRITE_ENABLE 0x06
#define CMD_WRITE_DISABLE 0x04
#define CMD_READ_STATUS_REG1 0x05
#define CMD_READ_DATA 0x03
#define CMD_PAGE_PROGRAM 0x02
#define CMD_SECTOR_ERASE_4KB 0x20
#define CMD_READ_JEDEC_ID 0x9F
#define CMD_RELEASE_POWER_DOWN 0xAB

// ============================================================================
// MEMORY ORGANIZATION
// ============================================================================
#define FLASH_PAGE_SIZE 256UL
#define FLASH_SECTOR_SIZE_4KB 4096UL
#define FLASH_TOTAL_SIZE (2UL * 1024UL * 1024UL)

#define FLASH_CONFIG_SECTOR ((FLASH_TOTAL_SIZE / FLASH_SECTOR_SIZE_4KB) - 1)
#define FLASH_CONFIG_ADDR (FLASH_CONFIG_SECTOR * FLASH_SECTOR_SIZE_4KB)

#define FLASH_MAGIC_NUMBER 0x3D0FF407UL
#define FLASH_VERSION 2UL
#define FLASH_TIMEOUT_MS 10000UL
#define FLASH_RETRY_COUNT 5

// ============================================================================
// DATA STRUCTURE
// ============================================================================
struct FlashConfig {
  uint32_t magic;
  uint32_t version;
  float pidKp;
  float pidKi;
  float pidKd;
  float pidKs;
  uint16_t pidBlend;
  uint16_t pidFlags;
  uint16_t defaultSpeed;
  uint16_t slowSpeed;
  uint32_t acceleration;
  int32_t savedPos[4];
  uint32_t crc32;
};

// ============================================================================
// SpiFlashStorage CLASS - Software SPI
// ============================================================================
class SpiFlashStorage {
private:
  static bool _initialized;
  static uint8_t _csPin;
  static uint8_t _clkPin;
  static uint8_t _doPin;
  static uint8_t _diPin;

  // Software SPI transfer (bit-banging)
  static uint8_t spiTransfer(uint8_t data) {
    uint8_t rxData = 0;

    for (int i = 7; i >= 0; i--) {
      // MOSI - set data bit
      if (data & (1 << i)) {
        digitalWrite(_diPin, HIGH);
      } else {
        digitalWrite(_diPin, LOW);
      }

      // SCK rising edge
      digitalWrite(_clkPin, HIGH);
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");

      // MISO - read data bit
      if (digitalRead(_doPin)) {
        rxData |= (1 << i);
      }

      // SCK falling edge
      digitalWrite(_clkPin, LOW);
      asm volatile("nop");
      asm volatile("nop");
    }

    return rxData;
  }

  static inline void select() {
    digitalWrite(_csPin, LOW);
    delayMicroseconds(1);
  }

  static inline void deselect() {
    delayMicroseconds(1);
    digitalWrite(_csPin, HIGH);
  }

  static inline void sendAddress24(uint32_t address) {
    spiTransfer((address >> 16) & 0xFF);
    spiTransfer((address >> 8) & 0xFF);
    spiTransfer(address & 0xFF);
  }

  static bool waitForReady(uint32_t timeout = FLASH_TIMEOUT_MS) {
    uint32_t start = millis();
    select();
    spiTransfer(CMD_READ_STATUS_REG1);
    while (millis() - start < timeout) {
      if ((spiTransfer(0xFF) & 0x01) == 0) {
        deselect();
        return true;
      }
      yield();
    }
    deselect();
    return false;
  }

  static bool writeEnable() {
    select();
    spiTransfer(CMD_WRITE_ENABLE);
    deselect();

    select();
    spiTransfer(CMD_READ_STATUS_REG1);
    uint8_t status = spiTransfer(0xFF);
    deselect();
    return (status & 0x02) != 0;
  }

  static uint32_t readJedecId() {
    select();
    spiTransfer(CMD_READ_JEDEC_ID);
    uint32_t id = (uint32_t)spiTransfer(0xFF) << 16;
    id |= (uint32_t)spiTransfer(0xFF) << 8;
    id |= spiTransfer(0xFF);
    deselect();
    return id;
  }

public:
  static uint32_t calculateCRC32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; j++) {
        crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
      }
    }
    return ~crc;
  }

  static bool begin(uint8_t csPin = FLASH_CS_PIN) {
    if (_initialized) return true;

    _csPin = csPin;
    _clkPin = FLASH_CLK_PIN;  // PB3
    _doPin = FLASH_DO_PIN;    // PB4
    _diPin = FLASH_DI_PIN;    // PB5

    // Configure pins
    pinMode(_csPin, OUTPUT);
    pinMode(_clkPin, OUTPUT);
    pinMode(_diPin, OUTPUT);
    pinMode(_doPin, INPUT);

    digitalWrite(_csPin, HIGH);
    digitalWrite(_clkPin, LOW);
    digitalWrite(_diPin, LOW);

    delay(5);

    // Wake up from power-down
    select();
    spiTransfer(CMD_RELEASE_POWER_DOWN);
    deselect();
    delayMicroseconds(50);

    // Read JEDEC ID
    uint32_t id = readJedecId();

    if (id != 0xEF4015) {
#ifdef DEBUG
      Serial.print("Flash error: ID 0x");
      Serial.println(id, HEX);
#endif
      return false;
    }

#ifdef DEBUG
    Serial.print("SPI Flash (soft): ID 0x");
    Serial.print(id, HEX);
    Serial.print(", Size ~");
    Serial.print(FLASH_TOTAL_SIZE / 1024);
    Serial.println(" KB");
#endif

    _initialized = true;
    return true;
  }

  static bool readBytes(uint32_t address, uint8_t* buffer, size_t length) {
    if (!_initialized) return false;
    if (!waitForReady()) return false;

    select();
    spiTransfer(CMD_READ_DATA);
    sendAddress24(address);
    for (size_t i = 0; i < length; i++) {
      buffer[i] = spiTransfer(0xFF);
    }
    deselect();
    return true;
  }

  static bool writeBytes(uint32_t address, const uint8_t* buffer, size_t length) {
    if (!_initialized || length == 0 || length > FLASH_PAGE_SIZE) return false;
    if (address + length > FLASH_TOTAL_SIZE) return false;

    uint32_t pageEnd = (address & ~(FLASH_PAGE_SIZE - 1)) + FLASH_PAGE_SIZE;
    if (address + length > pageEnd) return false;

    if (!waitForReady() || !writeEnable()) return false;

    select();
    spiTransfer(CMD_PAGE_PROGRAM);
    sendAddress24(address);
    for (size_t i = 0; i < length; i++) {
      spiTransfer(buffer[i]);
    }
    deselect();

    return waitForReady();
  }

  static bool eraseSector(uint32_t sectorAddress) {
    if (!_initialized || sectorAddress % FLASH_SECTOR_SIZE_4KB != 0) return false;
    if (!waitForReady() || !writeEnable()) return false;

    select();
    spiTransfer(CMD_SECTOR_ERASE_4KB);
    sendAddress24(sectorAddress);
    deselect();

    return waitForReady(FLASH_TIMEOUT_MS * 3);
  }

  static bool saveConfig(const FlashConfig& config) {
    FlashConfig cfg = config;
    cfg.version = FLASH_VERSION;
    cfg.crc32 = 0;
    cfg.crc32 = calculateCRC32((const uint8_t*)&cfg, sizeof(FlashConfig) - 4);

    FlashConfig stored;
    if (readBytes(FLASH_CONFIG_ADDR, (uint8_t*)&stored, sizeof(FlashConfig)) && stored.magic == FLASH_MAGIC_NUMBER && memcmp(&cfg, &stored, sizeof(FlashConfig) - 4) == 0) {
      return true;
    }

    if (!eraseSector(FLASH_CONFIG_ADDR)) return false;

    for (int retry = 0; retry < FLASH_RETRY_COUNT; retry++) {
      if (writeBytes(FLASH_CONFIG_ADDR, (const uint8_t*)&cfg, sizeof(FlashConfig))) {
        FlashConfig verify;
        if (readBytes(FLASH_CONFIG_ADDR, (uint8_t*)&verify, sizeof(FlashConfig)) && memcmp(&cfg, &verify, sizeof(FlashConfig)) == 0) {
          return true;
        }
      }
      delay(20);
    }
    return false;
  }

  static bool loadConfig(FlashConfig& config) {
    if (!readBytes(FLASH_CONFIG_ADDR, (uint8_t*)&config, sizeof(FlashConfig))) return false;
    if (config.magic != FLASH_MAGIC_NUMBER) return false;

    if (config.version == 1) {
      config.pidFlags = 0;
    } else if (config.version > FLASH_VERSION) {
      return false;
    }

    uint32_t storedCrc = config.crc32;
    config.crc32 = 0;
    uint32_t calcCrc = calculateCRC32((const uint8_t*)&config, sizeof(FlashConfig) - 4);

    if (storedCrc != calcCrc) return false;
    return true;
  }

  static bool resetToDefaults(FlashConfig& config) {
    memset(&config, 0, sizeof(FlashConfig));
    config.magic = FLASH_MAGIC_NUMBER;
    config.version = FLASH_VERSION;
    config.pidKp = 1.5f;
    config.pidKi = 0.0f;
    config.pidKd = 0.02f;
    config.pidKs = 0.50f;
    config.pidBlend = 100;
    config.pidFlags = 0;
    config.defaultSpeed = 90;
    config.slowSpeed = 10;
    config.acceleration = 5000;
    return saveConfig(config);
  }

  static bool getChipInfo(uint32_t& jedecId, uint32_t& capacity) {
    if (!_initialized) return false;
    jedecId = readJedecId();
    uint8_t capByte = jedecId & 0xFF;
    capacity = (1UL << (capByte - 10)) * 1024UL;
    return true;
  }

  static void end() {
    _initialized = false;
  }

  static bool isInitialized() {
    return _initialized;
  }
  static uint8_t getCSPin() {
    return _csPin;
  }
};

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================
bool SpiFlashStorage::_initialized = false;
uint8_t SpiFlashStorage::_csPin = FLASH_CS_PIN;
uint8_t SpiFlashStorage::_clkPin = FLASH_CLK_PIN;
uint8_t SpiFlashStorage::_doPin = FLASH_DO_PIN;
uint8_t SpiFlashStorage::_diPin = FLASH_DI_PIN;