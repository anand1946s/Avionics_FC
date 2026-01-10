#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "data.h"

// ---------------------- LoRa Pins ----------------------
#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

// ------------------- LoRa Registers -------------------
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FR_MSB            0x06
#define REG_FR_MID            0x07
#define REG_FR_LSB            0x08
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_ADDR_PTR     0x0D
#define REG_IRQ_FLAGS         0x12
#define REG_RX_NB_BYTES       0x13
#define REG_PKT_RSSI_VALUE    0x1A
#define REG_VERSION           0x42

#define MODE_LONG_RANGE_MODE  0x80
#define MODE_SLEEP            0x00
#define MODE_STDBY            0x01
#define MODE_TX               0x03
#define MODE_RX_CONTINUOUS    0x05

// ------------------- Globals --------------------------
String cmd = "";
uint8_t txBuffer[256];
uint8_t rxBuffer[256];

// ------------------- SPI Helpers ----------------------
void writeReg(uint8_t addr, uint8_t val) {
    digitalWrite(LORA_SS, LOW);
    SPI.transfer(addr | 0x80); // MSB=1 for write
    SPI.transfer(val);
    digitalWrite(LORA_SS, HIGH);
}

uint8_t readReg(uint8_t addr) {
    digitalWrite(LORA_SS, LOW);
    SPI.transfer(addr & 0x7F); // MSB=0 for read
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(LORA_SS, HIGH);
    return val;
}

// ------------------- LoRa Core Functions -----------------
bool initLoRa() {
    pinMode(LORA_SS, OUTPUT);
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    SPI.begin();

    // Reset and switch to LoRa mode
    writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    delay(10);

    // Check version
    if (readReg(REG_VERSION) != 0x12) return false; // SX1278 version

    // Set frequency (e.g., 433 MHz)
    long frf = ((long)LORA_FREQUENCY << 19) / 32000000;
    writeReg(REG_FR_MSB, (uint8_t)(frf >> 16));
    writeReg(REG_FR_MID, (uint8_t)(frf >> 8));
    writeReg(REG_FR_LSB, (uint8_t)(frf >> 0));

    // Set FIFO base addresses
    writeReg(REG_FIFO_TX_BASE_ADDR, 0);
    writeReg(REG_FIFO_RX_BASE_ADDR, 0);

    // Standby mode
    writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    return true;
}

void sendCommand(const String &msg) {
    size_t len = msg.length();
    writeReg(REG_FIFO_ADDR_PTR, 0);
    for (size_t i = 0; i < len; i++) {
        writeReg(REG_FIFO, msg[i]);
    }
    writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // Wait for TX done
    while ((readReg(REG_IRQ_FLAGS) & 0x08) == 0) {}
    writeReg(REG_IRQ_FLAGS, 0x08); // Clear IRQ
}

String receiveData() {
    // Check RX continuous mode
    uint8_t irq = readReg(REG_IRQ_FLAGS);
    if (irq & 0x40) { // RxDone
        uint8_t len = readReg(REG_RX_NB_BYTES);
        writeReg(REG_FIFO_ADDR_PTR, 0);
        cmd = "";
        for (uint8_t i = 0; i < len; i++) {
            cmd += (char)readReg(REG_FIFO);
        }
        writeReg(REG_IRQ_FLAGS, 0xFF); // Clear all IRQs
        return cmd;
    }
    return "";
}

String checkCmd() {
    return receiveData();
}

bool sendData(const SensorData &data) {
    String payload = String(data.time) + "," +
                     String(data.ax, 2) + "," + String(data.ay, 2) + "," + String(data.az, 2) + "," +
                     String(data.gx, 2) + "," + String(data.gy, 2) + "," + String(data.gz, 2) + "," +
                     String(data.alti, 2) + "," +
                     String(data.temp, 2) + "," + String(data.pre, 2);

    sendCommand(payload);
    return true;
}
