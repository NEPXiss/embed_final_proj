// === UART bridge: ESP32-S3 <-> STM32 ===

// Choose pins for UART1 (change if you want)
#define STM32_RX_PIN 18   // ESP32-S3 receives from STM32 TX
#define STM32_TX_PIN 17   // ESP32-S3 transmits to STM32 RX

#define UART_BAUD    115200 // UART speed for STM32 link
#define USB_BAUD     115200 // Serial Monitor speed

void setup() {
  // USB serial for debugging / Serial Monitor
  Serial.begin(USB_BAUD);
  delay(2000); // give time for USB to come up

  Serial.println();
  Serial.println("=== ESP32-S3 UART Bridge Started ===");
  Serial.println("USB Serial <-> UART1 (STM32)");
  Serial.println("Type in Serial Monitor to send to STM32.");
  Serial.println();

  // Hardware UART1 for STM32
  // Serial1.begin(baud, config, RX_pin, TX_pin)
  Serial1.begin(UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
}

void loop() {
  // 1) From PC -> ESP32 -> STM32
  if (Serial.available()) {
    int c = Serial.read();
    Serial1.write(c);  // forward to STM32
  }

  // 2) From STM32 -> ESP32 -> PC
  if (Serial1.available()) {
    int c = Serial1.read();
    Serial.write(c);   // show on Serial Monitor
  }
}