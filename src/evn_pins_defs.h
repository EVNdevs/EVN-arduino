#pragma once

#if (defined(ARDUINO_GENERIC_RP2040))

// Button
#define PIN_BUTTON          (24u)

// Servo Ports
#define PIN_SERVO1          (2u)
#define PIN_SERVO2          (3u)
#define PIN_SERVO3          (10u)
#define PIN_SERVO4          (11u)

// Motor Ports
#define PIN_MOTOR1_OUTA     (29u)
#define PIN_MOTOR1_OUTB     (28u)
#define PIN_MOTOR2_OUTA     (27u)
#define PIN_MOTOR2_OUTB     (26u)
#define PIN_MOTOR3_OUTA     (23u)
#define PIN_MOTOR3_OUTB     (22u)
#define PIN_MOTOR4_OUTA     (21u)
#define PIN_MOTOR4_OUTB     (20u)

#define PIN_MOTOR1_ENCA     (18u)
#define PIN_MOTOR1_ENCB     (19u)
#define PIN_MOTOR2_ENCA     (17u)
#define PIN_MOTOR2_ENCB     (16u)
#define PIN_MOTOR3_ENCA     (14u)
#define PIN_MOTOR3_ENCB     (15u)
#define PIN_MOTOR4_ENCA     (13u)
#define PIN_MOTOR4_ENCB     (12u)

// SPI
#undef PIN_SPI0_MISO
#define PIN_SPI0_MISO       (0u)
#undef PIN_SPI0_MOSI
#define PIN_SPI0_MOSI       (3u)
#undef PIN_SPI0_SCK
#define PIN_SPI0_SCK        (2u)
#undef PIN_SPI0_SS
#define PIN_SPI0_SS         (1u)

#undef PIN_SPI1_MISO
#define PIN_SPI1_MISO       (8u)
#undef PIN_SPI1_MOSI
#define PIN_SPI1_MOSI       (11u)
#undef PIN_SPI1_SCK
#define PIN_SPI1_SCK        (10u)
#undef PIN_SPI1_SS
#define PIN_SPI1_SS         (9u)

// Wire
#undef PIN_WIRE1_SDA
#define PIN_WIRE1_SDA       (6u)

#undef PIN_WIRE1_SCL
#define PIN_WIRE1_SCL       (7u)

#endif