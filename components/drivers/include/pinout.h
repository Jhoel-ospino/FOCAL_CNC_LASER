/**
 * @file pinout.h
 * @brief Asignación definitiva de pines para Cortadora/Grabadora Láser CNC 200x160 mm
 *        en ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM Octal).
 * 
 * Filosofía de Asignación:
 * - Aislamiento total de pines del bus Octal PSRAM/Flash (GPIO 33..37 reservados).
 * - Preservación de la configuración probada de pantalla TFT ILI9488 y MicroSD SPI (Bus SPI2).
 * - Drivers Trinamic TMC2209 en modo autónomo con habilitación común en GPIO 3.
 * - Eje Y sincronizado en paralelo Dual-Y (Y1 en GPIO 4/5, Y2 en GPIO 6/7).
 * - Eje X en GPIO 1/2.
 * - Módulo Láser Tree Laser 20W con control PWM en GPIO 18 (LEDC 5 kHz).
 * - Buzzer pasivo de avisos en GPIO 38.
 * - GPIO 15 reservado / desconectado por ahora.
 * - Finales de carrera en GPIO 16 (X) y GPIO 17 (Y) desacoplados para pruebas seguras.
 */

#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. PANTALLA TFT 3.5" (ILI9488 SPI 480x320) & PANEL TÁCTIL (XPT2046)
 * ========================================================================= */
#define PIN_NUM_TFT_MOSI        GPIO_NUM_11 // Bus SPI2 Maestro (SDI TFT / T_DIN Touch)
#define PIN_NUM_TFT_CLK         GPIO_NUM_12 // Bus SPI2 Reloj (SCK TFT / T_CLK Touch)
#define PIN_NUM_TFT_MISO        GPIO_NUM_13 // Bus SPI2 Entrada (T_DO del Touch)

#define PIN_NUM_TFT_CS          GPIO_NUM_10 // Chip Select Display ILI9488
#define PIN_NUM_TFT_DC          GPIO_NUM_9  // Data/Command Display (RS)
#define PIN_NUM_TFT_RST         GPIO_NUM_14 // Reset Físico por Hardware Display

#define PIN_NUM_TOUCH_CS        GPIO_NUM_21 // Chip Select Touch XPT2046
#define PIN_NUM_TOUCH_IRQ       GPIO_NUM_39 // Interrupción Touch (Activo en LOW)

/* =========================================================================
 * 2. ALMACENAMIENTO LOCAL MICRO-SD (FATFS / VFS)
 * ========================================================================= */
#define PIN_NUM_SD_CS           GPIO_NUM_47 // Chip Select Ranura MicroSD integrada

/* =========================================================================
 * 3. MOTORES PASO A PASO Y CONTROLADORES TRINAMIC TMC2209 (STANDALONE)
 * ========================================================================= */
// Habilitación común de potencia (Activo en LOW para TMC2209)
#define PIN_NUM_MOTORS_EN       GPIO_NUM_3  // Enable común para X, Y1, Y2

// Eje X (Carro transversal porta-láser - Motor NEMA 17 42BYGHW609)
#define PIN_NUM_X_STEP          GPIO_NUM_1  // Pulso de paso Eje X
#define PIN_NUM_X_DIR           GPIO_NUM_2  // Dirección Eje X

// Eje Y (Pórtico Dual-Y paralelo - Motores NEMA 17 17HS4023 en tándem)
#define PIN_NUM_Y1_STEP         GPIO_NUM_4  // Pulso Eje Y1 (Izquierdo)
#define PIN_NUM_Y1_DIR          GPIO_NUM_5  // Dirección Eje Y1
#define PIN_NUM_Y2_STEP         GPIO_NUM_6  // Pulso Eje Y2 (Derecho)
#define PIN_NUM_Y2_DIR          GPIO_NUM_7  // Dirección Eje Y2

// Aliases para temporizadores cinemáticos
#define PIN_MOTORS_EN           PIN_NUM_MOTORS_EN
#define PIN_STEP_X              PIN_NUM_X_STEP
#define PIN_DIR_X               PIN_NUM_X_DIR
#define PIN_STEP_Y1             PIN_NUM_Y1_STEP
#define PIN_DIR_Y1              PIN_NUM_Y1_DIR
#define PIN_STEP_Y2             PIN_NUM_Y2_STEP
#define PIN_DIR_Y2              PIN_NUM_Y2_DIR

/* =========================================================================
 * 4. MÓDULO LÁSER ÓPTICO 20W (TREE LASER)
 * ========================================================================= */
#define PIN_NUM_LASER_PWM       GPIO_NUM_18 // Salida PWM de alta frecuencia (LEDC 5 kHz)

/* =========================================================================
 * 5. ZUMBADOR PASIVO DE ALERTAS (BUZZER)
 * ========================================================================= */
#define PIN_NUM_BUZZER          GPIO_NUM_38 // Aviso acústico de trabajo finalizado / fallos

/* =========================================================================
 * 6. SENSORES DE LÍMITE (FINALES DE CARRERA)
 * ========================================================================= */
#define PIN_NUM_LIMIT_X         GPIO_NUM_16 // Fin de carrera X (Entrada Pull-up, futuro)
#define PIN_NUM_LIMIT_Y         GPIO_NUM_17 // Fin de carrera Y (Entrada Pull-up, futuro)

/* =========================================================================
 * 7. PARÁMETROS CINEMÁTICOS DE HUSILLOS DE AVANCE RÁPIDO (4-START LEADSCREWS)
 * ========================================================================= */
// Husillos T8x8 (Paso 2 mm, 4 entradas = 8 mm de avance por revolución).
// Motor paso a paso estándar de 1.8° = 200 pasos/revolución.
// Con microstepping a 16x (MS1=3.3V, MS2=3.3V): 200 * 16 = 3200 micropasos/rev -> 400.0 pasos/mm.
// Con microstepping a 8x (MS1=NC, MS2=NC): 200 * 8 = 1600 micropasos/rev -> 200.0 pasos/mm.
// Calibración experimental real medida por el usuario:
// Eje Y: 20000 pasos / 100.0 mm medidos = 200.0 pasos/mm
// Eje X: 20000 pasos / 97.5 mm medidos = 205.13 pasos/mm
#define CNC_DEFAULT_X_STEPS_PER_MM   205.13f
#define CNC_DEFAULT_Y_STEPS_PER_MM   200.0f

#define CNC_MAX_TRAVEL_X_MM          102.0f
#define CNC_MAX_TRAVEL_Y_MM          82.0f

#define CNC_DEFAULT_MAX_FEEDRATE     800.0f  // mm/min (optimo anti-resonancia para husillos T8)
#define CNC_DEFAULT_ACCELERATION     60.0f   // mm/s^2 (rampa suave S-curve con jerk nulo)

#ifdef __cplusplus
}
#endif
