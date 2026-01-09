#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"

// System clock frequency (16 MHz)
#define SYSTEM_CLOCK 16000000UL

// Control thresholds
#define TEMP_HIGH_THRESHOLD 25.0f    // Open vent above 25°C
#define TEMP_LOW_THRESHOLD 18.0f     // Close vent below 18°C
#define SOIL_DRY_THRESHOLD 30.0f     // Start watering below 30%
#define SOIL_WET_THRESHOLD 60.0f     // Stop watering above 60%

// Pin Definitions
#define UART_PORT GPIO_PORTA_BASE
#define UART_TX_PIN GPIO_PIN_1
#define UART_RX_PIN GPIO_PIN_0

#define MOTOR_PORT GPIO_PORTE_BASE
#define MOTOR_EN_PIN GPIO_PIN_0
#define MOTOR_DIR_PIN GPIO_PIN_1

// Sensor data structure
typedef struct {
    float temperature;      // in °C
    float humidity;         // in %
    float soilMoisture;     // in %
} SensorData_t;

// Global variables
SensorData_t currentSensors = {22.5f, 55.0f, 45.0f};
bool ventOpen = false;
bool pumpRunning = false;
uint32_t cycleCount = 0;

//-----------------------------------------------------------------------------
// UART Functions
//-----------------------------------------------------------------------------
void UART_SendString(const char *string) {
    while(*string) {
        UARTCharPut(UART0_BASE, *(string++));
    }
}

void UART_SendFloat(float value) {
    // Simple float display (XX.X format)
    int integer = (int)value;
    int fraction = (int)((value - integer) * 10);
    
    // Send integer part
    if(integer >= 100) {
        UARTCharPut(UART0_BASE, '0' + integer/100);
        integer %= 100;
    }
    if(integer >= 10) {
        UARTCharPut(UART0_BASE, '0' + integer/10);
        integer %= 10;
    }
    UARTCharPut(UART0_BASE, '0' + integer);
    
    // Send decimal and fraction
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, '0' + fraction);
}

//-----------------------------------------------------------------------------
// Peripheral Initialization
//-----------------------------------------------------------------------------
void initUART(void) {
    // Enable UART0 and GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    // Wait for UART to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    
    // Configure GPIO pins for UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(UART_PORT, UART_RX_PIN | UART_TX_PIN);
    
    // Configure UART (115200, 8N1)
    UARTConfigSetExpClk(UART0_BASE, SYSTEM_CLOCK, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE));
    
    // Enable UART
    UARTEnable(UART0_BASE);
}

void initMotorControl(void) {
    // Enable GPIOE for motor control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    
    // Configure pins as outputs
    GPIOPinTypeGPIOOutput(MOTOR_PORT, MOTOR_EN_PIN | MOTOR_DIR_PIN);
    
    // Start with motor off
    GPIOPinWrite(MOTOR_PORT, MOTOR_EN_PIN | MOTOR_DIR_PIN, 0);
}

void initServoPWM(void) {
    // Enable PWM0 and GPIOB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    // Configure PB6 as PWM output
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    
    // Configure PWM generator
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, 
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    // Set period for 50Hz servo signal (20ms = 320,000 cycles)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 320000);
    
    // Set initial pulse width for 0° (1ms = 16,000 cycles)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 16000);
    
    // Enable PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

//-----------------------------------------------------------------------------
// Sensor Simulation (I2C & SPI)
//-----------------------------------------------------------------------------
void simulateI2CSensor(void) {
    static float baseTemp = 22.0f;
    static float baseHumidity = 55.0f;
    
    // Simulate realistic environmental changes
    baseTemp += ((rand() % 100) - 50) * 0.01f;   // ±0.5°C
    baseHumidity += ((rand() % 100) - 50) * 0.02f; // ±1%
    
    // Keep within realistic greenhouse ranges
    if(baseTemp < 15.0f) baseTemp = 15.0f;
    if(baseTemp > 35.0f) baseTemp = 35.0f;
    if(baseHumidity < 30.0f) baseHumidity = 30.0f;
    if(baseHumidity > 80.0f) baseHumidity = 80.0f;
    
    currentSensors.temperature = baseTemp;
    currentSensors.humidity = baseHumidity;
}

void simulateSPISensor(void) {
    static float soilMoisture = 45.0f;
    
    // Simulate drying and watering effects
    if(pumpRunning) {
        soilMoisture += 0.5f;    // Increase when watering
        if(soilMoisture > 100.0f) soilMoisture = 100.0f;
    } else {
        soilMoisture -= 0.1f;    // Gradual drying
        if(soilMoisture < 0.0f) soilMoisture = 0.0f;
    }
    
    currentSensors.soilMoisture = soilMoisture;
}

//-----------------------------------------------------------------------------
// Control Functions
//-----------------------------------------------------------------------------
void controlVent(void) {
    if(currentSensors.temperature > TEMP_HIGH_THRESHOLD && !ventOpen) {
        // Open vent (2ms pulse = 180°)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 32000);
        ventOpen = true;
        UART_SendString("VENT: OPEN (Temp: ");
        UART_SendFloat(currentSensors.temperature);
        UART_SendString("°C)\r\n");
    }
    else if(currentSensors.temperature < TEMP_LOW_THRESHOLD && ventOpen) {
        // Close vent (1ms pulse = 0°)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 16000);
        ventOpen = false;
        UART_SendString("VENT: CLOSE (Temp: ");
        UART_SendFloat(currentSensors.temperature);
        UART_SendString("°C)\r\n");
    }
}

void controlWaterPump(void) {
    if(currentSensors.soilMoisture < SOIL_DRY_THRESHOLD && !pumpRunning) {
        // Start watering
        GPIOPinWrite(MOTOR_PORT, MOTOR_EN_PIN, MOTOR_EN_PIN);
        pumpRunning = true;
        UART_SendString("PUMP: ON (Soil: ");
        UART_SendFloat(currentSensors.soilMoisture);
        UART_SendString("%)\r\n");
    }
    else if(currentSensors.soilMoisture > SOIL_WET_THRESHOLD && pumpRunning) {
        // Stop watering
        GPIOPinWrite(MOTOR_PORT, MOTOR_EN_PIN, 0);
        pumpRunning = false;
        UART_SendString("PUMP: OFF (Soil: ");
        UART_SendFloat(currentSensors.soilMoisture);
        UART_SendString("%)\r\n");
    }
}

//-----------------------------------------------------------------------------
// Display Function
//-----------------------------------------------------------------------------
void displayStatus(void) {
    static uint32_t displayCount = 0;
    
    if(++displayCount >= 10) {  // Display every 10 cycles
        displayCount = 0;
        
        UART_SendString("\r\n--- Greenhouse Status ---\r\n");
        
        UART_SendString("Temperature: ");
        UART_SendFloat(currentSensors.temperature);
        UART_SendString("°C\r\n");
        
        UART_SendString("Humidity:    ");
        UART_SendFloat(currentSensors.humidity);
        UART_SendString("%\r\n");
        
        UART_SendString("Soil:        ");
        UART_SendFloat(currentSensors.soilMoisture);
        UART_SendString("%\r\n");
        
        UART_SendString("Vent:        ");
        UART_SendString(ventOpen ? "OPEN" : "CLOSED");
        UART_SendString("\r\n");
        
        UART_SendString("Water Pump:  ");
        UART_SendString(pumpRunning ? "ON" : "OFF");
        UART_SendString("\r\n");
        
        UART_SendString("Cycle:       ");
        // Simple cycle count display
        uint32_t temp = cycleCount;
        if(temp == 0) {
            UART_SendString("0");
        } else {
            char buffer[10];
            int i = 0;
            while(temp > 0) {
                buffer[i++] = '0' + (temp % 10);
                temp /= 10;
            }
            while(i > 0) {
                UARTCharPut(UART0_BASE, buffer[--i]);
            }
        }
        UART_SendString("\r\n");
        
        UART_SendString("-------------------------\r\n");
    }
}

//-----------------------------------------------------------------------------
// Delay Function
//-----------------------------------------------------------------------------
void delayMs(uint32_t milliseconds) {
    // Simple delay using SysCtlDelay (3 cycles per iteration)
    uint32_t delayCount = (SYSTEM_CLOCK / 3000) * milliseconds;
    SysCtlDelay(delayCount);
}

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
int main(void) {
    // Initialize system clock (16 MHz)
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | 
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    // Initialize all peripherals
    initUART();
    initMotorControl();
    initServoPWM();
    
    // System startup message
    UART_SendString("\r\n================================\r\n");
    UART_SendString("GREENHOUSE CONTROL SYSTEM\r\n");
    UART_SendString("Simplified TM4C123G Implementation\r\n");
    UART_SendString("Initialization Complete\r\n");
    UART_SendString("================================\r\n\r\n");
    
    // Seed random number generator for sensor simulation
    // Using system timer as simple seed
    uint32_t seed = SysCtlClockGet();
    srand(seed);
    
    // Main control loop
    while(1) {
        // Update simulated sensor readings
        simulateI2CSensor();    // Temperature & humidity
        simulateSPISensor();    // Soil moisture
        
        // Execute control logic
        controlVent();          // Temperature-based vent control
        controlWaterPump();     // Soil moisture-based watering
        
        // Display current status
        displayStatus();
        
        // Increment cycle counter
        cycleCount++;
        
        // Wait 1 second before next cycle
        delayMs(1000);
    }
    
    // Should never reach here
    return 0;
}