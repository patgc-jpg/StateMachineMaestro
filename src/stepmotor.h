#pragma once

#include <stdint.h>
#include <driver/gpio.h>
#include <esp_timer.h>

class StepperMotor {
public:
    // enaPin opcional: si se pasa, se configura activo-LOW (TB6600 ENA-)
    StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin, uint32_t delayUs,
                 gpio_num_t enaPin = GPIO_NUM_MAX);

    void begin();
    bool update();                   // Non-blocking; returns true if a step fired
    void setDelay(uint32_t delayUs);
    void setDirection(bool forward); // true = forward (DIR=HIGH), false = reverse (DIR=LOW)
    void resetTimer();               // Reinicia el timer interno (evita paso inmediato al reanudar)
    void enable();                   // ENA- = LOW  → driver activo
    void disable();                  // ENA- = HIGH → driver apagado (sin torque)
    void ReverseDirec();

private:
    gpio_num_t _stepPin;
    gpio_num_t _dirPin;
    gpio_num_t _enaPin;
    uint32_t   stepDelayUs;
    uint64_t   lastTime;
    bool       _forward;
};
