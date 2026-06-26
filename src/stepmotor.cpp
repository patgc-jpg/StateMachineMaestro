#include "stepmotor.h"
#include <esp_rom_sys.h>

StepperMotor::StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin, uint32_t delayUs,
                            gpio_num_t enaPin)
    : _stepPin(stepPin), _dirPin(dirPin), _enaPin(enaPin),
      stepDelayUs(delayUs), lastTime(0), _forward(true) {}

void StepperMotor::begin() {
    gpio_reset_pin(_stepPin);
    gpio_set_direction(_stepPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_stepPin, 0);

    gpio_reset_pin(_dirPin);
    gpio_set_direction(_dirPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_dirPin, _forward ? 1 : 0);

    // ENA es activo-LOW en el TB6600: LOW = driver habilitado
    if (_enaPin != GPIO_NUM_MAX) {
        gpio_reset_pin(_enaPin);
        gpio_set_direction(_enaPin, GPIO_MODE_OUTPUT);
        gpio_set_level(_enaPin, 0);   // habilita el driver al arrancar
    }

    lastTime = esp_timer_get_time();
}

// Genera un pulso STEP no-bloqueante cuando el timer interno lo permite.
// TB6600: TSET(DIR→PUL) ≥ 5 µs   |   ancho de pulso PUL ≥ 2.2 µs (usamos 5 µs)
bool StepperMotor::update() {
    uint64_t now = esp_timer_get_time();
    if (now - lastTime >= stepDelayUs) {
        esp_rom_delay_us(5);          // garantiza TSET ≥ 5 µs tras cualquier cambio de DIR
        gpio_set_level(_stepPin, 1);
        esp_rom_delay_us(5);          // ancho de pulso PUL HIGH
        gpio_set_level(_stepPin, 0);
        lastTime = now;
        return true;
    }
    return false;
}

void StepperMotor::setDirection(bool forward) {
    if (_forward == forward) return;  // sin cambio → sin escritura GPIO innecesaria
    _forward = forward;
    gpio_set_level(_dirPin, forward ? 1 : 0);
}

void StepperMotor::resetTimer() {
    lastTime = esp_timer_get_time();
}

void StepperMotor::enable() {
    if (_enaPin != GPIO_NUM_MAX)
        gpio_set_level(_enaPin, 0);   // activo-LOW
}

void StepperMotor::disable() {
    if (_enaPin != GPIO_NUM_MAX)
        gpio_set_level(_enaPin, 1);   // HIGH = driver apagado
}

void StepperMotor::ReverseDirec() {
    setDirection(!_forward);
}

void StepperMotor::setDelay(uint32_t delayUs) {
    stepDelayUs = delayUs;
}
