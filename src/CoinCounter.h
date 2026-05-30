#pragma once
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

class CoinCounter {
public:
    CoinCounter(gpio_num_t pin = GPIO_NUM_12, int meta = 12);

    void begin();   // Configura GPIO e ISR — llamar una vez en app_main
    void start();   // Inicia sesión de cobro (sumaTotal = -1 la primera vez, 0 las siguientes)
    bool update();  // Procesa pulsos; retorna true cuando metaAlcanzada

    int  getTotal() const;
    bool isMetaAlcanzada() const;

private:
    gpio_num_t _pin;
    int        _meta;
    bool       _first_run;
    int        _sumaTotal;
    bool       _metaAlcanzada;

    static portMUX_TYPE     _mux;
    static volatile int     _pulsosTemporales;
    static volatile int64_t _ultimoTiempoPulso;

    static void IRAM_ATTR _isr_handler(void* arg);

    static constexpr int64_t TIEMPO_ESPERA_US = 150000;
};
