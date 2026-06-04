#include "CoinCounter.h"
#include "esp_timer.h"
#include <stdio.h>

// Definición de miembros estáticos
portMUX_TYPE     CoinCounter::_mux              = portMUX_INITIALIZER_UNLOCKED;
volatile int     CoinCounter::_pulsosTemporales  = 0;
volatile int64_t CoinCounter::_ultimoTiempoPulso = 0;

CoinCounter::CoinCounter(gpio_num_t pin, int meta)
    : _pin(pin), _meta(meta), _first_run(true),
      _sumaTotal(-1), _metaAlcanzada(false), _lastPulses(0) {} 

void IRAM_ATTR CoinCounter::_isr_handler(void* arg) {
    int64_t ahora = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&_mux);
    // Solo cuenta si pasó el tiempo de debounce desde el último pulso válido
    if ((ahora - _ultimoTiempoPulso) >= DEBOUNCE_PULSO_US) {
        _pulsosTemporales++;
        _ultimoTiempoPulso = ahora;
    }
    portEXIT_CRITICAL_ISR(&_mux);
}

void CoinCounter::begin() {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_POSEDGE;   // dispara en cada flanco HIGH
    io_conf.pin_bit_mask = (1ULL << _pin);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // reposo en LOW
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(_pin, _isr_handler, nullptr);
}

void CoinCounter::start() {
    _sumaTotal     = _first_run ? 0 : 0;
    _metaAlcanzada = false;
    _first_run     = false;

    portENTER_CRITICAL(&_mux);
    _pulsosTemporales  = 0;
    _ultimoTiempoPulso = esp_timer_get_time() - DEBOUNCE_PULSO_US; // permite el primer pulso inmediatamente
    portEXIT_CRITICAL(&_mux);
}

bool CoinCounter::update() {
    if (_metaAlcanzada) return true;
    if (_pulsosTemporales == 0) return false;

    int64_t tiempoActual = esp_timer_get_time();
    if ((tiempoActual - _ultimoTiempoPulso) <= TIEMPO_ESPERA_US) return false;

    portENTER_CRITICAL(&_mux);
    int pulsosConfirmados = _pulsosTemporales;
    _pulsosTemporales     = 0;
    portEXIT_CRITICAL(&_mux);

    _lastPulses  = pulsosConfirmados;
    _sumaTotal  += pulsosConfirmados;   // cada pulso HIGH = 1 unidad

    printf("Pulsos: %d | Total: $%d\n", pulsosConfirmados, _sumaTotal);

    if (_sumaTotal >= _meta) {
        _metaAlcanzada = true;
        printf("=> Meta alcanzada ($%d).\n", _sumaTotal);
    }

    return _metaAlcanzada;
}

int  CoinCounter::getTotal()        const { return _sumaTotal;     }
bool CoinCounter::isMetaAlcanzada() const { return _metaAlcanzada; }
int  CoinCounter::getLastPulses()   const { return _lastPulses;    }
