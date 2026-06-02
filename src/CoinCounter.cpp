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
    portENTER_CRITICAL_ISR(&_mux);
    _pulsosTemporales++;
    _ultimoTiempoPulso = esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&_mux);
}

void CoinCounter::begin() {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << _pin);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
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
    _ultimoTiempoPulso = 0;
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

    _lastPulses = pulsosConfirmados;   // guarda para debug en LCD

    int valorMoneda = 0;
    if      (pulsosConfirmados >= 50  && pulsosConfirmados <= 100)  valorMoneda = 1;
    else if (pulsosConfirmados >= 130 && pulsosConfirmados <= 200)  valorMoneda = 2;
    else if (pulsosConfirmados >= 300 && pulsosConfirmados <= 480) valorMoneda = 5;
    else if (pulsosConfirmados >= 700 && pulsosConfirmados <= 1000) valorMoneda = 10;

    if (valorMoneda > 0) {
        _sumaTotal += valorMoneda;
        printf("Lectura: %d pulsos -> $%d | Total: $%d\n",
               pulsosConfirmados, valorMoneda, _sumaTotal);

        if (_sumaTotal >= _meta) {
            _metaAlcanzada = true;
            printf("=> Credito suficiente ($%d). Meta alcanzada.\n", _sumaTotal);
        }
    } else {
        printf("Ruido descartado: %d pulsos\n", pulsosConfirmados);
    }

    return _metaAlcanzada;
}

int  CoinCounter::getTotal()        const { return _sumaTotal;     }
bool CoinCounter::isMetaAlcanzada() const { return _metaAlcanzada; }
int  CoinCounter::getLastPulses()   const { return _lastPulses;    }
