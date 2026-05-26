// ============================================================
// main.cpp — ESP32 MASTER — Máquina de Garra (Claw Machine)
// ============================================================
// Periféricos en ESTE ESP (maestro):
//   - 2× NEMA stepper → eje X  (uno montado invertido, corregido por software)
//   - LCD 16×2 por I2C
//   - Tragamonedas: 4 líneas digitales  (+ botón de simulación temporal)
//   - Botón START / push
//   - Joystick eje X  (ADC)
//   - Sensor de proximidad en la compuerta de premio
//   - Señal de salida de cambio
//
// Comunicación con ESP ESCLAVO (maneja eje Y + eje Z + cierre de garra):
//   SLAVE_BEGIN (salida, GPIO15): maestro → esclavo; HIGH = "ejecuta tu siguiente fase"
//   SLAVE_DONE  (entrada, GPIO2): esclavo → maestro; HIGH = "terminé"
//   Protocolo: maestro pone BEGIN=HIGH → esclavo ejecuta su fase actual
//              esclavo pone DONE=HIGH → maestro pone BEGIN=LOW → esclavo pone DONE=LOW
//   El esclavo distingue "centrar Y" vs "secuencia de garra" por su propio estado.
// ============================================================

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>

#include "stepmotor.h"
#include "lcd.h"
#include "dbutton.h"
//todos tontos menos nosotros aaaaa
// ============================================================
// CONSTANTES MECÁNICAS
// Correa GT2 (paso 2 mm) + polea 20 dientes = 40 mm/vuelta
// NEMA 200 pasos/vuelta  →  0.2 mm por paso (modo full-step)
// ============================================================
#define MM_PER_STEP         0.2f
#define AXIS_LENGTH_MM      550.0f
#define AXIS_MAX_STEPS      ((int32_t)(AXIS_LENGTH_MM / MM_PER_STEP))   // 2750 pasos
#define AXIS_CENTER_STEPS   (AXIS_MAX_STEPS / 2)                         // 1375 pasos = 275 mm
#define AXIS_HOME_STEPS     0
//
// ============================================================
// VELOCIDAD DE MOTORES
// stepDelayUs: menor valor = más rápido; mínimo seguro ≈ 500 µs para NEMA
// ============================================================
#define STEP_DELAY_MIN_US   1000UL    // máxima velocidad (joystick en extremo)
#define STEP_DELAY_MAX_US   5000UL   // mínima velocidad (recién sale del deadband)
#define STEP_DELAY_TRAV_US  1000UL   // velocidad de travesía: BEGIN, ZERO_X (mitad de máxima)

// ============================================================
// BOTONES DIRECCIONALES (reemplazan joystick analógico)
// Pull-down interno: reposo = LOW, presionado = HIGH (botón conecta pin a 3.3V)
// ============================================================
#define BTN_LEFT   GPIO_NUM_14
#define BTN_RIGHT  GPIO_NUM_27
/*#define BTN_UP     GPIO_NUM_16   // reservado (eje Y va al esclavo)
#define BTN_DOWN   GPIO_NUM_17   // reservado (eje Y va al esclavo)*/

// ============================================================
// DURACIONES (µs)
// ============================================================
#define DEFAULT_DUR_US  (5LL  * 1000000LL)   // 5 s  pantalla de bienvenida
#define GAME_DUR_US     (20LL * 1000000LL)   // 20 s timer de juego
#define END_DUR_US      (5LL  * 1000000LL)   // 5 s  pantalla ganador/perdedor

// ============================================================
// MONEDAS / PRECIO
// ============================================================
#define COIN_PRICE        12   // pesos necesarios para jugar
#define COIN_SIM_PRESSES  3    // pulsaciones del botón de simulación para "pagar"

// ============================================================
// DEBOUNCE
// ============================================================
#define DEBOUNCE_US  40000UL   // 40 ms

// ============================================================
// PINES GPIO
// Nota: GPIO34, 35, 36, 39 son solo-entrada — sin pull-up/down interno.
//       Usa resistencias externas de 10 kΩ a 3.3 V donde se indica.
// ============================================================

// Motor X1 — lado izquierdo, dirección normal
#define MX1_STEP  GPIO_NUM_19
#define MX1_DIR   GPIO_NUM_18

// Motor X2 — lado derecho, montado invertido → se invierte por software
#define MX2_STEP  GPIO_NUM_4
#define MX2_DIR   GPIO_NUM_16

// LCD I2C
#define LCD_SDA_NUM   GPIO_NUM_21
#define LCD_SCL_NUM   GPIO_NUM_22
#define LCD_I2C_ADDR  0x27

// Tragamonedas — 4 líneas activo-LOW
// GPIO34/35 son solo-entrada → pull-up externo de 10 kΩ obligatorio
#define COIN_PIN_1   GPIO_NUM_32
#define COIN_PIN_2   GPIO_NUM_33
#define COIN_PIN_5   GPIO_NUM_34   // solo-entrada — pull-up externo
#define COIN_PIN_10  GPIO_NUM_35   // solo-entrada — pull-up externo

// Botones
#define BTN_COIN_SIM  GPIO_NUM_25   // simulación: pulsar COIN_SIM_PRESSES veces = monedas listas
#define BTN_START     GPIO_NUM_26   // botón start / bajar garra

// Sensor de proximidad en compuerta de premio (activo-LOW)
// GPIO39 es solo-entrada → pull-up externo de 10 kΩ obligatorio
#define PROX_SENSOR  GPIO_NUM_39

// Comunicación maestro ↔ esclavo
#define SLAVE_BEGIN  GPIO_NUM_15   // SALIDA: HIGH = esclavo ejecuta su siguiente fase
#define SLAVE_DONE   GPIO_NUM_2    // ENTRADA: HIGH = esclavo terminó su fase actual

// Señal de dispensador de cambio
#define CHANGE_OUT  GPIO_NUM_5    // SALIDA: HIGH mientras debe darse cambio

// ============================================================
// MÁQUINA DE ESTADOS
// ============================================================
enum State {
    STATE_DEFAULT,
    STATE_MONEY,
    STATE_BEGIN,
    STATE_GAME,
    STATE_WAIT_SLAVE,
    STATE_ZERO_X,
    STATE_WINNER,
    STATE_LOSER,
    NUM_STATES
};
typedef State (*StateAction)();
struct StateNode { const char *name; StateAction on_loop; };

// ============================================================
// OBJETOS DE HARDWARE
// ============================================================
static StepperMotor   motorX1(MX1_STEP, MX1_DIR, STEP_DELAY_TRAV_US);
static StepperMotor   motorX2(MX2_STEP, MX2_DIR, STEP_DELAY_TRAV_US);
static LCD            lcd;
static DebouncedButton btnCoinSim(BTN_COIN_SIM, DEBOUNCE_US);
static DebouncedButton btnStart  (BTN_START,    DEBOUNCE_US);
static DebouncedButton btnLeft   (BTN_LEFT,     DEBOUNCE_US);
static DebouncedButton btnRight  (BTN_RIGHT,    DEBOUNCE_US);
/*static DebouncedButton btnUp     (BTN_UP,       DEBOUNCE_US);
static DebouncedButton btnDown   (BTN_DOWN,     DEBOUNCE_US);
*/
// ============================================================
// VARIABLES GLOBALES DE ESTADO
// ============================================================
static bool    is_new_state   = true;
static int64_t state_start_us = 0;

// Posición del eje X en pasos desde home (0 = home, AXIS_CENTER_STEPS = centro)
static int32_t x_steps = 0;

// Dinero y simulación
static int  money_total    = 0;
static int  coin_sim_count = 0;
static bool sim_ready      = false;

// Detección de flanco: estado anterior de cada botón que necesita evento "presionado"
static bool prev_btn_coin_sim = false;
static bool prev_btn_start    = false;


// Estado anterior del segundo de juego (para actualizar LCD sólo al cambiar el segundo)
static int32_t game_prev_sec = -1;

// Cache del último mensaje enviado al LCD (evita reescribir si no cambió)
static char lcd_last_msg[33] = "";

// ============================================================
// HELPERS GENERALES
// ============================================================

// Detecta flanco ascendente en DebouncedButton (evento único de presión).
// 'prev' guarda el estado anterior; se pasa por referencia para persistir entre llamadas.
static bool justPressed(DebouncedButton &btn, bool &prev) {
    bool cur  = btn.isPressed();
    bool edge = cur && !prev;
    prev = cur;
    return edge;
}

// Escribe al LCD sólo cuando el mensaje cambia, para evitar parpadeo por reescritura.
static void lcdUpdate(const char *msg) {
    if (strncmp(msg, lcd_last_msg, 32) != 0) {
        lcd.printStr(msg);
        strncpy(lcd_last_msg, msg, 32);
        lcd_last_msg[32] = '\0';
    }
}

// Ejecutar al entrar en un estado nuevo: resetea timer, botones y cache de LCD.
static void onEnterState() {
    state_start_us    = esp_timer_get_time();
    prev_btn_coin_sim = false;
    prev_btn_start    = false;
    lcd_last_msg[0]   = '\0';   // fuerza escritura al LCD en la primera llamada
    btnCoinSim.reset();
    btnStart.reset();
}

// ============================================================
// CONTROL DE MOTORES X
// Avanza ambos motores un paso en la dirección indicada y actualiza x_steps.
//   dir = +1 → dirección positiva (hacia el extremo del eje)
//   dir = -1 → dirección negativa (hacia home)
// Motor X2 se montó invertido en el gantry, por eso siempre corre en sentido
// opuesto eléctricamente a X1 para que ambos muevan el carro en la misma dirección.
// Retorna true cuando motorX1 ejecutó un paso (sirve para contar posición).
// ============================================================
static bool stepX(int dir) {
    bool going_pos = (dir > 0);

    // X1: adelante para X positivo   |   X2: invertido porque está montado al revés
    motorX1.setDirection(going_pos);
    motorX2.setDirection(going_pos);

    bool stepped = motorX1.update();   // true si el timer interno disparó un paso
    motorX2.update();                  // mismo delay → ambos avanzan juntos

    if (stepped) x_steps += going_pos ? 1 : -1;
    return stepped;
}

// Dirección de movimiento según botones con debounce: +1 (derecha), -1 (izquierda), 0 (ninguno)
static int joyToDir() {
    if (btnRight.isPressed()) return  1;
    if (btnLeft.isPressed())  return -1;
    return 0;
}

// ============================================================
// TRAGAMONEDAS
// Detecta flancos en las 4 líneas del aceptador de monedas (modo real, placeholder).
// En simulación: COIN_SIM_PRESSES pulsaciones del botón BTN_COIN_SIM activan sim_ready.
// ============================================================
static void pollCoins() {
    // --- Aceptador real: descomenta cuando esté conectado ---
    // Las líneas son activo-LOW: flanco descendente = moneda insertada.
    // bool c1  = !gpio_get_level(COIN_PIN_1);
    // bool c2  = !gpio_get_level(COIN_PIN_2);
    // bool c5  = !gpio_get_level(COIN_PIN_5);
    // bool c10 = !gpio_get_level(COIN_PIN_10);
    // if (c1  && !prev_coin1)  money_total += 1;
    // if (c2  && !prev_coin2)  money_total += 2;
    // if (c5  && !prev_coin5)  money_total += 5;
    // if (c10 && !prev_coin10) money_total += 10;
    // prev_coin1=c1; prev_coin2=c2; prev_coin5=c5; prev_coin10=c10;

    // --- Simulación temporal ---
    if (justPressed(btnCoinSim, prev_btn_coin_sim)) {
        if (coin_sim_count < COIN_SIM_PRESSES) coin_sim_count++;
        if (coin_sim_count >= COIN_SIM_PRESSES) sim_ready = true;
        printf("[BTN_COIN_SIM] Pulsaciones: %d/%d%s\n",
               coin_sim_count, COIN_SIM_PRESSES,
               sim_ready ? " — LISTO" : "");
    }
}

// ============================================================
// ESTADOS
// ============================================================

// DEFAULT — Pantalla de bienvenida durante DEFAULT_DUR_US, luego va a MONEY
static State executeDefault() {
    if (is_new_state) onEnterState();

    // TODO ← coloca tu mensaje para la pantalla DEFAULT (máx 16 chars por línea, '\n' = 2ª línea)
    lcdUpdate("Iniciando Maquina \n Bienvenid@");

    if (esp_timer_get_time() - state_start_us >= DEFAULT_DUR_US)
        return STATE_MONEY;
    return STATE_DEFAULT;
}

// MONEY — Espera monedas (simulación: 3 pulsaciones) y luego el botón START
static State executeMoney() {
    if (is_new_state) {
        onEnterState();
        money_total    = 0;
        coin_sim_count = 0;
        sim_ready      = false;
        x_steps        = 0;
        gpio_set_level(SLAVE_BEGIN, 0);
        gpio_set_level(CHANGE_OUT,  0);
    }

    pollCoins();

    // paid = suficiente dinero (monedas reales O simulación completada)
    bool paid = sim_ready || (money_total >= COIN_PRICE);

    char msg[48];
    if (!paid) {
        // Fase 1: mostrar dinero acumulado y avance de simulación
        snprintf(msg, sizeof(msg),"BIENVENIDO@ \n $%d/$%d pesos",
                 money_total, COIN_PRICE );
    } else {
        // Fase 2: precio cubierto, esperar START
        snprintf(msg, sizeof(msg), "Listo! $%d\nPulsa START!", COIN_PRICE);
    }
    lcdUpdate(msg);

    // Siempre se requiere START después de pagar
    if (paid && justPressed(btnStart, prev_btn_start)) {
        motorX1.setDelay(STEP_DELAY_TRAV_US);
        motorX2.setDelay(STEP_DELAY_TRAV_US);
        return STATE_BEGIN;
    }
    return STATE_MONEY;
}

// BEGIN — Lleva el eje X al centro; avisa al esclavo que haga lo mismo en Y
static State executeBegin() {
    if (is_new_state) {
        onEnterState();
        motorX1.setDelay(STEP_DELAY_TRAV_US);
        motorX2.setDelay(STEP_DELAY_TRAV_US);
        gpio_set_level(SLAVE_BEGIN, 1);   // esclavo empieza a mover Y al centro
    }

    lcdUpdate("Centrando garra\nEspera...");

    if      (x_steps < AXIS_CENTER_STEPS) stepX(+1);
    else if (x_steps > AXIS_CENTER_STEPS) stepX(-1);
    else {
        gpio_set_level(SLAVE_BEGIN, 0);   // X llegó al centro, baja señal
        return STATE_GAME;
    }

    return STATE_BEGIN;
}

// GAME — Joystick controla X; speed binaria; timer de 20 s
static State executeGame() {
    static int64_t joy_grace_us = 0;
    static int     joy_last_dir = 0;

    if (is_new_state) {
        onEnterState();
        game_prev_sec = -1;
        joy_grace_us  = 0;
        joy_last_dir  = 0;
    }

    // Leer botones con período de gracia de 60 ms para absorber glitches de EMI
    // (el debounce tarda hasta 40 ms en re-establecerse tras un ruido breve)
    int dir = joyToDir();
    if (dir != 0) {
        joy_last_dir = dir;
        joy_grace_us = esp_timer_get_time();
    } else if (esp_timer_get_time() - joy_grace_us < 60000LL) {
        dir = joy_last_dir;
    }

    if (dir != 0) {
        bool at_limit = (dir > 0 && x_steps >= AXIS_MAX_STEPS - 1) ||
                        (dir < 0 && x_steps <= AXIS_HOME_STEPS);
        if (!at_limit) {
            motorX1.setDelay(STEP_DELAY_MIN_US);
            motorX2.setDelay(STEP_DELAY_MIN_US);
            stepX(dir);
        }
    }

    // Actualizar LCD una vez por segundo con el tiempo restante
    int64_t elapsed_us  = esp_timer_get_time() - state_start_us;
    /**/
    int32_t remain_s    = (int32_t)((GAME_DUR_US - elapsed_us) / 1000000LL);
    if (remain_s < 0) remain_s = 0;
    if (remain_s != game_prev_sec) {
        game_prev_sec = remain_s;
        char msg[33];
        // TODO ← personaliza el mensaje de GAME (el timer se muestra en remain_s)
        snprintf(msg, sizeof(msg), "Tiempo: %2lds\nPulsa START", remain_s);
        lcdUpdate(msg);
    }

    // Transición: botón START O timer agotado
   // bool time_up = (elapsed_us >= GAME_DUR_US);
    if (justPressed(btnStart, prev_btn_start)  ) //Aqui va el ||  time up
        return STATE_WAIT_SLAVE;

    return STATE_GAME;
}

// WAIT_SLAVE — Activa al esclavo para que baje, cierre y suba la garra.
// Mientras espera, muestra un mensaje diferente en el LCD.
// El esclavo gestiona Y durante todo esto; esta ESP solo espera la señal DONE.
static State executeWaitSlave() {
    if (is_new_state) {
        onEnterState();
        gpio_set_level(SLAVE_BEGIN, 1);   // dispara la secuencia del esclavo
    }

    // TODO ← puedes mostrar sub-fases distintas según el tiempo transcurrido
    lcdUpdate("Bajando garra...\nEspera");

    // El esclavo pone SLAVE_DONE=HIGH cuando termina (bajar → cerrar → subir)
    if (gpio_get_level(SLAVE_DONE) == 1) {
        gpio_set_level(SLAVE_BEGIN, 0);   // desactiva; el esclavo bajará DONE en respuesta
        motorX1.setDelay(STEP_DELAY_TRAV_US);
        motorX2.setDelay(STEP_DELAY_TRAV_US);
        return STATE_ZERO_X;
    }
    return STATE_WAIT_SLAVE;
}

// ZERO_X — Regresa el eje X a home (0 pasos) a velocidad de travesía.
// Al llegar, lee el sensor de proximidad para decidir WINNER o LOSER.
static State executeZeroX() {
    if (is_new_state) {
        onEnterState();
        motorX1.setDelay(STEP_DELAY_TRAV_US);
        motorX2.setDelay(STEP_DELAY_TRAV_US);
    }

    // TODO ← coloca tu mensaje para ZERO_X
    lcdUpdate("Regresando...\n");

    if      (x_steps > AXIS_HOME_STEPS) stepX(-1);
    else if (x_steps < AXIS_HOME_STEPS) stepX(+1);
    else {
        // Llegó a home — revisar si cayó un premio
        // Sensor activo-LOW: LOW = premio detectado; GPIO39 requiere pull-up externo
        bool prize = (gpio_get_level(PROX_SENSOR) == 0);

        // Activar dispensador de cambio si se pagó de más con monedas reales
        int change = money_total - COIN_PRICE;
        if (change > 0) gpio_set_level(CHANGE_OUT, 1);

        return prize ? STATE_WINNER : STATE_LOSER;
    }
    return STATE_ZERO_X;
}

// WINNER — Premio detectado; muestra felicitación por END_DUR_US y vuelve a MONEY
static State executeWinner() {
    if (is_new_state) {
        onEnterState();
        // TODO ← coloca tu mensaje para WINNER
        lcdUpdate("Premio!\nFelicidades! :)");
    }
    if (esp_timer_get_time() - state_start_us >= END_DUR_US) {
        gpio_set_level(CHANGE_OUT, 0);
        return STATE_MONEY;
    }
    return STATE_WINNER;
}

// LOSER — Sin premio; muestra mensaje por END_DUR_US y vuelve a MONEY
static State executeLoser() {
    if (is_new_state) {
        onEnterState();
        // TODO ← coloca tu mensaje para LOSER
        lcdUpdate("Sin premio...\nIntenta de nuevo!");
    }
    if (esp_timer_get_time() - state_start_us >= END_DUR_US) {
        gpio_set_level(CHANGE_OUT, 0);
        return STATE_MONEY;
    }
    return STATE_LOSER;
}

// ============================================================
// TABLA DE ESTADOS
// ============================================================
static StateNode state_table[NUM_STATES] = {
    { "DEFAULT",    executeDefault    },
    { "MONEY",      executeMoney      },
    { "BEGIN",      executeBegin      },
    { "GAME",       executeGame       },
    { "WAIT_SLAVE", executeWaitSlave  },
    { "ZERO_X",     executeZeroX      },
    { "WINNER",     executeWinner     },
    { "LOSER",      executeLoser      },
};

// ============================================================
// INICIALIZACIÓN DE GPIO
// ============================================================
static void setupGPIO() {
    // Salidas digitales — arrancan en LOW
    gpio_reset_pin(SLAVE_BEGIN);
    gpio_set_direction(SLAVE_BEGIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SLAVE_BEGIN, 0);

    gpio_reset_pin(CHANGE_OUT);
    gpio_set_direction(CHANGE_OUT, GPIO_MODE_OUTPUT);
    gpio_set_level(CHANGE_OUT, 0);

    // Entrada del esclavo: pull-down para que no dispare en flotante
    gpio_reset_pin(SLAVE_DONE);
    gpio_set_direction(SLAVE_DONE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SLAVE_DONE, GPIO_PULLDOWN_ONLY);

    // Sensor de proximidad: GPIO39 es solo-entrada — NO tiene pull interno
    // → conecta resistencia externa de 10 kΩ entre GPIO39 y 3.3 V
    gpio_set_direction(PROX_SENSOR, GPIO_MODE_INPUT);

    // Tragamonedas línea $1 y $2 — pines bidireccionales, pull-up interno OK
    gpio_reset_pin(COIN_PIN_1);
    gpio_set_direction(COIN_PIN_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(COIN_PIN_1, GPIO_PULLUP_ONLY);

    gpio_reset_pin(COIN_PIN_2);
    gpio_set_direction(COIN_PIN_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(COIN_PIN_2, GPIO_PULLUP_ONLY);

    // Tragamonedas línea $5 y $10 — GPIO34/35 son solo-entrada, sin pull interno
    // → conecta resistencias externas de 10 kΩ a 3.3 V
    gpio_set_direction(COIN_PIN_5,  GPIO_MODE_INPUT);
    gpio_set_direction(COIN_PIN_10, GPIO_MODE_INPUT);

}

// ============================================================
// ENTRADA PRINCIPAL
// ============================================================
extern "C" void app_main() {
    esp_task_wdt_deinit();

    setupGPIO();

    motorX1.begin();
    motorX2.begin();
    // X2 arranca en reversa porque está físicamente montado al revés del gantry.
    // stepX() sobreescribirá esto en cada llamada, pero es buena práctica inicializar.
    motorX2.setDirection(false);

    LCD::scan(LCD_SDA_NUM, LCD_SCL_NUM);
    if (!lcd.setup(LCD_SDA_NUM, LCD_SCL_NUM, LCD_I2C_ADDR))
        printf("[WARN] LCD no encontrado en 0x%02X — revisa el cableado\n", LCD_I2C_ADDR);

    btnCoinSim.init();
    btnStart.init();
    btnLeft.init();
    btnRight.init();
    /*btnUp.init();
    btnDown.init();
    */

    printf("[MASTER] Maquina de garra iniciada\n");

    State current_state = STATE_DEFAULT;
    State last_state    = STATE_DEFAULT;

    while (true) {
        // 1. Ejecutar la lógica del estado actual
        if (state_table[current_state].on_loop)
            current_state = state_table[current_state].on_loop();

        // 2. Detectar transición de estado
        if (current_state != last_state) {
            printf("[MASTER] Estado: %s\n", state_table[current_state].name);
            last_state   = current_state;
            is_new_state = true;
        } else {
            is_new_state = false;
        }
    }
}
