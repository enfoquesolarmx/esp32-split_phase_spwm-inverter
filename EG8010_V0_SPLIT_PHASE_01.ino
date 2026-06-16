/*==============================================================================
  INVERSOR SPLIT-PHASE (2 PUENTES H) — ESP32 (MCPWM)
  ------------------------------------------------------------------------------
  Genera DOS fases senoidales (L1 y L2) desfasadas 180°, cada una de un puente
  H independiente con su propio transformador. Entre L1 y L2 se obtiene el
  DOBLE de voltaje (split-phase: 2x120V = 240V), con neutro común N.

  PROPÓSITO DEL PROYECTO:
    Ofrecer una referencia ABIERTA y validada de un inversor de onda senoidal
    pura para quienes no pueden comprar equipo comercial (Outback Radian,
    Schneider XW). Diseño resiliente: dos puentes independientes -> si uno
    falla, el otro puede seguir (degradación elegante).

 =============================================================================
  ¿QUÉ ES SPLIT-PHASE Y CÓMO LO LOGRAMOS?
  ------------------------------------------------------------------------------
    L1(t) = +sin(t)   (puente H1)
    L2(t) = -sin(t)   (puente H2)  <- MISMA onda, signo OPUESTO

    En cada instante L2 es el espejo de L1. Cuando L1 está en su pico
    positivo (+170V), L2 está en su pico negativo (-170V), AL MISMO TIEMPO.
    La tensión entre líneas es la diferencia:  L1 - L2 = 2*sin(t) -> 340Vpico.

    Cada "fase" es un PUENTE H COMPLETO. La salida AC de cada puente alimenta
    el primario de SU transformador. Los secundarios comparten el neutro N.

 =============================================================================
  CÓMO SE GENERAN LAS SEÑALES (lo que verás en el analizador):
  ------------------------------------------------------------------------------
    Cada puente H usa DOS tipos de señal coordinadas:

    1) PWM de alta frecuencia (20 kHz) -> define la MAGNITUD (ancho de pulso).
       Sale del MCPWM con dead-time complementario. Modula el ancho según el
       seno: pulsos anchos en el pico, angostos cerca del cruce.

    2) Conmutación de RAMA a baja frecuencia (60 Hz) -> define el SIGNO.
       Decide qué DIAGONAL del puente conduce (positivo vs negativo).
       Conmuta una vez por semiciclo, en el cruce por cero.

    La onda senoidal de salida = MAGNITUD (PWM) x SIGNO (rama). El PWM dice
    "qué tan fuerte", la rama dice "en qué dirección".

 =============================================================================
  CÓMO LOS DOS PUENTES OPERAN EN PARALELO Y OPUESTOS:
  ------------------------------------------------------------------------------
    - H1 usa el OPERADOR 0 (timer 0). H2 usa el OPERADOR 1 (timer 1).
    - Los dos timers se SINCRONIZAN (phase-lock) con mcpwm_sync_configure:
      cuentan exactamente en fase -> los puentes operan EN PARALELO, no se
      turnan. Esta fue la clave para que H2 no saliera caótico ni a medias.
    - H2 usa -sineVal y su rama va INVERTIDA respecto a H1 -> fase opuesta.
    - AMBOS operadores escriben SIEMPRE en su comparador principal (H1->0x40,
      H2->0x78). El signo lo da la rama, NO el cambio de comparador. (Escribir
      en dos comparadores hacía que el modo complementario solo diera medio
      ciclo: lección aprendida.)

 =============================================================================
  CAUSA RAÍZ DEL "PULSO ANCHO" (resuelta, heredada del inversor de 1 fase):
    El shadow del comparador (0x3FF5E03C) en modo INMEDIATO permitía que un
    nuevo duty entrara a mitad de ciclo, perdiendo el match -> pulso ancho.
    SOLUCIÓN: shadow en TEP (2) -> el valor se transfiere en el PICO del
    contador, determinista. Aplicado a AMBOS operadores (0x3C y 0x74).

  CORRECCIONES VALIDADAS:
   [A] Base de tiempo por silicio: prescaler 5 -> tick 62.5 ns -> 20 kHz.
   [B] Conmutación de rama por registro atómico (out_w1ts/tc), no digitalWrite.
   [C] Orden clear->set: apaga la rama opuesta ANTES de abrir la correcta.
   [D] Cruce por ÍNDICE (i==0, i==N/2), robusto ante saltos del seno.
   [E] Shadow en TEP en ambos operadores.
   [F] Sincronización de timers 0 y 1 (phase-lock) para paralelismo.
   [G] Autotest 'r' por serial: valida registros contra el silicio.

 =============================================================================
  MAPA DE CANALES PARA EL ANALIZADOR LÓGICO:
  ------------------------------------------------------------------------------
    PUENTE H1 (operador 0, fase L1 = +sin):
      D0 = HO1 (23) PWM positivo        D1 = LO1 (22) PWM complementario
      D2 = HO2 (21) rama positiva       D3 = LO2 (19) rama negativa
    PUENTE H2 (operador 1, fase L2 = -sin, OPUESTA):
      D4 = HO3 (26) PWM positivo        D5 = LO3 (25) PWM complementario
      D6 = HO4 (33) rama positiva       D7 = LO4 (32) rama negativa

    QUÉ BUSCAR: el PWM de H1 (D0/D1) y el de H2 (D4/D5) modulan EN PARALELO
    (los dos a la vez, no turnándose). Las ramas de H1 (D2/D3) y H2 (D6/D7)
    van en OPOSICIÓN: cuando H1 está en su semiciclo positivo, H2 está en el
    negativo. Eso es el split-phase: L2 = -L1 en todo instante.

 =============================================================================
  VALIDACIÓN ELÉCTRICA (la prueba definitiva, con osciloscopio + voltímetro):
  ------------------------------------------------------------------------------
    1) Osciloscopio en señales lógicas (SIN potencia): confirmar 180° entre
       H1 y H2 antes de energizar.
    2) Con transformadores y carga mínima, medir:
         L1-N ~ 120V    L2-N ~ 120V    L1-L2 = ?
       *** L1-L2 ~ 240V -> split-phase CORRECTO.
       *** L1-L2 ~ 0V   -> fases EN PARALELO (en fase): invertir la polaridad
                           de UN secundario del transformador.
    NUNCA conectar carga de 240V sin confirmar antes L1-L2 con el voltímetro.

  ADVERTENCIA: topología de 240V, alta energía. Requiere protecciones
  (sobrecorriente, sobretemperatura), arranque suave, y validación con sonda
  de corriente. El micro es la parte fácil; la potencia es la crítica.

  Comandos serial:  'r' = leer/validar registros del MCPWM.
 =============================================================================
*/

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"

/*------------------------------------------------------------------------------
  PINES — cada puente H tiene 4 señales: 2 de PWM (magnitud) + 2 de rama (signo)
  ------------------------------------------------------------------------------
  HOx requiere bootstrap (lado alto); LOx referencia a GND (lado bajo).        */

/* ---- PUENTE H1 (fase L1 = +sin) : operador 0, timer 0 ---- */
const int HO1 = 23; // MCPWM0A — PWM lado alto  (magnitud, 20 kHz)
const int LO1 = 22; // MCPWM0B — PWM complementario (por dead-time)
const int HO2 = 21; // RAMA positiva (signo, 60 Hz): conduce en semiciclo +
const int LO2 = 19; // RAMA negativa (signo, 60 Hz): conduce en semiciclo -

/* ---- PUENTE H2 (fase L2 = -sin, OPUESTA a H1) : operador 1, timer 1 ---- */
const int HO3 = 26; // MCPWM1A — PWM lado alto H2 (magnitud, 20 kHz)
const int LO3 = 25; // MCPWM1B — PWM complementario H2
const int HO4 = 33; // RAMA positiva H2 (signo): banco ALTO de GPIO
const int LO4 = 32; // RAMA negativa H2 (signo): banco ALTO de GPIO

/* Máscaras de bit para conmutación atómica de rama.
   H1 (pines 19,21) -> banco bajo: GPIO.out_w1ts / out_w1tc.
   H2 (pines 32,33) -> banco ALTO: GPIO.out1_w1ts / out1_w1tc (bit - 32).     */
#define HO2_BIT (1 << 21)
#define LO2_BIT (1 << 19)
#define HO4_BIT1 (1 << (33-32))   // HO4=GPIO33 en banco alto
#define LO4_BIT1 (1 << (32-32))   // LO4=GPIO32 en banco alto

/*------------------------------------------------------------------------------
  REGISTROS MCPWM (ESP32 clásico). Comparador = umbral del duty (ancho de PWM).
  ------------------------------------------------------------------------------
  Cada operador tiene 2 comparadores (A y B). Usamos SIEMPRE el principal de
  cada operador (H1->CMP0A, H2->CMP1A). El signo lo da la rama, no el cambio
  de comparador.                                                              */
#define MCPWM_CMPR0_REG    0x3FF5E040  // CMP0A — duty del H1 (operador 0)
#define MCPWM_CMPR1_REG    0x3FF5E078  // CMP1A — duty del H2 (operador 1)
#define MCPWM_CMPR1B_REG   0x3FF5E07C  // CMP1B — (no usado; ref.)
#define MCPWM_INT_CLR_REG  0x3FF5E11C  // limpiar flag de interrupción
#define MCPWM_CLK_CFG      0x3FF5E000  // prescaler de grupo
#define MCPWM_TIMER0_CFG0  0x3FF5E004  // prescaler + periodo del timer 0
#define MCPWM_GEN0_STMP    0x3FF5E03C  // shadow update H1 (operador 0)
#define MCPWM_GEN1_STMP    0x3FF5E074  // shadow update H2 (operador 1)
#define MCPWM_TIMERSEL     0x3FF5E0FC  // operador<->timer (no usado: usamos sync)
#define MCPWM_INT_ENA      0x3FF5E110  // habilitación de interrupción

/*------------------------------------------------------------------------------
  PARÁMETROS DE SEÑAL
  ----------------------------------------------------------------------------*/
const float freqCarr = 20000.0;   // portadora (Hz)
float       freqMod   = 60.0;     // fundamental (Hz)

/* [A] Base de tiempo validada por silicio (APB 80 MHz, tick base 12.5 ns).
   prescaler=5 -> en el registro se escribe (5-1)=4 -> tick (4+1)*12.5=62.5ns.
   Con 62.5 ns, tmrRegVal=399 corresponde a 20 kHz reales (modo up-down).    */
const int   prescaler    = 5;
const float timer_clk_hz = 80e6;
const float tick_ns      = (1.0 / timer_clk_hz) * prescaler * 1e9;   // 62.5 ns

const int   tmrRegVal = int(((((1/freqCarr)/2)/(tick_ns*1e-9)) - 1) + 0.5); // 399
float real_freqCarr   = 1.0 / ((tmrRegVal + 1) * tick_ns * 1e-9 * 2);
int   sampleNum       = int(real_freqCarr / freqMod);

int   amplitude = int(0.9 * tmrRegVal);   // 90% del periodo (margen 10%)
float radVal    = 2 * PI / sampleNum;
volatile int i  = 0;

/* Máscaras de las CUATRO compuertas de cada puente (apagado total en cruce). */
#define HO1_BIT (1 << 23)
#define LO1_BIT (1 << 22)

/*------------------------------------------------------------------------------
  BANDA DE CRUCE (ZERO_BAND_SAMPLES) — guarda contra el pulso del dead-time
  ------------------------------------------------------------------------------
  En el cruce, el dead-time del MCPWM puede dejar un pulso residual (~500ns,
  del ancho del dead-time). Esta banda apaga TODOS los MOSFETs de AMBOS puentes
  durante varias muestras alrededor del cruce -> el pulso cae dentro de la zona
  de apagado total y NO se genera. Además refuerza la protección del cambio de
  diagonal (turn-off + reverse recovery).

  Cada muestra = 1 periodo de portadora = 50µs (a 20kHz). El ancho total es:
     ZERO_BAND_SAMPLES=1 -> ~50µs a cada lado del cruce
     ZERO_BAND_SAMPLES=2 -> ~100µs a cada lado
  Empezar en 1 (suficiente para cubrir el pulso de 500ns). Subir si el pulso
  persiste; bajar si introduce demasiada distorsión cerca del cruce. Validar
  con osciloscopio + sonda de corriente.                                       */
const int ZERO_BAND_SAMPLES = 1;

/*------------------------------------------------------------------------------
  BANDA MUERTA VARIABLE estilo EG8010 — YA PRESENTE EN EL SPWM NATURAL
  ------------------------------------------------------------------------------
  El EG8010 usa una banda muerta MÁXIMA en el cruce y MÍNIMA en el pico
  (patente SPWM). Eso equivale a duty pequeño en el cruce y grande en el pico.
  La envolvente senoidal de este código YA produce exactamente eso:

     zona   | duty (sineVal) | banda muerta (tmrRegVal - duty)
     -------+----------------+--------------------------------
     cruce  |   6, 13, 20    |   393, 386, 379   (banda GRANDE)
     pico   |   ~358         |   ~41             (banda MÍNIMA)

  Por tanto NO se fuerza ninguna banda extra: el comportamiento del chip de
  referencia se obtiene de forma natural y suave por la propia modulación
  senoidal. El cambio de diagonal (signo) ocurre en el cruce, donde el duty
  ya está en su mínimo -> transición de baja energía, sin shoot-through.

  La protección entre el high/low de cada rama la da el dead-time de hardware
  (500 ns). La "banda muerta variable" hacia el cruce la da el seno. Juntas
  replican el esquema del EG8010 sin lógica adicional.
  ----------------------------------------------------------------------------*/


/*==============================================================================
  ISR del MCPWM (IRAM_ATTR). Disparada en TEZ (timer = 0).
  ------------------------------------------------------------------------------
  [D] Árbol mutuamente excluyente -> una sola escritura del duty por muestra.
  [B][C] Conmutación de rama por registro atómico, orden clear->set.
==============================================================================*/
void IRAM_ATTR MCPWM_ISR(void*) {
  float sineValue;
  int   sineVal;

  WRITE_PERI_REG(MCPWM_INT_CLR_REG, BIT(3));   // limpiar flag TEZ

  sineValue = amplitude * sin(radVal * i);
  sineVal   = int(sineValue);

  /* ====================================================================
     GUARDA DE BANDA DE CRUCE (quirúrgica) — cubre el pulso del dead-time.
     Si estamos a ZERO_BAND_SAMPLES muestras de un cruce (i==0 o i==N/2),
     apaga TODOS los MOSFETs de AMBOS puentes y sale de la ISR antes de la
     lógica de semiciclos. El pulso residual del dead-time cae aquí y NO se
     genera. Fuera de la banda, todo funciona igual que sin esta guarda.
     ==================================================================== */
  int dist0   = (i < (sampleNum - i)) ? i : (sampleNum - i);   // dist. a 0/360°
  int distMid = abs(i - sampleNum/2);                          // dist. a 180°
  if (dist0 <= ZERO_BAND_SAMPLES || distMid <= ZERO_BAND_SAMPLES) {
    /* Apaga las 4 compuertas del H1 (banco bajo) y duty 0. */
    GPIO.out_w1tc = HO1_BIT | LO1_BIT | HO2_BIT | LO2_BIT;
    WRITE_PERI_REG(MCPWM_CMPR0_REG, 0);
    /* Apaga las 2 ramas del H2 (banco alto) y duty 0. */
    GPIO.out1_w1tc.val = HO4_BIT1 | LO4_BIT1;
    WRITE_PERI_REG(MCPWM_CMPR1_REG, 0);
    /* Avanza el índice y sale: no ejecuta la lógica de semiciclos. */
    i++;
    if (i > sampleNum) i = 0;
    return;
  }

  /* ====================================================================
     PUENTE H1 — fase L1 = +sin.  Operador 0.
     El PWM (CMP0A) pone la MAGNITUD; la rama (HO2/LO2) pone el SIGNO.
     ==================================================================== */
  if (sineVal > 0) {                              // --- SEMICICLO POSITIVO ---
    GPIO.out_w1tc = LO2_BIT;                      // 1º apaga rama negativa (seguridad)
    GPIO.out_w1ts = HO2_BIT;                      // 2º abre rama positiva (conduce diagonal +)
    WRITE_PERI_REG(MCPWM_CMPR0_REG, sineVal);     // duty = magnitud del seno
  }

  if (sineVal < 0) {                              // --- SEMICICLO NEGATIVO ---
    GPIO.out_w1tc = HO2_BIT;                      // 1º apaga rama positiva
    GPIO.out_w1ts = LO2_BIT;                      // 2º abre rama negativa (conduce diagonal -)
    WRITE_PERI_REG(MCPWM_CMPR0_REG, tmrRegVal + sineVal); // duty espejado al rango positivo
  }

  /* NOTA: con ZERO_BAND_SAMPLES activo, los cruces (i==0, i==N/2) ya los
     cubre la GUARDA DE BANDA al inicio de la ISR (que apaga todo y hace
     return). Estos bloques quedan como respaldo / referencia, pero en la
     práctica no se alcanzan mientras ZERO_BAND_SAMPLES >= 0.                 */
  if (i == 0) {                                   // --- CRUCE 0° (cubierto por guarda) ---
    GPIO.out_w1tc = LO2_BIT;
    WRITE_PERI_REG(MCPWM_CMPR0_REG, 0);
  }

  if (i == (sampleNum/2)) {                       // --- CRUCE 180° (cubierto por guarda) ---
    WRITE_PERI_REG(MCPWM_CMPR0_REG, tmrRegVal);
    GPIO.out_w1tc = HO2_BIT;
  }

  /* ====================================================================
     PUENTE H2 — fase L2 = -sin (OPUESTA a H1).  Operador 1.
     Estructura IDÉNTICA a H1, pero con sineVal2 = -sineVal y rama
     invertida -> cuando H1 va positivo, H2 va negativo, AL MISMO TIEMPO.
     Escribe SIEMPRE en CMP1A (0x78); el signo lo da la rama H2 (HO4/LO4).
     Rama en banco ALTO de GPIO (out1_) porque usa pines 32/33.
     ==================================================================== */
  int sineVal2 = -sineVal;                        // L2 es el espejo de L1

  if (sineVal2 > 0) {                             // --- H2 POSITIVO (H1 negativo) ---
    GPIO.out1_w1tc.val = LO4_BIT1;                // 1º apaga rama negativa H2
    GPIO.out1_w1ts.val = HO4_BIT1;                // 2º abre rama positiva H2
    WRITE_PERI_REG(MCPWM_CMPR1_REG, sineVal2);    // duty H2 (siempre CMP1A)
  }

  if (sineVal2 < 0) {                             // --- H2 NEGATIVO (H1 positivo) ---
    GPIO.out1_w1tc.val = HO4_BIT1;                // 1º apaga rama positiva H2
    GPIO.out1_w1ts.val = LO4_BIT1;                // 2º abre rama negativa H2
    WRITE_PERI_REG(MCPWM_CMPR1_REG, tmrRegVal + sineVal2);
  }

  if (i == 0) {                                   // --- CRUCE 0° H2 (protección) ---
    /* H2 es ESPEJO de H1: aquí H2 entra a su semiciclo NEGATIVO (380+sv2),
       que con sv2->0 corresponde a duty 380 (NO 0). Poner 0 aquí causaba
       el pulso solitario. Apaga ambas ramas (banda 50µs) + duty 380.         */
    GPIO.out1_w1tc.val = HO4_BIT1 | LO4_BIT1;     // apaga las DOS ramas H2 (protección)
    WRITE_PERI_REG(MCPWM_CMPR1_REG, tmrRegVal);
  }

  if (i == (sampleNum/2)) {                       // --- CRUCE 180° H2 (protección) ---
    /* Aquí H2 entra a su semiciclo POSITIVO (sv2), que con sv2->0 es duty 0. */
    GPIO.out1_w1tc.val = HO4_BIT1 | LO4_BIT1;     // apaga las DOS ramas H2 (protección)
    WRITE_PERI_REG(MCPWM_CMPR1_REG, 0);
  }

  i++;
  if (i > sampleNum) i = 0;
}


/*------------------------------------------------------------------------------
  [F] Lectura/validación de registros contra el silicio (comando 'r').
  ----------------------------------------------------------------------------*/
void leerRegistros() {
  uint32_t tcfg0 = READ_PERI_REG(MCPWM_TIMER0_CFG0);
  uint32_t stmp  = READ_PERI_REG(MCPWM_GEN0_STMP);
  uint32_t presc = tcfg0 & 0xFF;
  uint32_t per   = (tcfg0 >> 8) & 0xFFFF;
  double   tick  = (presc + 1) * 12.5;                  // ns
  double   fpwm  = 1.0 / (2.0 * (per + 1) * tick * 1e-9);

  Serial.println("===== REGISTROS MCPWM (silicio) =====");
  Serial.print("APB             : "); Serial.print(rtc_clk_apb_freq_get()/1e6,3); Serial.println(" MHz");
  Serial.print("prescaler timer : "); Serial.print(presc); Serial.print(" (tick "); Serial.print(tick,1); Serial.println(" ns)");
  Serial.print("periodo timer   : "); Serial.println(per);
  Serial.print("shadow update   : "); Serial.print(stmp);
  Serial.println(stmp == 2 ? "  (TEP, correcto)" : "  (NO es TEP!)");

  /* Validar también el shadow del H2 (operador 1). Si 0x74 es la dirección
     correcta, debe leer 2 (TEP). Si lee otra cosa, el offset está mal y por
     eso el PWM de H2 sale caótico.                                           */
  uint32_t stmp1 = READ_PERI_REG(MCPWM_GEN1_STMP);
  Serial.print("shadow H2 (0x74): "); Serial.print(stmp1);
  Serial.println(stmp1 == 2 ? "  (TEP, correcto)" : "  (NO es TEP! -> PWM H2 caotico)");
  Serial.print("freq PWM calc   : "); Serial.print(fpwm/1000.0,4); Serial.println(" kHz");
  Serial.print("sampleNum       : "); Serial.println(sampleNum);
  Serial.println("=====================================");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(LO2, OUTPUT);
  pinMode(HO2, OUTPUT);
  pinMode(LO4, OUTPUT);   // rama H2 (banco alto)
  pinMode(HO4, OUTPUT);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, HO1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LO1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, HO3);   // PWM H2
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, LO3);

  mcpwm_config_t pwm_config;
  pwm_config.frequency    = real_freqCarr * 2;   // up-down -> *2
  pwm_config.cmpr_a       = 0;
  pwm_config.cmpr_b       = 0;
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
  pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   // configura generador op1 (H2)

  /* Prescaler de grupo = 0. */
  WRITE_PERI_REG(MCPWM_CLK_CFG, 0);

  /* [A] Periodo + prescaler del timer en el registro (base de tiempo real). */
  uint32_t reg_val = ((prescaler - 1) & 0xFF) | ((tmrRegVal & 0xFFFF) << 8);
  WRITE_PERI_REG(MCPWM_TIMER0_CFG0, reg_val);
  WRITE_PERI_REG(0x3FF5E014, reg_val);   // timer 1 mismo periodo (para el sync)

  /* [E] *** SHADOW EN TEP (2) — LA CAUSA RAÍZ DEL FENÓMENO. *** */
  WRITE_PERI_REG(MCPWM_GEN0_STMP, 2);
  WRITE_PERI_REG(MCPWM_GEN1_STMP, 2);   // shadow TEP también para H2

  /* *** SINCRONIZACIÓN DE TIMERS (método nuevo) ***
     Cada operador queda en SU propio timer (op0->t0, op1->t1), como el chip
     espera. Para que NO se turnen, sincronizamos el timer 1 con el timer 0:
     ambos cuentan PHASE-LOCKED (en fase). Así el generador del op1 es
     coherente con su timer 1, y los dos puentes operan EN PARALELO.
     NO se usa TIMERSEL (que rompía el generador del op1).                    */

  esp_intr_alloc(ETS_PWM0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, MCPWM_ISR, NULL, NULL);

  /* Dead-time 80 ticks * 6.25 ns = 500 ns, en ambos operadores. */
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 80, 80);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 80, 80);

  /* Sincroniza el TIMER 1 con el TIMER 0 en FASE 0 (cuentan juntos).
     API de core 2.0.x (ESP-IDF 4.4): mcpwm_sync_configure con estructura.
     timer_val=0 -> fase 0, timer 1 en fase con timer 0.                      */
  mcpwm_sync_config_t sync_conf;
  sync_conf.sync_sig        = MCPWM_SELECT_TIMER0_SYNC;  // sync que emite timer 0
  sync_conf.timer_val       = 0;                          // fase 0 (en fase)
  sync_conf.count_direction = MCPWM_TIMER_DIRECTION_UP;
  mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_conf);

  /* El timer 0 debe EMITIR su señal de sync (en TEZ, al llegar a cero) para
     que el timer 1 la reciba y se reinicie en fase.                          */
  mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0,
                              MCPWM_SWSYNC_SOURCE_TEZ);


  WRITE_PERI_REG(MCPWM_INT_ENA, BIT(3));   // habilita interrupción TEZ

  Serial.println("Inversor DEFINITIVO listo. Comando 'r' = validar registros.");
  leerRegistros();
}


void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n');
    c.trim();
    if (c == "r" || c == "R") leerRegistros();
  }
}
