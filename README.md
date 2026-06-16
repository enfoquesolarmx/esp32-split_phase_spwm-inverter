<img src="/mcpwm_spwm_split_phase_00.png" width="600" alt="Split-phase a 20 kHz">

# Inversor Split-Phase de Onda Senoidal Pura — ESP32 (MCPWM)

Etapa de control para un inversor **split-phase** (fase dividida, 120/240 V)
basado en dos puentes H independientes, generada íntegramente por software
sobre el periférico MCPWM de un ESP32 clásico, programado a nivel de registro.

---

## 🔎 Alcance de este trabajo (léelo primero)

Este proyecto incluye **únicamente las señales lógicas** de control — es la
**base** sobre la que se construye un inversor, no un inversor completo.

**Lo que SÍ hace:**
- Genera la señalización SPWM split-phase (dos puentes H, fases a 180°).
- Permite ajustar la **frecuencia fundamental** de salida (Hz, p. ej. 50/60 Hz)
  mediante `freqMod`.
- Permite ajustar la **frecuencia de portadora** del PWM (p. ej. 20/25 kHz)
  mediante `freqCarr`, de forma paramétrica (todo escala solo).

**Lo que NO incluye (y hay que añadir para un inversor real):**
- **Ningún módulo de realimentación (feedback):** no hay lazo cerrado de
  voltaje ni de corriente. La salida es a lazo abierto.
- **Ningún ajuste dinámico de la modulación:** el índice de modulación
  (amplitud) es fijo; no se regula ante cambios de carga o de batería.
- Ninguna etapa de **potencia** ni de **protección** (sobrecorriente,
  sobretemperatura, arranque suave, etc.).

En otras palabras: esto produce la **forma de onda de referencia** correcta.
Convertirla en un inversor seguro y regulado requiere las etapas de potencia,
realimentación y protección, que **no** están aquí.

---

> **Propósito.** Ofrecer una referencia **abierta y validada** de la etapa de
> control de un inversor de onda senoidal pura, para quienes no pueden costear
> equipo comercial (Outback Radian, Schneider XW). El objetivo no es solo dar
> código que funcione, sino código que se pueda **entender y aprender**.

---

## ⚠️ Aviso de seguridad y alcance

Este repositorio cubre **la etapa de control**: las señales lógicas que
manejan los gate drivers. **No es un inversor terminado.** Falta la etapa de
potencia y, sobre todo, las **protecciones**, que son lo más importante para
que sea seguro.

- Topología de **240 V** y alta energía: un error puede causar incendio o
  descarga.
- **Nunca** conectes carga sin validar primero con osciloscopio + voltímetro
  + sonda de corriente (ver "Validación").
- El microcontrolador es la parte fácil. La potencia, la regulación y la
  protección son el trabajo difícil y el que de verdad hace seguro el equipo.

---

## ¿Qué hace?

Genera dos fases senoidales desfasadas 180°:

```
L1(t) = +sin(t)   (puente H1)
L2(t) = -sin(t)   (puente H2)   ->  misma onda, signo opuesto
```

Cuando L1 está en su pico positivo, L2 está en su pico negativo, **al mismo
tiempo**. Entre las dos líneas se obtiene el doble de tensión
(`L1 - L2 = 2·sin(t)`), es decir 240 V de dos fases de 120 V, con neutro común.

Cada fase es un **puente H completo** que alimenta su propio transformador; los
secundarios comparten el neutro N.

---

## Cómo se generan las señales

Cada puente usa dos tipos de señal coordinadas:

| Señal | Frecuencia | Qué define | Origen |
|-------|-----------|------------|--------|
| **PWM** | 20–25 kHz | la **magnitud** (ancho de pulso) | MCPWM + dead-time |
| **Rama** | 60 Hz | el **signo** (qué diagonal conduce) | GPIO atómico |

La onda de salida = **magnitud (PWM) × signo (rama)**. El PWM dice "qué tan
fuerte", la rama dice "en qué dirección".

Los dos puentes operan **en paralelo** (no se turnan) gracias a la
**sincronización de timers** (phase-lock): el timer 1 cuenta en fase con el
timer 0 vía `mcpwm_sync_configure`.

---

## Mapa de canales (analizador lógico)

| Canal | Pin | Señal |
|-------|-----|-------|
| D0 | 23 | H1 PWM (HO1) |
| D1 | 22 | H1 PWM complementario (LO1) |
| D2 | 21 | H1 rama positiva (HO2) |
| D3 | 19 | H1 rama negativa (LO2) |
| D4 | 26 | H2 PWM (HO3) |
| D5 | 25 | H2 PWM complementario (LO3) |
| D6 | 33 | H2 rama positiva (HO4) |
| D7 | 32 | H2 rama negativa (LO4) |

**Qué buscar:** el PWM de H1 (D0/D1) y el de H2 (D4/D5) modulan en paralelo;
las ramas (D2/D3 vs D6/D7) van en oposición. Eso confirma `L2 = -L1`.

---

## Recorrido de depuración (las lecciones)

Este proyecto se depuró midiendo, no suponiendo. Cada problema dejó una
lección que vale más que el código:

1. **Pulso ancho.** Causa raíz: el shadow del comparador en modo *inmediato*
   permitía que el duty entrara a mitad de ciclo y se perdiera el match.
   Solución: shadow en **TEP** (se actualiza en el pico del contador,
   determinista). Aplicado a ambos operadores.

2. **Base de tiempo.** No confiar en constantes hardcodeadas
   (`62.5e-9`): derivar el tick de la fórmula `(1/APB)·prescaler`. El
   registro guarda `prescaler-1`; al leerlo, reconstruir con `valor+1`.

3. **Segundo puente caótico / se turnaba.** Causa: los dos operadores en
   timers distintos sin sincronizar, o forzados al mismo timer rompiendo su
   generador. Solución correcta: cada operador en su timer + **sincronización
   phase-lock** (como el chip espera).

4. **Medio ciclo.** El modo complementario solo modula bien un semiciclo si
   se usa **un** comparador. Hay que escribir siempre en el comparador
   principal y dejar que la **rama** dé el signo (no cambiar de comparador).

5. **Pulso solitario.** El duty de cruce del puente espejo va **intercambiado**
   respecto al otro (porque `L2 = -L1`). Y el residuo del dead-time se absorbe
   con una **banda de cruce** (`ZERO_BAND_SAMPLES`).

6. **Número mágico.** El offset del semiciclo negativo debe ser **paramétrico**
   (`tmrRegVal + sineVal`), no un valor fijo (ej. 380) atado a una sola
   frecuencia. Así el código escala a 20/25/30 kHz sin tocar nada.

> Lección transversal: **cuando un defecto sobrevive a reescribir el software,
> mide el hardware.** Y cambia **una variable a la vez**, validando entre cada
> cambio.

---

## Validación eléctrica (antes de energizar)

1. **Osciloscopio en señales lógicas (sin potencia):** confirmar 180° entre
   H1 y H2.
2. **Con transformadores y carga mínima**, medir:
   - `L1-N ≈ 120 V`, `L2-N ≈ 120 V`
   - **`L1-L2`** → la prueba definitiva:
     - `≈ 240 V` → split-phase **correcto**.
     - `≈ 0 V` → fases **en paralelo**: invertir la polaridad de un secundario.
3. **Sonda de corriente** para confirmar que no hay picos de shoot-through.

**Nunca** conectar carga de 240 V sin confirmar antes `L1-L2` con el voltímetro.

---

## Requisitos

- ESP32 clásico (WROOM).
- Núcleo **Arduino-ESP32 2.0.x** (por la API `mcpwm_sync_configure`).
- Analizador lógico para validar las señales (recomendado).

## Comandos

- `r` por monitor serial: lee y valida los registros del MCPWM contra el
  silicio (frecuencia real, shadow en TEP de ambos operadores, etc.).

---
## ¿Por qué el periférico MCPWM (y no ledc/PWM genérico)?

La generación se hace sobre el **MCPWM** (Motor Control PWM) del ESP32, no
sobre el PWM genérico (`ledc`), por dos características que un inversor
necesita y el PWM genérico no ofrece de forma nativa:

- **Dead-time por hardware.** El MCPWM inserta un tiempo muerto configurable
  entre las salidas complementarias de cada rama (aquí 500 ns). Esto protege
  el puente H contra *shoot-through* (que el lado alto y el bajo conduzcan a
  la vez y cortocircuiten la batería). Hacerlo por software sería frágil y de
  latencia no determinista; en hardware es exacto y automático.

- **Modulación fina y determinista.** El MCPWM compara el contador del timer
  contra un umbral (comparador) con resolución de tick (62.5 ns), y actualiza
  el duty mediante un registro *shadow* en un punto determinista del ciclo
  (TEP, el pico del contador). Esto permite una envolvente senoidal suave y
  sin glitches — clave para una onda de baja distorsión.

Además, el MCPWM ofrece **sincronización de timers** (phase-lock), que es lo
que permite que los dos puentes operen en paralelo y en fase opuesta sin
turnarse. Con el PWM genérico no habría forma limpia de coordinarlos.

---

<img src="/mcpwm_spwm_split_phase_01.png" width="600" alt="Split-phase a 20 kHz">


## Estado y siguientes pasos

**Hecho (validado en analizador lógico):** generación split-phase, dos puentes
en paralelo y opuestos, paramétrico en frecuencia (20–25 kHz), pulso ancho y
pulso solitario resueltos, protección de cruce en ambos puentes.

**Pendiente (etapa de potencia, lo crítico):** validación en osciloscopio,
medición de `L1-L2` con transformadores, protección de sobrecorriente con
sensado físico, sobretemperatura, arranque suave, realimentación de voltaje
(lazo cerrado), y análisis formal de modos de fallo antes de cualquier uso
real.

---

## Licencia

Proyecto abierto. Úsalo, apréndelo, mejóralo y compártelo — especialmente con
quien lo necesite. Si lo llevas a potencia, hazlo con las protecciones y la
cautela que merece la energía con la que trabaja.
