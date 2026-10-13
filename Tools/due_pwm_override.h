// Force-included by PlatformIO for the Arduino Due environments (see platformio.ini).
// Overrides the PWM and timer-counter frequencies of the Due core, which the
// firmware needs at 50 kHz. This replaces the manual edit of variant.h that is
// required when building with the Arduino IDE.
//
// NOTE: PlatformIO does not track this file as a dependency. After editing it,
// run `pio run -t clean` before building.

#include "variant.h"

#undef PWM_FREQUENCY
#define PWM_FREQUENCY 50000
#undef TC_FREQUENCY
#define TC_FREQUENCY 50000
