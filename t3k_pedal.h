// T3K Pedal Header
// Pin definitions for the T3K Pedal board (Daisy Seed), derived from the
// nam-pedal-diy KiCad netlist.
//
// IMPORTANT: Sw and LED values are *logical* Daisy pin numbers (seed::Dn),
// as expected by DaisySeed::GetPin(), NOT physical 40-pin header numbers.
// Header pins 1..15 map to D0..D14 (header = D + 1); header pins 22..37 map
// to D15..D30 (header = D + 7).

namespace t3k_pedal {
class T3kPedal {
public:
  enum Sw {
    FOOTSWITCH = 23, // header 30 (PA4)  net /FTSW, momentary SPDT to GND
    // Rotary preset switch: one common pin per position, grounded
    // when that position is selected, floating otherwise.
    // NOTE: the switch's physical position order is the reverse of the PCB
    // net names (turning to position 1 grounds net /ROTARY4, etc.), so the
    // mapping here is mirrored to make position N select preset N / LED N.
    ROTARY_1 = 1, // header 2 (PC11)  net /ROTARY4  <- physical position 1
    ROTARY_2 = 2, // header 3 (PC10)  net /ROTARY3  <- physical position 2
    ROTARY_3 = 3, // header 4 (PC9)   net /ROTARY2  <- physical position 3
    ROTARY_4 = 4  // header 5 (PC8)   net /ROTARY1  <- physical position 4
  };

  // Index into the ADC channel array (see kKnobPins in NAMPedal.cpp),
  // not a Seed pin number. kKnobPins must list pins in this same order.
  // Order follows the PCB: ADC_0..ADC_5 (header 22..27, seed::A0..A5).
  enum Knob {
    INPUT_GAIN = 0,           // ADC_0  net /INPUT_VOL
    OUTPUT_VOLUME = 1,        // ADC_1  net /OUTPUT_VOL
    BASS = 2,                 // ADC_2  net /BASS
    MID = 3,                  // ADC_3  net /MID
    TREBLE = 4,               // ADC_4  net /TREBLE
    NOISE_GATE_THRESHOLD = 5, // ADC_5  net /NOISE_GATE
  };

  // LEDs: anode -> series resistor -> Daisy pin, cathode -> GND (active high).
  enum LED {
    LED_STATUS = 28,   // header 35 (PA2)  net /LED5, next to footswitch
    LED_PRESET_1 = 24, // header 31 (PA1)  net /LED1
    LED_PRESET_2 = 25, // header 32 (PA0)  net /LED2
    LED_PRESET_3 = 26, // header 33 (PD11) net /LED3
    LED_PRESET_4 = 27  // header 34 (PG9)  net /LED4
  };
};
} // namespace t3k_pedal
