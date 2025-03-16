/* 
** By Turi Scandurra
** https://turiscandurra.com/circuits
**
** I2S driver based on the work of Ricardo Massaro
** https://github.com/moefh/
**/

/*
 * Prueba n3:
 * - Usar timers como osciladores
 *
 */

#include "pico/stdlib.h"
#include <synth_sequencer.h>
#include "hardware/pwm.h"

#include <math.h>


  /*   MIDI over UART   */

#define UART_ID uart0
#define BAUD_RATE 31250
#define LED_PIN 25
#define MIDI_CC_CANTIDAD 8

volatile uint8_t MIDI_byte = 0, MIDI_msgFlag = 0;
volatile uint8_t MIDI_byte0 = 0x90, MIDI_byte1 = 0, MIDI_byte2 = 0;

volatile uint8_t MIDI_cc[MIDI_CC_CANTIDAD];

void uart_irq_handler() {
  uint8_t byte = uart_getc(UART_ID);  // Leer el byte recibido
  uint8_t byte_channel = byte & 0xf0; // byte sin el canal MIDI

  if(byte_channel == 0xf0) return;    // si es un mensaje de sistema lo omito (start, stop, beat)
  if(byte > 0x7f) MIDI_byte = 0;      // si es un note on o note off reinicio la máquina de estados
  

  switch(MIDI_byte){
      case 0:
          MIDI_byte0 = byte_channel;
          MIDI_byte = 1;
          return;
      
      case 1:
          MIDI_byte1 = byte;
          MIDI_byte = 2;
          return;

      case 2:
          MIDI_byte2 = byte;
          MIDI_byte = 1; // running-mode

          MIDI_msgFlag = 1;
          return;
  }

  // si llegó hasta acá es porque hubo un error
  MIDI_byte = 1;


}

void setup_uart() {
  uart_init(UART_ID, BAUD_RATE);  // Configurar el UART con la velocidad de 31,250 baudios
  gpio_set_function(1, GPIO_FUNC_UART);  // Configurar el pin 1 como RX

  // Habilitar interrupciones en el UART para recibir datos
  uart_set_irq_enables(UART_ID, true, false);  // Habilitar interrupciones cuando hay datos disponibles

  // Configurar la interrupción
  irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);  // Asociar la interrupción con el manejador
  irq_set_enabled(UART0_IRQ, true);  // Habilitar la interrupción en el sistema
}




  /*   Configuración del I2S   */

#define SAMPLE_RATE 44100

// I2S_SAMPLE_NUM = 1
#define I2S_DATA_PIN             28 // -> I2S DIN
#define I2S_CLOCK_PIN_BASE       26 // -> I2S BCK
// The third required connection is GPIO 27 -> I2S LRCK (BCK+1)

static const struct sound_i2s_config sound_config = {
  .pin_sda         = I2S_DATA_PIN,
  .pin_scl         = I2S_CLOCK_PIN_BASE,
  .pin_ws          = I2S_CLOCK_PIN_BASE + 1,
  .sample_rate     = SAMPLE_RATE,
  .bits_per_sample = 16,
  .pio_num         = 0, // 0 for pio0, 1 for pio1
};



  /*   Timers de PWM   */

  #define TIMERS_CANTIDAD 12

  void pwm_setup(){
  
    pwm_config config = pwm_get_default_config();
  
  
    // habilito los 12 PWMs y establezco el wrap en 0xffff
    for(uint8_t k = 0; k < TIMERS_CANTIDAD; k++){
      pwm_set_wrap(k, 0x7fff);
      pwm_set_enabled(k, true);
      pwm_set_clkdiv(k, 1);
    }
  
    // podría subir la octava de oscilador cambiando el wrap a 2^15 ó 2^14
    // para modificar la frecuencia cambio el prescaler con:
    // pwm_set_clkdiv(slice, div), siendo div un float >= 1.f y < 256.f
    // y obtengo el valor actual del timer con:
    // pwm_get_counter(slice);
  
  }


  /*   Manejo de voces   */

#define VOCES_CANTIDAD 6

volatile bool voice_on[VOCES_CANTIDAD];
volatile uint8_t voice_note[VOCES_CANTIDAD];
//volatile uint8_t voice_order[VOCES_CANTIDAD];
  
volatile int16_t sample_anterior;

enum {SAW, SQR, TRI} forma_de_onda;

// Filtro LPF
volatile float lpf_a[2], lpf_b[2];
volatile float lpf_wn, lpf_Q;
volatile int16_t in_anterior[2], out_anterior[2];

void lpf_recalcular(){
  float alpha = sin(lpf_wn)/(2*lpf_Q);
  float cos_wn = cos(lpf_wn);
  
  lpf_b[0] = (1 - cos_wn) / (2 * (1 + alpha));
  lpf_b[1] = 2*lpf_b[0];
  lpf_a[0] = -2 * cos_wn / (1 + alpha);
  lpf_a[1] = (1 - alpha) / (1 + alpha);
}

int16_t nuevo_sample(){
  int16_t sample = 0;
    

  if(forma_de_onda == SQR){
    for(uint8_t k = 0; k < VOCES_CANTIDAD; k++){
      if(voice_on[k]){
        
        sample += (pwm_get_counter(2*k) > 0x3fff ? (1 << 12) : 0) + (pwm_get_counter(2*k + 1) > 0x3fff ? (1 << 12) : 0) - (1 << 12);
      
      }
    }
  } else if(forma_de_onda == TRI) {
    for(uint8_t k = 0; k < VOCES_CANTIDAD; k++){
      if(voice_on[k]){
        
        sample += (pwm_get_counter(2*k) >> 3) + (pwm_get_counter(2*k + 1) >> 3) - (1 << 12);
      
      }
    }
  } else {
    for(uint8_t k = 0; k < VOCES_CANTIDAD; k++){
      if(voice_on[k]){
        
        sample += (pwm_get_counter(2*k) >> 4) + (pwm_get_counter(2*k + 1) >> 4) - (1 << 12);
      
      }
    }
  }

  int16_t sample_filtrado = lpf_b[0] * sample + lpf_b[1] * in_anterior[0] + lpf_b[0] * in_anterior[1] - lpf_a[0] * out_anterior[0] - lpf_a[1] * out_anterior[1];


  in_anterior[1] = in_anterior[0];
  in_anterior[0] = sample;
  out_anterior[1] = out_anterior[0];
  out_anterior[0] = sample_filtrado;

  return sample_filtrado;
}
  


  /*   Timer de audio   */

repeating_timer_t audio_timer;

bool audio_timer_callback(repeating_timer_t *timer) {
  
      static int16_t *last_buffer;
      int16_t *buffer = sound_i2s_get_next_buffer();
      if (buffer == NULL || buffer == last_buffer) { return true; }
      last_buffer = buffer;
  
  
      
        int16_t level = nuevo_sample();
  
        // Copy to I2S buffer
        *buffer++ = level;
        *buffer++ = level;
      
      return true;
}



volatile uint8_t voice_pointer = 0;
volatile uint8_t voices_note[4];

//const uint16_t MIDI_to_freq[] = {65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1046};
// empieza desde C2 = 36

const float MIDI_to_freq[] = {32.703, 34.648, 36.708, 38.891, 41.203, 43.653, 46.249, 48.999, 51.913, 55.000, 58.270, 61.735, 65.406, 69.295, 73.416, 77.781, 82.406, 87.307, 92.498, 97.998, 103.826, 109.999, 116.540, 123.470, 130.812, 138.590, 146.832, 155.563, 164.813, 174.613, 184.996, 195.997, 207.651, 219.999, 233.080, 246.940, 261.624, 277.181, 293.663, 311.125, 329.626, 349.226, 369.992, 391.993, 415.302, 439.997, 466.161, 493.880, 523.248, 554.362, 587.326, 622.250, 659.251, 698.452, 739.984, 783.986, 830.604, 879.995, 932.322, 987.761, 1046.496, 1108.724, 1174.652, 1244.500, 1318.502, 1396.905, 1479.969, 1567.972, 1661.209, 1759.989, 1864.644, 1975.521, 2092.992, 2217.448, 2349.304, 2489.001, 2637.005, 2793.809, 2959.938, 3135.945, 3322.418, 3519.979, 3729.288, 3951.043, 4185.984};
// empieza desde C1 = 24

int main() {

  stdio_init_all();

  setup_uart();
  pwm_setup();
  sound_i2s_init(&sound_config);


  

  // Inicializo el audio
  sound_i2s_playback_start();
  add_repeating_timer_us(10, audio_timer_callback, NULL, &audio_timer);



  while (true) {
    
    if(MIDI_msgFlag){
      if(MIDI_byte0 == 0x90){ // Note ON
        // frecuencia
        float freq = (SYS_CLK_HZ >> 15) / (float)(MIDI_to_freq[MIDI_byte1 - 24]);
        pwm_set_clkdiv(2*voice_pointer    , freq);
        pwm_set_clkdiv(2*voice_pointer + 1, freq);

        // control
        voice_note[voice_pointer] = MIDI_byte1;
        voice_on[voice_pointer] = true;

        voice_pointer = (voice_pointer + 1) % VOCES_CANTIDAD;
      }

      if(MIDI_byte0 == 0x80){ // Note OFF
        for(volatile uint8_t k = 0; k < VOCES_CANTIDAD; k++){
          if(voice_note[k] == MIDI_byte1){
            voice_on[k] = false;
            voice_note[k] = 0;
          }
        }
      }

      if(MIDI_byte0 == 0xB0){ // Control Change (CC)
        if(MIDI_byte1 < MIDI_CC_CANTIDAD)
          MIDI_cc[MIDI_byte1] = MIDI_byte2;

        if(MIDI_byte1 == 0){ // f cutoff
          lpf_wn = 6.28f * (MIDI_byte2 / 350.0f + .02f);
          lpf_recalcular();
        }

        if(MIDI_byte1 == 1){ // resonancia
          lpf_Q = .1 + MIDI_byte2 / 150.0f * 4.0f;
          lpf_recalcular();
        }

        if(MIDI_byte1 == 2){ // detune
          for(uint8_t i = 0; i < VOCES_CANTIDAD; i++){
            pwm_set_wrap(2*i    , (forma_de_onda == TRI ? 0x3fff : 0x7fff) + ((uint16_t)MIDI_byte2 << 1));
            pwm_set_wrap(2*i + 1, (forma_de_onda == TRI ? 0x3fff : 0x7fff) - ((uint16_t)MIDI_byte2 << 1));
          }
        }

        if(MIDI_byte1 == 3){ // forma de onda
          if(MIDI_byte2 < 41 && forma_de_onda != SAW){
            forma_de_onda = SAW;
            for(uint8_t i = 0; i < 12; i++){
              pwm_set_phase_correct(i, false);
            }
          }
          if(MIDI_byte2 >= 41 && MIDI_byte2 < 82 && forma_de_onda != SQR){
            forma_de_onda = SQR;
            for(uint8_t i = 0; i < 12; i++){
              pwm_set_phase_correct(i, false);
            }
          } else if(MIDI_byte2 >= 82 && forma_de_onda != TRI){
            forma_de_onda = TRI;
            for(uint8_t i = 0; i < 12; i++){
              pwm_set_phase_correct(i, true);
            }
          }
          
        }
      }
    

      MIDI_msgFlag = 0;
    }
    
  }

  

  return 0;
}



