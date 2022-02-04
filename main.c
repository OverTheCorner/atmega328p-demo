/*
/ Demo Code for atmega328p
/ Presents basic PORT IO, ADC, Timers/Counters, and Interrupts
/ 
*/

#include <avr\io.h>
#include <util\delay.h>
#include <avr\interrupt.h>

#define BIT_IS_SET(byte, bit) (byte & (1 << bit))			// these macros make checking if bits are set or clear easier and more readable
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))

void setup();
void loop();
void io_config();
void interrupt_config();
void pwm_config();
void adc_config();

const uint8_t BTTN1 = PD2;
const uint8_t BTTN2 = PD3;
const uint8_t BOARD_LED = PB5;
const uint8_t LED1 = PD5;
const uint8_t LED2 = PD6;
const uint8_t R_LED = PB1;
const uint8_t G_LED = PB2;
const uint8_t B_LED = PB3;

static const uint8_t red_val[5]=   {0xFF,0xCC,0x33,0x11,0x88}; 
static const uint8_t green_val[5]= {0xFF,0x66,0x22,0x77,0xCC};
static const uint8_t blue_val[5]=  {0xFF,0x77,0x88,0x33,0xEE};

volatile uint8_t adc_value_b=127; //brightness control
volatile uint8_t state = 0;
volatile int8_t dir = 0;
uint8_t period = 0;

int main(){
  setup();
  while(1){
    loop();
  }
  return 0;
}

void setup(){
  io_config();
  interrupt_config();
  pwm_config();
  adc_config();
}

void loop(){
  if (dir==1 && state==4){
      state=0;
    }
    else if (dir==-1 && state==0){
      state=4;
    }
    else{
      state+=dir;
    }

  OCR1A = (red_val[state] * adc_value_b) / 255 ;                         
  OCR1B = (green_val[state] * adc_value_b) / 255 ;             
  OCR2A = (blue_val[state] * adc_value_b) / 255 ;

  if (period){
    _delay_ms(400);
  }
  else{
    _delay_ms(800);
  }
}

void io_config(){
  //IO config
  DDRD &= ~( (1<<BTTN1) | (1<<BTTN2) ); //set PD2, PD3 to input
  DDRB |= ((1<<BOARD_LED) | (1<<R_LED) | (1<<G_LED) | (1<<B_LED)); // set PB5, PB1, PB2, PB3 to output
  DDRD |=( (1<<LED1) | (1<<LED2) ); //set PD5, PD6 to output
  //IO on reset
  PORTD &= ~(1<<LED1);
  PORTD &= ~(1<<LED2);
  PORTB &= ~(1<<BOARD_LED);
}

void interrupt_config(){
  cli(); //disable global interrupts;

  //External Interrupt INT0/INT1 (BTTN1,BTTN2)
  EICRA |= (1<<ISC01); //set ext int0 to falling edge detection
  EICRA &= ~(1<<ISC00);
  EICRA |= (1<<ISC11); //set ext int1 to falling edge detection
  EICRA &= ~(1<<ISC10);
  EIMSK |= (1<<INT0); //Enable ext interrupt 0
  EIMSK |= (1<<INT1); //enable ext interrupt 1

  /* commented out due to Timer1 being used by RGB PWM
  //Timer1 Output Compare Interrupt
  TCCR1A = 0x00; //Clear TCCR1A settings in case PWM set them
  TCCR1A &= ~(1<<WGM10); //Set to CTC (mode4) (Clear Timer on Compare)
  TCCR1A &= ~(1<<WGM11);
  TCCR1B |= (1<<WGM12);
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |= (1<<CS12); //Set prescaler to 256  using 3 CS bits
  TCCR1B &= ~(1<<CS11);
  TCCR1B &= ~(1<<CS10);
  TCNT1 = 0; //Set timer/counter to 0 
  OCR1A = 15624; //Set output compare PR value
  TIMSK1 = (1<<OCIE1A); //Enable output compare A interrupt
  */

  sei(); //Enable global interrupts(can also be SREG |= (1<<7) )
}

void pwm_config(){
  //Timer0 PWM mode (OC0A-LED2, OC0B-LED1)
  //note: IO_config needs pwm output pins
  TCCR0A |= (1<<WGM00); //Set to Fast PWM (mode3)
  TCCR0A |= (1<<WGM01);
  TCCR0B &= ~(1<<WGM02);
  TCCR0B &= ~(1<<CS02); //Set prescaler to 8  using 3 CS bits (fastest at 1)
  TCCR0B |= (1<<CS01);
  TCCR0B &= ~(1<<CS00);
  TCCR0A |= (1<<COM0A1); //set to non-inverting mode for OC0A
  TCCR0A &= ~(1<<COM0A0);
  TCCR0A |= (1<<COM0B1); //set to non-inverting mode for OC0B
  TCCR0A &= ~(1<<COM0B0);
  TCNT0 = 0; //set timer/counter to 0 
  OCR0A = 127; //Set output compare duty cycle based on 100%-255
  OCR0B = 127;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
  //Timer1 PWM mode (OC1A-R_LED, OC1B-G_LED)
  //note: IO_config needs pwm output pins
  TCCR1A |= (1<<WGM10); //Set to Fast PWM, 8-bit (mode5)
  TCCR1A &= ~(1<<WGM11);
  TCCR1B |= (1<<WGM12);
  TCCR1B &= ~(1<<WGM13);
  TCCR1B &= ~(1<<CS12); //Set prescaler to 8  using 3 CS bits (fastest at 1)
  TCCR1B |= (1<<CS11);
  TCCR1B &= ~(1<<CS10);
  TCCR1A |= (1<<COM1A1); //set to non-inverting mode for OC1A
  TCCR1A &= ~(1<<COM1A0);
  TCCR1A |= (1<<COM1B1); //set to non-inverting mode for OC1B
  TCCR1A &= ~(1<<COM1B0);
  TCNT1 = 0; //set timer/counter to 0 
  OCR1A = 127; //Set output compare duty cycle 
  OCR1B = 127;

  //Timer2 PWM mode (OC2A-B_LED)
  //note: IO_config needs pwm output pins
  TCCR2A |= (1<<WGM20); //Set to Fast PWM (mode3)
  TCCR2A |= (1<<WGM21);
  TCCR2B &= ~(1<<WGM22);
  TCCR2B &= ~(1<<CS22); //Set prescaler to 8  using 3 CS bits (fastest at 1)
  TCCR2B |= (1<<CS21);
  TCCR2B &= ~(1<<CS20);
  TCCR2A |= (1<<COM2A1); //set to non-inverting mode for OC2A
  TCCR2A &= ~(1<<COM2A0);
  TCNT2 = 0; //set timer/counter to 0 
  OCR2A = 127; //Set output compare duty cycle 
}

void adc_config(){
  //ADC Configuration (POT-ADC7)
  ADMUX &= ~(1<<REFS1);//Set Reference voltage to Vcc
  ADMUX |= (1<<REFS0);
  ADMUX |= (1<<ADLAR); //left shift ADC for 8-bit value
  ADMUX &= ~(1<<MUX3); //Set input to ADC7
  ADMUX |= (1<<MUX2);
  ADMUX |= (1<<MUX1);
  ADMUX |= (1<<MUX0);
  ADCSRA |= (1<<ADEN); //enable ADC
  ADCSRA &= ~(1<<ADSC); //starts conversion when set 
  ADCSRA &= ~(1<<ADATE); //disable ADC Auto-trigger (single-conversion mode)
  ADCSRA &= ~(1<<ADIF); //clear interrupt flag
  ADCSRA &= ~(1<<ADIE); //disable interrupt
  ADCSRA |= (1<<ADPS2); //set ADC prescaler to 128 (results in 125kHz, has recommended speed)
  ADCSRA |= (1<<ADPS1);
  ADCSRA |= (1<<ADPS0);
  ADCSRB = 0b00000000; //trigger selection
  //DIDR0 needed for digital pins to disconnect for analog use, not needed for ADC6/ADC7

}

ISR (INT0_vect){
  _delay_ms(75); //debounce button
  if(BIT_IS_CLEAR(PIND, BTTN1)){
    ADCSRA |= (1 << ADSC);	//start sampling
    while (BIT_IS_SET(ADCSRA, ADSC)) {}
    adc_value_b = ADCH;
  }
  //EIFR |= (1<<INTF0); //clear interrupt flag optional
}

ISR (INT1_vect){
   _delay_ms(75); //debounce button
  if(BIT_IS_CLEAR(PIND, BTTN2)){
    ADCSRA |= (1 << ADSC);	//start sampling
    while (BIT_IS_SET(ADCSRA, ADSC)) {}
    if (ADCH < 51){
      dir = -1;
      period = 1;
    }
    else if (ADCH < 102){
      dir = -1;
      period =0;
    }
    else if (ADCH < 153){
      dir = 0;
      period = 0;
    }
    else if(ADCH < 204) {
      dir = 1;
      period = 0;
    }
    else{
      dir = 1;
      period = 1;
    }
  }
  //EIFR |= (1<<INTF1); //clear interrupt flag optional
}

/*
ISR (TIMER1_COMPA_vect){
  PORTB ^= (1<<BOARD_LED); //Toggle on_board LED
  //TIFR1 |= (1<<OCF1A); //Clear interrupt flag optional
}
*/

//GFVA