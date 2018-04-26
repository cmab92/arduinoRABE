#include <avr/io.h>
#include <avr/interrupt.h>


const int LEDPIN = 13;

// I want the count period to be 1 second.  This is how long
// (in seconds) it takes the counter to reset.
const float countinterval = 1.0;

// For Arduino Uno and Duemilanove and Mega boards F_CPU = 16 MHz
const unsigned long CPU = F_CPU;

// I'll pick a large prescale divisor to allow long intervals
const unsigned long prescale = 1024;

// Round the floating point value to an integer
const unsigned count1 = (CPU * countinterval / prescale) + 0.5 - 1;

//
// For countinterval = 1 sec, the LED goes high for one second
// and low for a second.
//
// For countinterval equal to 1.0 and F_CPU equal to 16000000,
// we will get count1 = 15624
//
// That is, the result fits in a 16-bit unsigned int, as it must.
// With this scheme you can get count intervals of slightly over
// four seconds.
//
// If you want a shorter interval you can decrease,
// the prescale value to get better granularity,
// but you must always keep the size so that count1
// fits into a 16-bit integer. A more sophisticated program
// might select the optimum prescale factor based on the value
// of countinterval.
//


unsigned long compa1time, compb1time; // Millisecond values for OCR matches

// Fetching a 16-bit quantity is not atomic.  If it just
// happens that a timer0 (or other) interrupt occurs when
// trying to get compa1time, it just might return
// low byte from one count and high byte from the next
// count.  This can lead to occasional gross errors in
// the count value.
// To avoid this, don't try to read compa1time from the application,
// use this function:
unsigned long get_compa1time()
{
   unsigned long t;
   byte oldSREG = SREG;

   cli();
   t = compa1time;
   SREG = oldSREG;
   return t;
}

// See comment preceding get_compa1time()
unsigned long get_compb1time()
{
   unsigned long t;
   byte oldSREG = SREG;

   cli();
   t = compb1time;
   SREG = oldSREG;
   return t;
}

ISR(TIMER1_COMPA_vect)
{
   compa1time = millis();

   // Two methods for writing to the LED pin.
   // Comment out one of them and uncomment the other

   // Direct manipulation of Arduino Pin 13 ia
   // faster than digitalWrite
   //PINB = (1 << 5); // Toggle the LED

   // On the other hand, digitalWrite makes it
   // easy to use a different LEDPIN without having
   // to look up the port and bit number.
   // Local variable keeps from having to read the LED pin each time
   static byte compa1;
   digitalWrite(LEDPIN, (++compa1)&1);

}

ISR(TIMER1_COMPB_vect)
{
   compb1time = millis();
}
void setup()
{
   pinMode(LEDPIN, OUTPUT);
   digitalWrite(LEDPIN, HIGH);

   // Disable interrupts while setting registers
   cli();

   // Make sure all is normal in TCCR1A
   TCCR1A = 0;

   // Configure timer 1 for CTC mode with TOP = ICR1A, prescale = 1024
   // For finer granularity with shorter intervals, you could use a lower
   // prescale factor
   //
   // Top is ICR1
   TCCR1B = (1<< WGM13) | (1 << WGM12) | (1<< CS12) | (1 << CS10);


   // This determines output frequency: Timer will reset when it hits this value
   ICR1   = count1;

   OCR1A = count1 / 3; // Make it about 1/3 of the way through the count cycle

   // Arithmetic can overflow an int, so do the calculation
   // with long ints and truncate the result to an int
   OCR1B = 2L*count1 / 3; // Make it about 2/3 of the way through the count cycle

   // Enable CTC Interrupts for count match on comparator A
   // and also for comparator B
   TIMSK1 = (1 << OCIE1B) | (1 << OCIE1A);
   // Enable global interrupts
   sei();

   // As a sanity check, show the register values that were obtained by
   // calculations
   Serial.begin(57600);
   Serial.print("count1 = ");Serial.println(count1);
   Serial.print("OCR1A  = ");Serial.println(OCR1A);
   Serial.print("OCR1B  = ");Serial.println(OCR1B);
}

void loop()
{
   static unsigned long oldtime;
   unsigned long newtime = get_compb1time();
   if (newtime != oldtime) {
       oldtime = newtime;
       //Serial.print(millis());
       //Serial.print(":  ");
       Serial.print(get_compa1time(), DEC);
       Serial.print("  ");
       Serial.println(newtime, DEC);
   }
}
