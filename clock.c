#ifndef F_CPU
#define F_CPU   16000000UL
#endif

#include "segm.h"
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

/* f = f_cpu / 1024, ovf happens every 256 tacts */
#define OVFPERSEC F_CPU / 1024 / 256
#define INC true
#define DEC false

/* 7-segment display pins & port can be configured in */
/* struct segm_Port & struct segm_Display initializations */

#define PINMODE		7
#define PINUP		6
#define PINDOWN		5
#define PINALRM		4
#define PININP		PIND

enum states {	RUN = 0,	/* Running(tiking) state */
		CHANGE1,	/* Changing 1st number */
		CHANGE2,	/* Changing 2nd number */
		ALARM		/* Alarm is ringing */
};

static enum states state = RUN;
static bool flag0_5s = false;	/* Switches every 0.5 sec */

/**
 * struct mode - stores data related to each mode
 *
 * @n:		numbers to display
 * @isset:	is this mode set
 * @tik:	function called every sec to alter the mode data
 */
struct mode {
	uint8_t n[2];
	bool isset;
	void (*tik)();
};

static struct mode modes[4] = {{.n = {0, 0}, .isset = true},
		 	       {.n = {0, 0}, .isset = false},
		 	       {.n = {0, 0}, .isset = false},
		 	       {.n = {0, 0}, .isset = false}};

enum modes {TIME = 0, CLOCK, TIMER, STW};
static enum modes curmode = TIME;

/**
 * convert - converts numbers into 7-segment representation
 *
 * @res:	pointer to where store the result
 * @n0:		1st number
 * @n1:		2nd number
 *
 * Uses segm_sym_table, stored in segm library
 */
static inline void convert(uint8_t *res, uint8_t n0, uint8_t n1)
{
	res[0] = segm_sym_table[n0 / 10];
	res[1] = segm_sym_table[n0 % 10];
	res[2] = segm_sym_table[n1 / 10];
	res[3] = segm_sym_table[n1 % 10];
}

/**
 * altermode - universal func for changng the data of specified mode
 *
 * @m:		specified mode
 * @nx:		alter 1st of 2nd number
 * @inc:	increment or decrement; pass INC or DEC macro in calls.
 *
 * The function has max values of each number of each mode, defined in a 
 * form of static array maxn[enum mode][nX], where X = [0, 1]. In case of 
 * adding extra modes, this array must be properly changed.
 */
static void altermode(enum modes m, uint8_t nx, bool inc)
{
	static uint8_t maxn[][2] = {[TIME]  = {24, 60},
				    [CLOCK] = {24, 60},
				    [TIMER] = {100, 60},
				    [STW]   = {100, 60}};

	/* Mode is set when incremented or decremented */ 
	modes[m].isset = true;

	uint8_t res = modes[m].n[nx] + (2 * inc - 1);

	if (res < maxn[m][nx]) {
		modes[m].n[nx] = res;
		return;
	} else if (inc) {
		modes[m].n[nx] = 0;
	} else
		modes[m].n[nx] = maxn[m][nx] - 1;
	if (nx)
		altermode(m, !nx, inc);
}

static void timetik()
{
	static uint8_t _sec = 0;

	if (++_sec >= 60) {
		altermode(TIME, 1, INC);
		_sec = 0;
	}
}

static void clocktik()
{
	if (!modes[CLOCK].isset)
		return;
	if (modes[CLOCK].n[1] == modes[TIME].n[1]
	    && modes[CLOCK].n[0] == modes[TIME].n[0]) {
		curmode = CLOCK;
		modes[CLOCK].isset = false;
		state = ALARM;
	}
}

static void timertik()
{
	if (!modes[TIMER].isset)
		return;
	altermode(TIMER, 1, DEC);
	if (modes[TIMER].n[0] == 0 && modes[TIMER].n[1] == 0) {
		curmode = TIMER;
		modes[TIMER].isset = false;
		state = ALARM;
	}
}

static void stwtik()
{
	if (!modes[STW].isset)
		return;
	altermode(STW, 1, INC);
}

static void reset()
{
	modes[curmode].n[0] = 0;
	modes[curmode].n[1] = 0;
	modes[curmode].isset = false;
}


/* Enabling and disabling buttons interrupts */

static void btns_on()
{
	PCMSK2 |= (1 << PINMODE) | (1 << PINUP) | (1 << PINDOWN);
}

static void btns_off()
{
	PCMSK2 &= ~((1 << PINMODE) | (1 << PINUP) | (1 << PINDOWN));
}

/* In case of button pressed things are done in TIM2_COMP2B */
/* interrupt after 255 tacts */
ISR(PCINT2_vect)
{
	btns_off();
	OCR2B =	TCNT2 + 255;
	TIMSK2 |= 1 << OCIE2B;	/* Enable TIM2_COMP2B interrupts */
}

/**
 * For the logic implemented here refer to README.md
 */
ISR(TIMER2_COMPB_vect)
{
	TIMSK2 &= ~(1 << OCIE2B);

	/* If PINMODE and one more button are held, reset current mode */
	if (PININP & (1 << PINMODE)
	    && (PININP & (1 << PINUP) || PININP & (1 << PINDOWN)))
		reset();

	if (PININP & (1 << PINMODE)) {
		state = state < CHANGE2 ? state + 1 : RUN;
	} else if (PININP & (1 << PINUP)) {
		if (state == RUN)
			curmode = curmode < STW ? curmode + 1 : TIME;
		else if (state < ALARM)
			altermode(curmode, state - 1, INC);
	} else if (PININP & (1 << PINDOWN)) {
		if (state == RUN)
			curmode = curmode > TIME ? curmode - 1 : STW;
		else if (state < ALARM)
			altermode(curmode, state - 1, DEC);
	}
	btns_on();
}

/* Occurs every F_CPU / 1024 / 256 times per second */
ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	static uint8_t cntovf = 0;

	if (++cntovf < OVFPERSEC) {
		if (cntovf == OVFPERSEC / 2)
			flag0_5s = !flag0_5s;
		return;
	}

	/* Folowwing occurs every 1 sec */
	cntovf = 0;
	flag0_5s = !flag0_5s;

	modes[TIME].tik();
	if (state == RUN) {
		modes[CLOCK].tik();
		modes[TIMER].tik();
		modes[STW].tik();
	}
}

ISR(TIMER2_COMPA_vect)
{
	TIMSK2 &= ~(1 << OCIE2A); /* Disable TIM2_COMP2A interrupts */
}

static void sleep_ms(uint16_t ms_val)
{
	static const uint8_t ms = (F_CPU / 1024) / 1000;
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	cli();		/* Disable interrupts -- as memory barrier */
	sleep_enable();	/* Set SE (sleep enable bit) */
	sei();  	/* Enable interrupts. We want to wake up, don't we? */
	while (ms_val--) {
		/* Enable TIM2_COMP2A interrupt by mask */
		TIMSK2 |= (1 << OCIE2A);
		/* Count 1 ms from TCNT2 to 0xFF (up direction) */
		OCR2A = TCNT2 + ms;
		sleep_cpu();
	}
	sleep_disable();	
}

static struct segm_Port PB = {
	.DDR = &DDRB,
	.PIN = &PINB,
	.PORT = &PORTB
};

static struct segm_Display display = {
	.SHCP = {.port = &PB, .pin = 0},
	.STCP = {.port = &PB, .pin = 1},
	.DS   = {.port = &PB, .pin = 2},
	.delay_func = &_delay_loop_1,
	.sleep_ms_func = &sleep_ms,
	.is_comm_anode = false
};

/**
 * process - function which manages output to display
 *
 * This function is isolated of clock logic, it just 
 * manages when and what digits must be shown.
 * Function must be called in endless loop.
 * Delays are provided by segm_indicate4 func.
 */
void process()
{
	uint8_t symbols[4];
	convert(symbols, modes[curmode].n[0], modes[curmode].n[1]);

	if (!flag0_5s) {
		if (state == CHANGE1) {
			symbols[0] = 0;
			symbols[1] = 0;
		} else if (state == CHANGE2) {
			symbols[2] = 0;
			symbols[3] = 0;
		} else if (state == ALARM) {
			symbols[0] = 0;
			symbols[1] = 0;
			symbols[2] = 0;
			symbols[3] = 0;
			PORTD ^= 1 << PINALRM;
		}
	}

	if (flag0_5s || state > RUN)
		symbols[1] |= 0x80; 	/* Dot blinking */

	segm_indicate4(&display, symbols);
}

int main()
{
	/* Modes initialization */
	modes[TIME].tik = timetik;
	modes[CLOCK].tik = clocktik;
	modes[TIMER].tik = timertik;
	modes[STW].tik = stwtik;

	char _time[] = __TIME__;	/* Expands to "HH:MM:SS" */
	modes[TIME].n[0] = (_time[0] - '0') * 10 + (_time[1] - '0');
	modes[TIME].n[1] = (_time[3] - '0') * 10 + (_time[4] - '0');

	segm_init(&display);

	/* Pins configuration */
	DDRD |= 1 << PINALRM;
	DDRD &= ~((1 << PINMODE) | (1 << PINUP) | (1 << PINDOWN));
	PCICR |= 1 << PCIE2;	/* Pin Change Interrupt Enable 2 */
	btns_on();

	/* Timer2 configuration */
	TIMSK2 |= 1 << TOIE2; /* TIM2_OVF interrupt enable */
	/* f = f_cpu / 1024, start timer */
	cli();
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	sei();

	while (1) {
		process();
	}
}
