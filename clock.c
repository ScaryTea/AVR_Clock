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

#ifndef HOURS
#define HOURS 0
#endif
#ifndef MINUTES
#define MINUTES 0
#endif

enum states {	RUN = 0,	/* running state */
		CHANGE1,	/* changing 1st number */
		CHANGE2,	/* changing 2nd number */
		ALARM		/* alarm is ringing */
};

static enum states state = RUN;
static uint8_t cnt1sec = 0;
static bool flag1s = false, flag0_3s = false;
static uint8_t _sec = 0;

/**
 * struct mode - stores data related to each mode
 *
 * @n:		numbers to display
 * @isset:	is this mode set
 * @tik:	function called every sec to alter the mode data
 *
 */
struct mode {
	uint8_t n[2];
	bool isset;
	void (*tik)();
};

enum modes {TIME = 0, CLOCK, TIMER, STW};
static struct mode modes[4] = {{.n = {0, 0}, .isset = true},
		 	       {.n = {0, 0}, .isset = false},
		 	       {.n = {0, 0}, .isset = false},
		 	       {.n = {0, 0}, .isset = false}};

static enum modes curmode = TIME;


/* alarm melody */

static bool notes_isplaying = false;
static uint8_t _cntdur = 0;
static uint32_t _cntfreqval = 0, _cntfreq = 0;

#define NOTES_LEN 100
static uint8_t notes_reallen = 9;
static uint16_t notes_freqs[NOTES_LEN] = {392, 392, 392, 311, 466, 392, 311, 466, 392}; /* Hz */
static uint16_t notes_durs[NOTES_LEN] = {1000,1000,1000,1000,250,1000,1000,250,2000}; /* ms */

static uint8_t cur = 0;
static void notes_play()
{
	_cntfreqval = F_CPU / 1024 / notes_freqs[cur];
	_cntfreq = 0;
	_cntdur = (F_CPU / 1024 / 256 / 1000) * notes_durs[cur];

	notes_isplaying = true;

	if (++cur >= notes_reallen) {
		cur = 0;
		notes_isplaying = false;
		return;
	}
	TIMSK0 |= 1 << TOIE0;
}

/* 15625 Hz event */
ISR(TIMER0_OVF_vect)
{
	if (--_cntdur == 0) {
		TIMSK0 &= ~(1 << TOIE0);
		PORTD |= 1 << 4;
		notes_play();
	}
	if (_cntfreq >= _cntfreqval) {
	PORTD |= 1 << 4;
		//PORTD ^= 1 << 4;
		_cntfreq = 0;
	}
}


static inline void convert(uint8_t *res, uint8_t n0, uint8_t n1)
{
	res[0] = segm_sym_table[n0 / 10];
	res[1] = segm_sym_table[n0 % 10];
	res[2] = segm_sym_table[n1 / 10];
	res[3] = segm_sym_table[n1 % 10];
}

static void altermode(enum modes m, uint8_t nx, bool inc)
{
	static uint8_t maxn[][2] = {{24, 60}, {24, 60}, {100, 60}, {100, 60}};

	modes[m].isset = true;

	uint8_t res = modes[m].n[nx] + 2 * inc - 1;

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
	if (++_sec >= 60) {
		altermode(TIME, 1, true);
		_sec = 0;
	}
}

static void clocktik()
{
	if (!modes[CLOCK].isset)
		return;
	if (modes[CLOCK].n[1] == modes[TIME].n[1] && 
	    modes[CLOCK].n[0] == modes[TIME].n[0]) {
		curmode = CLOCK;
		modes[CLOCK].isset = false;
		state = ALARM;
	}
}

static void timertik()
{
	if (!modes[TIMER].isset)
		return;
	altermode(TIMER, 1, false);
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
	altermode(STW, 1, true);
}

static void reset()
{
	modes[curmode].n[0] = 0;
	modes[curmode].n[1] = 0;
	modes[curmode].isset = false;
}

/* buttons handling */

static void btns_on()
{
	PCMSK2 |= (1 << 5) | (1 << 6) | (1 << 7);
}

static void btns_off()
{
	PCMSK2 &= ~((1 << 5) | (1 << 6) | (1 << 7));
}

ISR(PCINT2_vect)
{
	btns_off();
	TIMSK2 |= 1 << OCIE2B;
}

ISR(TIMER2_COMPB_vect)
{
	static uint8_t cnt = 0;
	
	if (cnt++ < 2)
		return;

	TIMSK2 &= ~(1 << OCIE2B);

	if (PIND & (1 << PIND7)) {		/* btn_mode */
		state = state < CHANGE2 ? state + 1 : RUN;
	} else if (PIND & (1 << PIND6)) {	/* btn_up */
		if (state == RUN)
			curmode = curmode < STW ? curmode + 1 : TIME;
		else if (state < ALARM)
			altermode(curmode, state - 1, true);
	} else if (PIND & (1 << PIND5)) {	/* btn_down */
		if (state == RUN)
			curmode = curmode > TIME ? curmode - 1 : STW;
		else if (state < ALARM)
			altermode(curmode, state - 1, false);
	}
	cnt = 0;
	btns_on();
}

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	if (++cnt1sec < 61) {
		if (cnt1sec == 25)
			flag0_3s = !flag0_3s; 
		return;
	}
	/* folowwing occurs every 1 ms */
	modes[TIME].tik();
	if (state == RUN) {
		modes[CLOCK].tik();
		modes[TIMER].tik();
		modes[STW].tik();
	}
	flag1s = !flag1s;
	flag0_3s = !flag0_3s;
	cnt1sec = 0;
}

ISR(TIMER2_COMPA_vect)
{
	TIMSK2 &= ~(1 << OCIE2A); /* disable tim2_comp2a interrupts */
}

static void sleep_ms(uint16_t ms_val)
{
	static const uint8_t ms = (F_CPU / 1024) / 1000;
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	cli();		/* Disable interrupts -- as memory barrier */
	sleep_enable();	/* Set SE (sleep enable bit) */
	sei();  	/* Enable interrupts. We want to wake up, don't we? */
	while (ms_val--) {
		/* Enable Timer2 CompareA interrupt by mask */
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

void process()
{
	uint8_t symbols[4];
	convert(symbols, modes[curmode].n[0], modes[curmode].n[1]);

	if (!flag0_3s) {
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
			PORTD ^= 1 << 4;
		}
	}

	if (flag0_3s || state > RUN)
		symbols[1] |= 0x80; 	/* dot */

	segm_indicate4(&display, symbols);
}

int main()
{
	DDRD |= 1 << 4;		/* alarm pin */

	/* modes initialization */
	modes[TIME].tik = timetik;
	modes[CLOCK].tik = clocktik;
	modes[TIMER].tik = timertik;
	modes[STW].tik = stwtik;

	modes[TIME].n[0] = HOURS;
	modes[TIME].n[1] = MINUTES;

	segm_init(&display);

	OCR2B = 255;	/* for buttons polling */
	TIMSK2 |= 1 << TOIE2; /* tim2_ovf interr enable */

	DDRD &= ~((1 << 5) | (1 << 6) | (1 << 7));	/* input */
	PCICR |= 1 << PCIE2;	/* Pin Change Interrupt Enable 2 */
	btns_on();

	/* Enable Timer2 */
	/* f = Fclk_io / 1024, start timer */
	cli();
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	sei();

	while (1) {
		process();
	}
}
