#include <string>

#include "mbed.h"
#include "falcon-20190918/falcon.h"
#include "stm32f7xx_hal.h"

void my_random_seed(int seed);

//------------------------------------
// Hyperterminal configuration
// 115200 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalOut myled(LED1);
Timer timer;

int main()
{
	uint64_t scc_ms = 96000;	//	clocks per millisecond
	uint64_t ofl_ms = 44739;	//	milliseconds per overflow
	
	#define CYCLES_VARS							\
	int ms;									\
	uint32_t t;

	#define CYCLES_START {						\
		ms = timer.read_ms();					\
		t = DWT->CYCCNT;						\
	}

	#define CYCLES_ADD(cc) {					\
		t = DWT->CYCCNT - t;					\
		cc += (uint64_t) t;						\
		ms = timer.read_ms() - ms;				\
		ms -= ((uint64_t) t) / scc_ms;			\
		cc += ((uint64_t) (ms / ofl_ms)) << 32;	\
	}
	
	//set so that cycle counter can be read from  DWT->CYCCNT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        uint32_t start;
        uint32_t stop;
        uint32_t delta;
        uint32_t min = 0;
        uint32_t max = 0;
	//uint32_t t;
	uint64_t kg;
	//int ms;

	// Just a simple print to know everything works.
	int i = 1;
	while(i < 4) {
		wait(1);
		pc.printf("This program runs since %d seconds.\n", i++);
		myled = !myled;
	}

	// TODO: Use actual randomness for this later?
	char seed[16] = {0};
	shake256_context sc;
	shake256_init_prng_from_seed(&sc, seed, 16);

	// Falcon-512
	char priv_key[FALCON_PRIVKEY_SIZE(9)] = {0};
	char pub_key[FALCON_PUBKEY_SIZE(9)] = {0};
	char tmp[FALCON_TMPSIZE_KEYGEN(9)] = {0};

	timer.reset();
	timer.start();
	
	CYCLES_VARS
	CYCLES_START
	
	start = DWT->CYCCNT;
	kg=0;
	int ret_val = falcon_keygen_make(
		&sc, 9,
		&priv_key, FALCON_PRIVKEY_SIZE(9),
		&pub_key, FALCON_PUBKEY_SIZE(9),
		&tmp, FALCON_TMPSIZE_KEYGEN(9)
	);
	CYCLES_ADD(kg)
	stop = DWT->CYCCNT;
	ms = timer.read_ms();
	delta = stop - start;
	pc.printf("Clock cycles taken: %lu\n", delta);
        pc.printf("Time taken in ms %d\n", ms);

	pc.printf("Clock cycles taken: %lu\n", kg);
        pc.printf("Time taken in ms %d\n", ms);

	//Enable Interrupts;
	//delta = stop - start;
	//if (max < delta) {
	//	max = delta;
	//}
	//if (min > delta) {
	//	min = delta;
	//}

	// Print status code over serial. 0 means success!
	pc.printf("Keypair generation status: ");
	pc.printf(std::to_string(ret_val).c_str());
	pc.printf("\n");
}
