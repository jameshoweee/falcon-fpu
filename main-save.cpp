#include <string>

#include <stdlib.h>
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

static void *
xmalloc(size_t len)
{
	void *buf;

	if (len == 0) {
		return NULL;
	}
	buf = malloc(len);
	if (buf == NULL) {
		fprintf(stderr, "memory allocation error\n");
		exit(EXIT_FAILURE);
	}
	return buf;
}

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
        //uint32_t min = 0;
        //uint32_t max = 0;
	//uint32_t t;
	uint64_t clks;

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
	char privkey[FALCON_PRIVKEY_SIZE(9)] = {0};
	char pubkey[FALCON_PUBKEY_SIZE(9)] = {0};
	char tmp[FALCON_TMPSIZE_KEYGEN(9)] = {0};

	shake256_context rng;
	shake256_context hd;
	uint8_t nonce[40];
	//void *privkey; 
	void *sig, *tmpsd;
	size_t privkey_len, sig_len, tmpsd_len;
	fflush(stdout);
	
	privkey_len = FALCON_PRIVKEY_SIZE(9);
	sig_len     = FALCON_SIG_VARTIME_MAXSIZE(9);
	tmpsd_len   = FALCON_TMPSIZE_SIGNDYN(9);
	sig         = xmalloc(sig_len);
	tmpsd 	    = xmalloc(tmpsd_len);
	
	timer.reset();
	timer.start();
	
	CYCLES_VARS
	CYCLES_START
	
	start = DWT->CYCCNT;
	clks=0;
	
	//simple swtich to decide keygen, sign, or verify
	int ret_val;
	
	/*
	int flag = 1;
	if (flag==0) { // Key Generation
		pc.printf("Doing KeyGen\n");
		ret_val = falcon_keygen_make(
			&sc, 9,
			&privkey, FALCON_PRIVKEY_SIZE(9),
			&pubkey, FALCON_PUBKEY_SIZE(9),
			&tmp, FALCON_TMPSIZE_KEYGEN(9));
		}
	else if (flag==1) { // Signing
		pc.printf("Doing Signing\n");
		ret_val = falcon_sign_start(&rng, nonce, &hd);
		shake256_inject(&hd, "data1", 5);
		memset(sig, 0, sig_len);
		falcon_sign_dyn_finish(&rng, &sig, &sig_len, privkey, 
			privkey_len, &hd, nonce, 0, tmpsd, tmpsd_len);
		}
	else if (flag==2) { // Verifying
		//break;
		}
	else { //Nothing
		//break;
		}
	*/
	
	pc.printf("Doing Signing\n");
	ret_val = falcon_sign_start(&rng, nonce, &hd);
	shake256_inject(&hd, "data1", 5);
	memset(sig, 0, sig_len);
	falcon_sign_dyn_finish(&rng, &sig, &sig_len, privkey, 
		privkey_len, &hd, nonce, 0, tmpsd, tmpsd_len);
	}
		
	CYCLES_ADD(clks)
	stop  = DWT->CYCCNT;
	ms    = timer.read_ms();
	delta = stop - start;
	
	pc.printf("KeyGen clock cycles taken: %lu\n", delta);
        pc.printf("KeyGen time taken in ms %d\n", ms);

	pc.printf("KeyGen clock cycles taken: %llu\n", clks);
        pc.printf("KeyGen time taken in ms %d\n", ms);
        
	//Enable Interrupts;
	//delta = stop - start;
	//if (max < delta) {
	//	max = delta;
	//}
	//if (min > delta) {
	//	min = delta;
	//}

	// Print status code over serial. 0 means success!
	pc.printf("Output Zero if Falcon is Correct: ");
	pc.printf(std::to_string(ret_val).c_str());
	pc.printf("\n");
}
