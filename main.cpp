#include <string>

#include <stdlib.h>
#include "mbed.h"
#include "stm32f7xx_hal.h"

extern "C" {
#include "falcon-20190918/falcon.h"

#include "dilithium-pqm4/api.h"
#include "dilithium-pqm4/config.h"
#include "dilithium-pqm4/keccakf1600.h"
#include "dilithium-pqm4/ntt.h"
#include "dilithium-pqm4/packing.h"
#include "dilithium-pqm4/params.h"
#include "dilithium-pqm4/pointwise_mont.h"
#include "dilithium-pqm4/poly.h"
#include "dilithium-pqm4/polyvec.h"
#include "dilithium-pqm4/randombytes.h"
#include "dilithium-pqm4/reduce.h"
#include "dilithium-pqm4/rounding.h"
#include "dilithium-pqm4/sign.h"
#include "dilithium-pqm4/symmetric.h"
#include "dilithium-pqm4/vector.h"

/*
#include "dilithium2-pqclean/api.h"
#include "dilithium2-pqclean/fips202.h"
#include "dilithium2-pqclean/ntt.h"
#include "dilithium2-pqclean/packing.h"
#include "dilithium2-pqclean/params.h"
#include "dilithium2-pqclean/poly.h"
#include "dilithium2-pqclean/polyvec.h"
#include "dilithium2-pqclean/randombytes.h"
#include "dilithium2-pqclean/reduce.h"
#include "dilithium2-pqclean/rounding.h"
#include "dilithium2-pqclean/sign.h"
#include "dilithium2-pqclean/symmetric.h"
*/
}

extern "C" {
void my_random_seed(int seed);
//#include "dilithium2-pqm4/fips202.h"
}

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

static uint32_t fibo_a = 0xDEADBEEF, fibo_b = 0x01234567;

//	we want to be able to quickly reset the state to some value

void my_random_seed(int seed)
{
	fibo_a = (0xDEADBEEF ^ (seed << 20) ^ (seed >> 12)) | 1;
	fibo_b = seed;
}

//	lightweight fibonacci generator

int randombytes(uint8_t *obuf, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		fibo_a += fibo_b;
		fibo_b += fibo_a;
		obuf[i] = (fibo_a >> 24) ^ (fibo_b >> 16);
	}

	return 0;
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
	
	#define timer_read_ms(x)    chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()

	
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
	int us;
	unsigned logn = 9; // set to 9 for 512 parameters, 10 for 1024

	// Just a simple print to know everything works.
	int i = 1;
	while(i < 4) {
		wait(1);
		pc.printf("This program runs since %d seconds.\n", i++);
		myled = !myled;
	}
	
	/*
	char seed[16] = {0};
	shake256_context sc;
	shake256_init_prng_from_seed(&sc, seed, 16);

	// Falcon-512
	//char privkey[FALCON_PRIVKEY_SIZE(9)] = {0};
	//char pubkey[FALCON_PUBKEY_SIZE(9)] = {0};
	//char tmp[FALCON_TMPSIZE_KEYGEN(9)] = {0};

	void *pubkey, *pubkey2, *privkey, *sig, *expkey;
	size_t pubkey_len, privkey_len, sig_len, expkey_len;
	shake256_context rng;
	shake256_context hd;
	uint8_t nonce[40];
	//void *privkey; 
	void *tmpsd, *tmpkg, *tmpmp, *tmpvv, *tmpek, *tmpst;
	size_t tmpkg_len, tmpvv_len, tmpmp_len, tmpsd_len, tmpek_len, tmpst_len;
	
	fflush(stdout);
	
	pubkey_len  = FALCON_PUBKEY_SIZE(logn);
	privkey_len = FALCON_PRIVKEY_SIZE(logn);
	sig_len     = FALCON_SIG_VARTIME_MAXSIZE(logn);
	expkey_len  = FALCON_EXPANDEDKEY_SIZE(logn);
	
	tmpsd_len   = FALCON_TMPSIZE_SIGNDYN(logn);
	tmpkg_len   = FALCON_TMPSIZE_KEYGEN(logn);
	tmpmp_len   = FALCON_TMPSIZE_MAKEPUB(logn);
	tmpvv_len   = FALCON_TMPSIZE_VERIFY(logn);
	tmpek_len   = FALCON_TMPSIZE_EXPANDPRIV(logn);
	tmpst_len   = FALCON_TMPSIZE_SIGNTREE(logn);

	pubkey      = xmalloc(pubkey_len);
	pubkey2     = xmalloc(pubkey_len);
	privkey     = xmalloc(privkey_len);
	tmpkg       = xmalloc(tmpkg_len);	
	sig         = xmalloc(sig_len);
	expkey      = xmalloc(expkey_len);
		
	tmpkg       = xmalloc(tmpkg_len);	
	tmpsd 	    = xmalloc(tmpsd_len);
	tmpmp 	    = xmalloc(tmpmp_len);
	tmpvv       = xmalloc(tmpvv_len);
	tmpek       = xmalloc(tmpek_len);
	tmpst       = xmalloc(tmpst_len);

	clks = 0;

	int ret_val = 0;

	memset(privkey, 0, privkey_len);
	memset(pubkey, 0, pubkey_len);
	pc.printf("Doing KeyGen\n");

	ret_val = falcon_keygen_make(&rng, logn, privkey, privkey_len,
			pubkey, pubkey_len, tmpkg, tmpkg_len);
	
        pc.printf("keygen failed if nonzero: %d\n", ret_val);

	memset(pubkey2, 0xFF, pubkey_len);
	ret_val = falcon_make_public(pubkey2, pubkey_len,
			privkey, privkey_len, tmpmp, tmpmp_len);
        pc.printf("makepub failed if nonzero: %d\n", ret_val);
	//check_eq(pubkey, pubkey2, pubkey_len, "pub / repub");

	pc.printf("Doing Signing\n");
	memset(sig, 0, sig_len);	
	
	timer.reset();
	timer.start();
	start = DWT->CYCCNT;
	
	ret_val = falcon_sign_dyn(&rng, sig, &sig_len,
			privkey, privkey_len,
			"data1", 5, 0, tmpsd, tmpsd_len);
        pc.printf("sign_dyn failed if nonzero: %d\n", ret_val);
        
	stop  = DWT->CYCCNT;
	us    = timer.read_us();
	delta = stop - start;	
	
	pc.printf("Doing Verifying\n");			
	ret_val = falcon_verify(sig, sig_len, pubkey, 
			pubkey_len, "data1", 5, tmpvv, tmpvv_len);
	pc.printf("verify failed if nonzero: %d\n", ret_val);
	
	//CYCLES_ADD(clks)
	//stop  = DWT->CYCCNT;
	//ms    = timer.read_ms();
	//us    = timer.read_us();
	//delta = stop - start;	


	pc.printf("Doing expand private key\n");			
	ret_val = falcon_expand_privkey(expkey, expkey_len,
			privkey, privkey_len, tmpek, tmpek_len);
	pc.printf("expand_privkey failed if nonzero: %d\n", ret_val);
	
	pc.printf("Doing signing (tree)\n");	
	sig_len = FALCON_SIG_VARTIME_MAXSIZE(logn);		
	memset(sig, 0, sig_len);
	ret_val = falcon_sign_tree(&rng, sig, &sig_len, expkey,
		"data1", 5, 0, tmpst, tmpst_len);
	pc.printf("sign_tree failed if nonzero: %d\n", ret_val);

	pc.printf("Doing verify 2\n");			
	ret_val = falcon_verify(sig, sig_len, pubkey, pubkey_len, 
				"data1", 5, tmpvv, tmpvv_len);
	pc.printf("verify 2 failed if nonzero: %d\n", ret_val);
	
	pc.printf("Falcon finished\n");		
	*/

	#define MLEN 59
	size_t mlen, smlen;
	uint8_t pk[CRYPTO_PUBLICKEYBYTES] = {0};
	uint8_t sk[CRYPTO_SECRETKEYBYTES] = {0};
	int ret_val = 0;
	uint8_t m[MLEN + CRYPTO_BYTES];
	uint8_t m2[MLEN + CRYPTO_BYTES];
	uint8_t sm[MLEN + CRYPTO_BYTES];
	randombytes(m, MLEN);

	ret_val = crypto_sign_keypair(pk, sk);
	
	//ret_val = crypto_sign_signature(sigp, &siglen, &m, mlen, skp);
	ret_val = crypto_sign(sm, &smlen, m, MLEN, sk);

	ret_val = crypto_sign_open(m2, &mlen, sm, smlen, pk);
	
	timer.reset();
	timer.start();
	start = DWT->CYCCNT;
		
	ret_val = crypto_sign_verify(sm, CRYPTO_BYTES, m, MLEN, pk);

	stop  = DWT->CYCCNT;
	us    = timer.read_us();
	delta = stop - start;

	pc.printf("Clock cycles taken: %lu\n", delta);
        pc.printf("Time taken in microsecs %d\n", us);

	pc.printf("Clock cycles taken: %llu\n", clks);
        //pc.printf("Time taken in ms %d\n", ms);
        
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
	fflush(stdout);		
}
