#include <string>

#include <stdlib.h>
#include "mbed.h"
#include "stm32f7xx_hal.h"

extern "C" {
#include "falcon-20201020/falcon.h"
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
}

//------------------------------------
// Hyperterminal configuration
// 115200 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalOut myled(LED1);
Timer timer;

static void
check_eq(const void *a, const void *b, size_t len, const char *banner)
{
	size_t u;

	if (memcmp(a, b, len) == 0) {
		return;
	}
	fprintf(stderr, "%s: wrong value:\n", banner);
	fprintf(stderr, "a: ");
	for (u = 0; u < len; u ++) {
		fprintf(stderr, "%02x", ((const unsigned char *)a)[u]);
	}
	fprintf(stderr, "\n");
	fprintf(stderr, "b: ");
	for (u = 0; u < len; u ++) {
		fprintf(stderr, "%02x", ((const unsigned char *)b)[u]);
	}
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

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

int randombytes(uint8_t *obuf, size_t len)
{
	static uint32_t fibo_a = 0xDEADBEEF, fibo_b = 0x01234567;
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
	#define BENCHMARK_ROUND 100
	uint64_t start;
        uint64_t stop;
       	uint64_t delta;
       	uint64_t average_clk;
	int us, average_us, cnt;
	
	#define CALC_RESET {		 \
		average_clk = 0;	 \
		average_us  = 0;	 \
		timer.reset();		 \
		cnt = 1;		 \
	}
	
	#define CALC_START {		 \
		timer.reset();		 \
		timer.start();		 \
		start = DWT->CYCCNT;	 \
	}
	
	#define CALC_STOP {		 		 \
		stop         = DWT->CYCCNT;     	 \
		us           = timer.read_us(); 	 \
		delta        = stop - start;		 \
		average_clk += (delta-average_clk)/cnt;	 \
		average_us  += (us-average_us)/cnt;	 \
		cnt         += 1;			 \
	}
	
	#define CALC_AVG {				\
	}
	//		delta = average_clk / BENCHMARK_ROUND;  \
	//		us    = average_us  / BENCHMARK_ROUND;  \

	#define timer_read_ms(x)    chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()

	
	//set so that cycle counter can be read from  DWT->CYCCNT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	int ret_val = 0;
	
	int i = 3;
	while(i > 0) {
		wait(1);
		pc.printf("This program runs will run in %d seconds.\n\r", i--);
		myled = !myled;
	}

	/*
 	* Falcon code below taken from optimised, using the native FPU.
	* ret_val outputs 0 if functions work as expected.
	* comment code out below to switch between Falcon and Dilithium.
	*/
	/*
	unsigned logn = 10; // set to 9 for 512 parameters, 10 for 1024
	char seed[16] = {0};
	shake256_context sc;
	shake256_init_prng_from_seed(&sc, seed, 16);

	void *pubkey, *pubkey2, *privkey, *sig, *expkey;// *sigct, *expkey;
	size_t pubkey_len, privkey_len, sig_len, expkey_len;
	shake256_context rng;
	shake256_context hd;
	uint8_t nonce[40];
	void *tmpsd, *tmpkg, *tmpmp, *tmpvv, *tmpek, *tmpst;
	size_t tmpkg_len, tmpvv_len, tmpmp_len, tmpsd_len, tmpek_len, tmpst_len;
	
	fflush(stdout);
	
	pubkey_len  = FALCON_PUBKEY_SIZE(logn);
	privkey_len = FALCON_PRIVKEY_SIZE(logn);
	sig_len     = FALCON_SIG_CT_SIZE(logn);
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

	pc.printf("-------------------\n\r");
	pc.printf("| Starting Falcon |\n\r");
	pc.printf("-------------------\n\r");

	pc.printf("------------------------\n\r");
	pc.printf("| Doing Key Generation |\n\r");
	pc.printf("------------------------\n\r");

	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		memset(privkey, 0, privkey_len);
		memset(pubkey, 0, pubkey_len);
		CALC_START
		ret_val = falcon_keygen_make(&rng, logn, privkey, privkey_len,
				pubkey, pubkey_len, tmpkg, tmpkg_len);
		CALC_STOP
	}
	CALC_AVG
		
	pc.printf("Key Generation clock cycles taken: %lld\n\r", delta);
        pc.printf("Key Generation time taken in microsecs: %d\n\r", us);
        pc.printf("Key Generation failed if nonzero: %d\n\r", ret_val);
        
	memset(pubkey2, 0xFF, pubkey_len);
	ret_val = falcon_make_public(pubkey2, pubkey_len,
			privkey, privkey_len, tmpmp, tmpmp_len);
        pc.printf("Make Public failed if nonzero: %d\n\r", ret_val);
	check_eq(pubkey, pubkey2, pubkey_len, "pub / repub");
		
	pc.printf("-----------------------\n\r");
	pc.printf("| Doing Signing (dyn) |\n\r");
	pc.printf("-----------------------\n\r");

	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		memset(sig, 0, sig_len);
		CALC_START
		ret_val = falcon_sign_dyn(&rng, sig, &sig_len, FALCON_SIG_CT,
				privkey, privkey_len, "data1", 5, tmpsd, tmpsd_len);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Signing (dyn) clock cycles taken: %lld\n\r", delta);
        pc.printf("Signing (dyn) time taken in microsecs: %d\n\r", us);
        pc.printf("Signing (dyn) failed if nonzero: %d\n\r", ret_val);

	pc.printf("-------------------\n\r");	
	pc.printf("| Doing Verifying |\n\r");
	pc.printf("-------------------\n\r");	
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CALC_START			
		ret_val = falcon_verify(sig, sig_len, FALCON_SIG_CT, pubkey, 
				pubkey_len, "data1", 5, tmpvv, tmpvv_len);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Verifying clock cycles taken: %lld\n\r", delta);
        pc.printf("Verifying time taken in microsecs: %d\n\r", us);
	pc.printf("Verifying failed if nonzero: %d\n\r", ret_val);

	pc.printf("----------------------------\n\r");
	pc.printf("| Doing Expand Private Key |\n\r");
	pc.printf("----------------------------\n\r");
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CALC_START			
		ret_val = falcon_expand_privkey(expkey, expkey_len,
				privkey, privkey_len, tmpek, tmpek_len);
		CALC_STOP
	}
	CALC_AVG

	pc.printf("Expand private key clock cycles taken: %lld\n\r", delta);
        pc.printf("Expand private key time taken in microsecs: %d\n\r", us);
	pc.printf("Expand private key failed if nonzero: %d\n\r", ret_val);

	pc.printf("------------------------\n\r");	
	pc.printf("| Doing Signing (tree) |\n\r");
	pc.printf("------------------------\n\r");	
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		memset(sig, 0, sig_len);
		CALC_START		
		ret_val = falcon_sign_tree(&rng, sig, &sig_len, FALCON_SIG_CT, expkey,
			"data1", 5, tmpst, tmpst_len);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Signing tree clock cycles taken: %lld\n\r", delta);
        pc.printf("Signing tree time taken in microsecs: %d\n\r", us);
	pc.printf("Signing tree failed if nonzero: %d\n\r", ret_val);

	pc.printf("---------------------\n\r");
	pc.printf("| Doing Verifying 2 |\n\r");			
	pc.printf("---------------------\n\r");
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CALC_START
		ret_val = falcon_verify(sig, sig_len, FALCON_SIG_CT, pubkey, pubkey_len, 
					"data1", 5, tmpvv, tmpvv_len);
		CALC_STOP
	}
	CALC_AVG

	pc.printf("Verifying 2 clock cycles taken: %lld\n\r", delta);
        pc.printf("Verifying 2 time taken in microsecs: %d\n\r", us);
	pc.printf("Verifying 2 failed if nonzero: %d\n\r", ret_val);

        pc.printf("-------------------\n\r");	
	pc.printf("| Falcon Finished |\n\r");
        pc.printf("-------------------\n\r");	
	fflush(stdout);
	*/
	
	/*
 	* Dilithium Round 3 code using pqm4 as it's faster than pqclean.
 	* change Dilithium's parameters in config.h to either 2, 3, or 5.
	* ret_val outputs 0 if functions work as expected.
	* comment code out below to switch between Falcon and Dilithium.
	*/

	#define MLEN 59
	size_t mlen, smlen;
	uint8_t pk[CRYPTO_PUBLICKEYBYTES] = {0};
	uint8_t sk[CRYPTO_SECRETKEYBYTES] = {0};
	uint8_t m[MLEN + CRYPTO_BYTES];
	uint8_t m2[MLEN + CRYPTO_BYTES];
	uint8_t sm[MLEN + CRYPTO_BYTES];
	randombytes(m, MLEN);
	        
	fflush(stdout);

	pc.printf("----------------------\n\r");
	pc.printf("| Starting Dilithium |\n\r");
	pc.printf("----------------------\n\r");
	
	pc.printf("------------------------\n\r");
	pc.printf("| Doing Key Generation |\n\r");
	pc.printf("------------------------\n\r");
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CALC_START
		ret_val = crypto_sign_keypair(pk, sk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Key Generation clock cycles taken: %lld\n\r", delta);
        pc.printf("Key Generation time taken in microsecs: %d\n\r", us);
        pc.printf("Key Generation failed if nonzero: %d\n\r", ret_val);
	
	pc.printf("-----------------\n\r");
	pc.printf("| Doing Signing |\n\r");
	pc.printf("-----------------\n\r");
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND/20; r++) {
		CALC_START
		ret_val = crypto_sign(sm, &smlen, m, MLEN, sk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Signing clock cycles taken: %lld\n\r", delta);
        pc.printf("Signing time taken in microsecs: %d\n\r", us);
	pc.printf("Signing failed if nonzero: %d\n\r", ret_val);

	pc.printf("------------------------\n\r");
	pc.printf("| Doing Signing (open) |\n\r");
	pc.printf("------------------------\n\r");

	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CALC_START
		ret_val = crypto_sign_open(m2, &mlen, sm, smlen, pk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Signing (open) clock cycles taken: %lld\n\r", delta);
        pc.printf("Signing (open) time taken in microsecs: %d\n\r", us);
	pc.printf("Signing (open) failed if nonzero: %d\n\r", ret_val);

	pc.printf("-------------------\n\r");
	pc.printf("| Doing Verifying |\n\r");
	pc.printf("-------------------\n\r");

	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CALC_START	
		ret_val = crypto_sign_verify(sm, CRYPTO_BYTES, m, MLEN, pk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Verifying clock cycles taken: %lld\n\r", delta);
        pc.printf("Verifying time taken in microsecs: %d\n\r", us);
	pc.printf("Verifying failed if nonzero: %d\n\r", ret_val);
	
	pc.printf("----------------------\n\r");
	pc.printf("| Dilithium Finished |\n\r");
	pc.printf("----------------------\n\r");
	
	fflush(stdout);
			
}
