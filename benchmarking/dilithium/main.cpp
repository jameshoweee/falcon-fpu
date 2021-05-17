#include <string>

#include <stdlib.h>
#include "mbed.h"
#include "stm32f7xx_hal.h"

extern "C" {
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

	#define BENCHMARK_ROUND 50
	uint64_t start, stop, delta, min, max;
	int us, cnt;
	long double average_us, average_clk, avclk_old, var, std_err, ddelta;

	#define MIN(a,b) (((a)<(b))?(a):(b))
	#define MAX(a,b) (((a)>(b))?(a):(b))

	#define CALC_RESET {		 \
		start = stop = 0;        \
		delta       = 0;         \
		ddelta      = 0;         \
		var         = 0;         \
		average_clk = 0;	 \
		average_us  = 0;	 \
		avclk_old   = 0;         \
		min         = 9999999999;\
		max         = 0;         \
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
		avclk_old    = average_clk;              \
		delta        = stop - start;             \
		ddelta       = (long double) delta;      \
		average_clk += ((ddelta-average_clk)/cnt);\
		var         += ((ddelta-average_clk)*(ddelta-avclk_old));\
		var         /= cnt;                      \
		average_us  += (long double)(us-average_us)/cnt;\
		min          = MIN(delta,min);           \
		max          = MAX(delta,max);           \
		cnt         += 1;			 \
	}
	
	#define CALC_AVG {				 \
		std_err      = sqrt(var/cnt);            \
	}

	#define timer_read_ms(x)    chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()

	//set so that cycle counter can be read from  DWT->CYCCNT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	int ret_val = 0;
	
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
		DWT->CYCCNT = 0;
		CALC_START
		ret_val = crypto_sign_keypair(pk, sk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Avg clock cycles:        %.0Lf\n\r", average_clk);
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
        pc.printf("Avg time in millisecs:   %.1Lf\n\r", average_us/1000);
	
	pc.printf("-----------------\n\r");
	pc.printf("| Doing Signing |\n\r");
	pc.printf("-----------------\n\r");
	
	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		DWT->CYCCNT = 0;
		randombytes(m, MLEN);
		CALC_START
	ret_val = crypto_sign(sm, &smlen, m, MLEN, sk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Avg clock cycles:        %.0Lf\n\r", average_clk);
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
        pc.printf("Avg time in millisecs:   %.1Lf\n\r", average_us/1000);

	pc.printf("------------------------\n\r");
	pc.printf("| Doing Signing (open) |\n\r");
	pc.printf("------------------------\n\r");

	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		DWT->CYCCNT = 0;		
		CALC_START
		ret_val = crypto_sign_open(m2, &mlen, sm, smlen, pk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Avg clock cycles:        %.0Lf\n\r", average_clk);
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
        pc.printf("Avg time in millisecs:   %.1Lf\n\r", average_us/1000);

	pc.printf("-------------------\n\r");
	pc.printf("| Doing Verifying |\n\r");
	pc.printf("-------------------\n\r");

	CALC_RESET
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		DWT->CYCCNT = 0;
		CALC_START	
	ret_val = crypto_sign_verify(sm, CRYPTO_BYTES, m, MLEN, pk);
		CALC_STOP
	}
	CALC_AVG
	
	pc.printf("Avg clock cycles:        %.0Lf\n\r", average_clk);
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
        pc.printf("Avg time in millisecs:   %.1Lf\n\r", average_us/1000);
	
	pc.printf("----------------------\n\r");
	pc.printf("| Dilithium Finished |\n\r");
	pc.printf("----------------------\n\r");
	
	fflush(stdout);
	
        pc.printf("Scheme fail if nonzero:  %d\n\r", ret_val);	
}
