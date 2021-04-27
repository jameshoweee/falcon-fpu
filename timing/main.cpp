#include <string>

#include <stdlib.h>
#include "mbed.h"
#include "stm32f7xx_hal.h"

//included to change clock freq
#include "mbed/TARGET_NUCLEO_F767ZI/TOOLCHAIN_GCC_ARM/system_stm32f7xx.h"
#include "mbed/TARGET_NUCLEO_F767ZI/TOOLCHAIN_GCC_ARM/stm32f7xx_ll_rcc.h"
#include "mbed/TARGET_NUCLEO_F767ZI/TOOLCHAIN_GCC_ARM/stm32f7xx_hal_rcc.h"

// for testing falcon
#include <stdio.h>
#include <time.h>
#include <math.h>

extern "C" {
#include "falcon-20201020/falcon.h"
#include "falcon-20201020/inner.h"
}

//------------------------------------
// Hyperterminal configuration
// 115200 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalOut myled(LED1);
Timer timer;

//unsigned logn = 9; // set to 9 for 512 parameters, 10 for 1024
#define MKN(logn)   ((size_t)1 << (logn))
#define RNG_CONTEXT   inner_shake256_context

/* ==================================================================== */

/*
 * Get a random 8-byte integer from a SHAKE-based RNG. This function
 * ensures consistent interpretation of the SHAKE output so that
 * the same values will be obtained over different platforms, in case
 * a known seed is used.
 */
static inline uint64_t
get_rng_u64(inner_shake256_context *rng)
{
	/*
	 * We enforce little-endian representation.
	 */

//#if FALCON_LE  // yyyLE+1
	/*
	 * On little-endian systems we just interpret the bytes "as is"
	 * (this is correct because the exact-width types such as
	 * 'uint64_t' are guaranteed to have no padding and no trap
	 * representation).
	 */
	uint64_t r;

	inner_shake256_extract(rng, (uint8_t *)&r, sizeof r);
	return r;
//#else  // yyyLE+0
//	uint8_t tmp[8];

//	inner_shake256_extract(rng, tmp, sizeof tmp);
//	return (uint64_t)tmp[0]
//		| ((uint64_t)tmp[1] << 8)
//		| ((uint64_t)tmp[2] << 16)
//		| ((uint64_t)tmp[3] << 24)
//		| ((uint64_t)tmp[4] << 32)
//		| ((uint64_t)tmp[5] << 40)
//		| ((uint64_t)tmp[6] << 48)
//		| ((uint64_t)tmp[7] << 56);
//#endif  // yyyLE-
}

/*
 * Minimal recursion depth at which we rebuild intermediate values
 * when reconstructing f and g.
 */
#define DEPTH_INT_FG   4

static inline uint64_t
fpr_ursh(uint64_t x, int n)
{
	x ^= (x ^ (x >> 32)) & -(uint64_t)(n >> 5);
	return x >> (n & 31);
}

/*
 * Right-shift a 64-bit signed value by a possibly secret shift count
 * (see fpr_ursh() for the rationale).
 *
 * Shift count n MUST be in the 0..63 range.
 */
static inline int64_t
fpr_irsh(int64_t x, int n)
{
	x ^= (x ^ (x >> 32)) & -(int64_t)(n >> 5);
	return x >> (n & 31);
}

/*
 * Left-shift a 64-bit unsigned value by a possibly secret shift count
 * (see fpr_ursh() for the rationale).
 *
 * Shift count n MUST be in the 0..63 range.
 */
static inline uint64_t
fpr_ulsh(uint64_t x, int n)
{
	x ^= (x ^ (x << 32)) & -(uint64_t)(n >> 5);
	return x << (n & 31);
}

static inline fpr *
align_fpr_1(void *tmp)
{
	uint8_t *atmp;
	unsigned off;

	atmp = (uint8_t *)tmp;
	off = (uintptr_t)atmp & 7u;
	if (off != 0) {
		atmp += 8u - off;
	}
	return (fpr *)atmp;
}


static inline uint8_t *
align_u64(void *tmp)
{
	uint8_t *atmp;
	unsigned off;

	atmp = (uint8_t *)tmp;
	off = (uintptr_t)atmp & 7u;
	if (off != 0) {
		atmp += 8u - off;
	}
	return atmp;
}

static inline uint8_t *
align_u16(void *tmp)
{
	uint8_t *atmp;

	atmp = (uint8_t *)tmp;
	if (((uintptr_t)atmp & 1u) != 0) {
		atmp ++;
	}
	return atmp;
}

#define IMAX_BITS(m) ((m)/((m)%255+1) / 255%255*8 + 7-86/((m)%255+12))
#define RAND_MAX_WIDTH IMAX_BITS(RAND_MAX)
_Static_assert((RAND_MAX & (RAND_MAX + 1u)) == 0, "RAND_MAX not a Mersenne number");

uint64_t rand64(void) {
  uint64_t r = 0;
  for (int i = 0; i < 64; i += RAND_MAX_WIDTH) {
    r <<= RAND_MAX_WIDTH;
    r ^= (unsigned) rand();
  }
  return r;
}

uint32_t rand32(void) {
  uint64_t r = 0;
  for (int i = 0; i < 32; i += RAND_MAX_WIDTH) {
    r <<= RAND_MAX_WIDTH;
    r ^= (unsigned) rand();
  }
  return r;
}

uint32_t rand16(void) {
  uint64_t r = 0;
  for (int i = 0; i < 16; i += RAND_MAX_WIDTH) {
    r <<= RAND_MAX_WIDTH;
    r ^= (unsigned) rand();
  }
  return r;
}

int main()
{

		///////////////////////////////////////////////////////
	// code for taking timing---
	///////////////////////////////////////////////////////
	
	#define BENCHMARK_ROUND 100000
	uint64_t start, stop, delta, delta_old, min, max;
	int us, cnt;
	long double average_us, average_clk, avclk_old, var, std_err;

	#define MIN(a,b) (((a)<(b))?(a):(b))
	#define MAX(a,b) (((a)>(b))?(a):(b))

	#define CALC_RESET {		 \
		delta       = 0;         \
		delta_old   = 0;         \
		var         = 0;         \
		average_clk = 0;	 \
		average_us  = 0;	 \
		delta_old   = 0;         \
		avclk_old   = 0;         \
		min         = 9999999999;\
		max         = 0;         \
		timer.reset();		 \
		cnt = 1;		 \
	}
	
	#define CALC_START {		 \
		wait(0.01);           \
		timer.reset();		 \
		timer.start();		 \
		start = DWT->CYCCNT;	 \
	}
	
	#define CALC_STOP {		 		 \
		stop         = DWT->CYCCNT;     	 \
		us           = timer.read_us(); 	 \
		delta_old    = delta;                    \
		avclk_old    = average_clk;              \
		delta        = stop - start;             \
		average_clk += (long double)(delta-average_clk)/cnt;	 \
		var         += (long double)((delta-average_clk)*(delta-avclk_old))/cnt; \
		average_us  += (long double)(us-average_us)/cnt;	 \
		min          = MIN(delta,min);           \
		max          = MAX(delta,max);           \
		cnt         += 1;			 \
		wait(0.01);                           \
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
	///////////////////////////////////////////////////////
	
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
	
	#define MUL31(x, y)   ((uint64_t)((x) | (uint32_t)0x80000000) \
		               * (uint64_t)((y) | (uint32_t)0x80000000) \
		               - ((uint64_t)(x) << 31) - ((uint64_t)(y) << 31) \
		               - ((uint64_t)1 << 62))
		               
	#define MUL15(x, y)   ((uint32_t)((x) | (uint32_t)0x80000000) \
		               * (uint32_t)((y) | (uint32_t)0x80000000) \
		               & (uint32_t)0x3FFFFFFF)
	
	#define rounds        500
	
	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing multiplication---\n\r");
	pc.printf("-------------------------\n\r");
	
	/// function for 32->32 mult
	pc.printf("-------------------\n\r");
	pc.printf("32->32 mult--------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
//		uint32_t r2  = rand32();
		uint32_t res = 0;
		CALC_START
		res = (uint32_t)r1*r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->64 mult
	pc.printf("-------------------\n\r");
	pc.printf("32->64 mult--------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1*r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for MUL31 and MUL15
	pc.printf("-------------------\n\r");
	pc.printf("MUL15 mult---------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){
		uint16_t r1  = rand16() >> 1;
		uint16_t r2  = 0;
		if (r % 16 == 0) continue; else (r2 = rand16() >> 1);
		uint32_t res = 0;
		CALC_START
		res = MUL15(r1, r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
	
	fflush(stdout);
	wait(1);
		
	pc.printf("-------------------\n\r");
	pc.printf("MUL31 mult---------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32() >> 1;
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32() >> 1;
		uint64_t res = 0;
		CALC_START
		res = MUL31(r1, r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
	
	fflush(stdout);
	wait(1);

	/// function for 64->64 mult
	pc.printf("-------------------\n\r");
	pc.printf("64->64 mult--------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		uint64_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand64();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1*r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
//////////////////////////////////////////////////////////////////////////////////////////

	/// function for 32->32 mult
	pc.printf("-------------------\n\r");
	pc.printf("float->float mult--\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		float r1  = (float)(rand64()/rand16());
		float r2  = 0;
		if (r % 16 == 0) continue; else r2 = (float)(rand64()/rand16());
		CALC_START
		float res = r1*r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->64 mult
	pc.printf("-------------------\n\r");
	pc.printf("float->double mult-\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		float r1  = (float)(rand64()/rand16());
		float r2  = 0;
		if (r % 16 == 0) continue; else r2 = (float)(rand64()/rand16());
		CALC_START
		double res = (double)r1*r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for 64->64 mult
	pc.printf("-------------------\n\r");
	pc.printf("double->double mult\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		double r1  = (double)(rand64()/rand16());
		double r2  = 0;
		if (r % 16 == 0) continue; else r2 = (double)(rand64()/rand16());
		CALC_START
		double res = r1*r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

//////////////////////////////////////////////////////////////////////////////////////////
	pc.printf("-------------------------\n\r");
	pc.printf("Testing Division---------\n\r");
	pc.printf("-------------------------\n\r");
	// casting to uints, rather than floats as I think this
	// might be more of the point of Pornin in BearSSL:
	// https://www.bearssl.org/ctmul.html	
	
	/// function for 32->32 div
	pc.printf("-------------------\n\r");
	pc.printf("32->32 div---------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint32_t res = 0;
		CALC_START
		res = (uint32_t)r1/r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->64 div
	pc.printf("-------------------\n\r");
	pc.printf("32->64 div---------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1/r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for 64->64 div
	pc.printf("-------------------\n\r");
	pc.printf("64->64 div---------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand64();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1/r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));
	
	/// function for float->float div
	pc.printf("-------------------\n\r");
	pc.printf("float->float div---\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		float r1  = (float)rand32();
		float r2  = 0;
		if (r % 16 == 0) continue; else r2 = (float)rand32();
		float res = 0;
		CALC_START
		res = r1/r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for float->double div
	pc.printf("-------------------\n\r");
	pc.printf("float->double div--\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		float r1  = (float)rand32();
		float r2  = 0;
		if (r % 16 == 0) continue; else r2 = (float)rand32();
		double res = 0;
		CALC_START
		res = (double)r1/r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));	
	
	/// function for double->double div
	pc.printf("-------------------\n\r");
	pc.printf("double->double div-\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		double r1  = (double)rand64();
		double r2  = 0;
		if (r % 16 == 0) continue; else r2 = (double)rand64();
		double res = 0;
		CALC_START
		res = r1/r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));	
		
	fflush(stdout);
	wait(1);
////////////////////////////////////////////////////////////////////////
	pc.printf("-------------------------\n\r");
	pc.printf("Testing L/R shifts-------\n\r");
	pc.printf("-------------------------\n\r");
	// casting to uints, rather than floats as I think this
	// might be more of the point of Pornin in BearSSL:
	// https://www.bearssl.org/ctmul.html	
	
	/// function for 32->32 rshift
	pc.printf("-------------------\n\r");
	pc.printf("32->32 lshift------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint32_t res = 0;
		CALC_START
		res = (uint32_t)r1 << r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->32 lshift
	pc.printf("-------------------\n\r");
	pc.printf("32->32 rshift------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint32_t res = 0;
		CALC_START
		res = (uint32_t)r1 >> r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->64 lshift
	pc.printf("-------------------\n\r");
	pc.printf("32->64 lshift------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1 << r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->64 rshift
	pc.printf("-------------------\n\r");
	pc.printf("32->64 rshift------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand32();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1 >> r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for 64->64 lshift
	pc.printf("-------------------\n\r");
	pc.printf("64->64 lshift------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		uint64_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand64();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1 << r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 64->64 rshift
	pc.printf("-------------------\n\r");
	pc.printf("64->64 rshift------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		uint64_t r2  = 0;
		if (r % 16 == 0) continue; else r2 = rand64();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)r1 >> r2;
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	pc.printf("-------------------------\n\r");
	pc.printf("Testing Sqrt-------------\n\r");
	pc.printf("-------------------------\n\r");
	// casting to uints, rather than floats as I think this
	// might be more of the point of Pornin in BearSSL:
	// https://www.bearssl.org/ctmul.html	
	
	/// function for 32->32 sqrt
	pc.printf("-------------------\n\r");
	pc.printf("32->32 sqrt--------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint32_t res = 0;
		CALC_START
		res = (uint32_t)sqrt(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for 32->64 dqrt
	pc.printf("-------------------\n\r");
	pc.printf("32->64 sqrt--------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1  = rand32();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)sqrt(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for 64->64 sqrt
	pc.printf("-------------------\n\r");
	pc.printf("64->64 sqrt--------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		uint64_t res = 0;
		CALC_START
		res = (uint64_t)sqrt(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

//////////////////////////////////////////////////////////////////////////
	/// function for fpr add
	pc.printf("-------------------\n\r");
	pc.printf("fpr add------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		fpr r2  = fpr_zero;
		if (r % 16 == 0) continue; else r2 = FPR((double)rand64());
		CALC_START
		fpr res = fpr_add(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for fpr sub
	pc.printf("-------------------\n\r");
	pc.printf("fpr sub------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		fpr r2  = fpr_zero;
		if (r % 16 == 0) continue; else r2 = FPR((double)rand64());
		CALC_START
		fpr res = fpr_sub(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr floor
	pc.printf("-------------------\n\r");
	pc.printf("fpr floor----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		int64_t res = fpr_floor(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr mul
	pc.printf("-------------------\n\r");
	pc.printf("fpr mul------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		fpr r2  = fpr_zero;
		if (r % 16 == 0) continue; else r2 = FPR((double)rand64());
		CALC_START
		fpr res = fpr_mul(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for fpr sqr
	pc.printf("-------------------\n\r");
	pc.printf("fpr sqr------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		fpr res = fpr_sqr(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr inv
	pc.printf("-------------------\n\r");
	pc.printf("fpr inv------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		fpr res = fpr_inv(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for fpr div
	pc.printf("-------------------\n\r");
	pc.printf("fpr div------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		fpr r2  = fpr_zero;
		if (r % 16 == 0) continue; else r2 = FPR((double)rand64());
		CALC_START
		fpr res = fpr_div(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr ursh
	pc.printf("-------------------\n\r");
	pc.printf("fpr ursh-----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		int r2  = rand16() >> 10;
		CALC_START
		uint64_t res = fpr_ursh(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for fpr irsh
	pc.printf("-------------------\n\r");
	pc.printf("fpr irsh-----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		int64_t r1  = rand64();
		int r2  = rand16() >> 10;
		CALC_START
		int64_t res = fpr_ursh(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr ulsh
	pc.printf("-------------------\n\r");
	pc.printf("fpr ulsh-----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		int r2  = rand16() >> 10;
		CALC_START
		uint64_t res = fpr_ursh(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr of
	pc.printf("-------------------\n\r");
	pc.printf("fpr of-------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		uint64_t r1  = rand64();
		CALC_START
		fpr res = fpr_of(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	/// function for fpr rint
	pc.printf("-------------------\n\r");
	pc.printf("fpr rint-----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		int64_t res = fpr_rint(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr trunc
	pc.printf("-------------------\n\r");
	pc.printf("fpr trunc----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		int64_t res = fpr_trunc(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr neg
	pc.printf("-------------------\n\r");
	pc.printf("fpr neg------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		fpr res = fpr_neg(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr half
	pc.printf("-------------------\n\r");
	pc.printf("fpr half-----------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		fpr res = fpr_half(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr double
	pc.printf("-------------------\n\r");
	pc.printf("fpr double---------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		CALC_START
		fpr res = fpr_double(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for fpr lt
	pc.printf("-------------------\n\r");
	pc.printf("fpr lt-------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		fpr r1  = FPR((double)rand64());
		fpr r2  = fpr_zero;
		if (r % 16 == 0) continue; else r2 = FPR((double)rand64());
		CALC_START
		int res = fpr_lt(r1,r2);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	pc.printf("-------------------------\n\r");
	pc.printf("Testing native rounding--\n\r");
	pc.printf("-------------------------\n\r");
	
	/// function for llrint
	pc.printf("-------------------\n\r");
	pc.printf("llrint-------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		double r1  = (double)rand64();
		CALC_START
		long long int res = llrint(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for llrintf
	pc.printf("-------------------\n\r");
	pc.printf("llrintf------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		float r1  = (float)rand64();
		CALC_START
		long long int res = llrintf(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);
	
	/// function for llrintl
	pc.printf("-------------------\n\r");
	pc.printf("llrintl------------\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		long double r1  = (long double)rand64();
		CALC_START
		long long int res = llrintl(r1);
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %.0Lf\n\r", (average_clk));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));

	fflush(stdout);
	wait(1);

	pc.printf("Testing finished");		
}












