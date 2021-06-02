#include <string>

#include <stdlib.h>
#include "mbed.h"
#include "stm32f7xx_hal.h"

#include <array>

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

double fmean(uint32_t values[], int n)
{
    int sum = 0;
    for (int i=0; i<n; i++) {
        sum += values[i];
    }
    return sum / n;
}

double fvar(uint32_t values[], int n)
{
    int valuesMean = fmean(values, n);
    int sum = 0;
    for (int i=0; i<n; i++) {
        sum += (values[i] - valuesMean) * (values[i] - valuesMean);
    }
    return sum / (n-1);
}

double fmin(uint32_t values[], int n) {
    double minimum = values[0];
    for (int i=1; i<n; i++) {
        if (minimum > values[i]) {
            minimum = values[i];
        }
    }
    return minimum;
}

double fmax(uint32_t values[], int n) {
    double maximum = values[0];
    for (int i=1; i<n; i++) {
        if (maximum < values[i]) {
            maximum = values[i];
        }
    }
    return maximum;
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
  uint32_t r = 0;
  for (int i = 0; i < 32; i += RAND_MAX_WIDTH) {
    r <<= RAND_MAX_WIDTH;
    r ^= (unsigned) rand();
  }
  return r;
}

uint16_t rand16(void) {
  uint16_t r = 0;
  for (int i = 0; i < 16; i += RAND_MAX_WIDTH) {
    r <<= RAND_MAX_WIDTH;
    r ^= (unsigned) rand();
  }
  return r;
}

void clobber() {
  __asm__ __volatile__ ("" : : : "memory");
}

template <typename T>
void use(T const& val) {
  asm volatile("" : : "m"(val) : "memory");
}

int64_t cast(double a) {
    union {
        double d;
        uint64_t u;
        int64_t i;
    } x;
    uint64_t mask;
    uint32_t high, low;
    x.d = a;
    mask =  x.i >> 63;
    x.u &= 0x7fffffffffffffffL;
    high = x.d / 4294967296.f;               // a / 0x1p32f;
    low = x.d - (double)high * 4294967296.f; // high * 0x1p32f;
    x.u = ((int64_t)high << 32) | low;
    return (x.u & ((uint64_t)-1 - mask)) | ((-x.u) & mask);
}

int main()
{

	///////////////////////////////////////////////////////
	// code for taking timing---
	///////////////////////////////////////////////////////

	uint64_t start, stop, delta, min, max, sum, sum_us, sum_squared, mean;
	int us;
	long double var, std_err;

	#define MIN(a,b) (((a)<(b))?(a):(b))
	#define MAX(a,b) (((a)>(b))?(a):(b))

	#define CALC_RESET {		 \
		delta       = 0;         \
		var         = 0;         \
		sum         = 0;	 \
		sum_us      = 0;	 \
		sum_squared = 0;         \
		mean        = 0;        \
		min         = 9999999999;\
		max         = 0;         \
		timer.reset();		 \
	}
	
	#define CALC_START {		 \
		timer.reset();		 \
		timer.start();		 \
		start = DWT->CYCCNT;	 \
	}
	
	#define CALC_STOP {		 		   \
		stop         = DWT->CYCCNT;    \
		us           = timer.read_us();\
		delta        = stop - start;   \
		sum         += delta;	       \
		sum_squared += delta * delta;  \
		sum_us      += us;	           \
		min          = MIN(delta,min); \
		max          = MAX(delta,max); \
	}
	
	#define CALC_AVG {				   \
		mean         = sum/rounds;     \
		var          = (sum_squared - ((sum*sum)/rounds))/rounds;\
		std_err      = var/sqrt(rounds);  \
	}

	#define rounds 1000
	#define dummy 50

	#define timer_read_ms(x)    chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()

	//set so that cycle counter can be read from DWT->CYCCNT
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


	/// function for 32->64 mult
	pc.printf("-------------------\n\r");
	pc.printf("casting-\n\r");
	CALC_RESET
	for(size_t r=0; r<rounds; r++){ 
		double r1  = (double)(rand64());
//		if (r % 16 != 0) {} else r1=0;
		CALC_START
		double res = cast(r1);
		use(&res);		
		CALC_STOP
	}	
	CALC_AVG
	pc.printf("Avg clock cycles:        %lld\n\r", (mean));
	pc.printf("Min clock cycles:        %lld\n\r", min);
	pc.printf("Max clock cycles:        %lld\n\r", max);
	pc.printf("Std dev of clock cycles: %.1Lf\n\r", (sqrt(var)));
	pc.printf("Std err of clock cycles: %.1Lf\n\r", (std_err));


	/// function for sqrt root
	pc.printf("-------------------\n\r");
	pc.printf("Testing vcvt f->u--\n\r");
	pc.printf("-------------------\n\r");
	uint32_t cycles_total[rounds] = {0};
	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		double r1 = (double)rand64();
		if (r % 31 != 0) {} else r1 = 0;
		if (r % 37 != 0) {} else r1 = (double)(1<<(rand64()%63));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d5, %2\n"
			"ldr r1, %1\n"                                                                   
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"vcvt.u32.f64 s5, d5\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1) : "r0", "r1", "r2", "d5");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));
	
	fflush(stdout);
	wait(1);

	pc.printf("-------------------\n\r");
	pc.printf("Testing vcvt u->f--\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		int32_t r1 = rand32();
		if (r % 31 != 0) {} else r1 = 0;
		if (r % 37 != 0) {} else r1 = (1<<(rand32()%31));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr s5, %2\n"
			"ldr r1, %1\n"                                                                   
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"vcvt.f64.u32 d5, s5\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1) : "r0", "r1", "r2", "s5");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));
	
	fflush(stdout);
	wait(1);


	pc.printf("-------------------\n\r");
	pc.printf("Testing vcmpe.f64--\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		int32_t r1 = rand32();
		int32_t r2 = rand32();
		if (r % 31 != 0) {} else r1 = 0;
		if (r % 37 != 0) {} else r1 = (1<<(rand32()%31));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d5, %2\n"
			"vldr d6, %2\n"
			"ldr r1, %1\n"                                                                   
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"vcmpe.f64 d5, d6\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "d5", "d6");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));
	
	fflush(stdout);
	wait(1);
	

	pc.printf("-------------------------\n\r");
	pc.printf("Testing AND--------------\n\r");
	pc.printf("-------------------------\n\r");

	cycles_total[rounds] = {0};
	for(size_t r=0; r<rounds; r++){ 
		int64_t r1=rand64(), r2=rand64();
		if (r % 3  == 0) {} else r1 = -r1;
		if (r % 2  == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = 0;
		if (r % 37 != 0) {} else r2 = 1<<(rand64()%31);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r4, %2\n"
			"ldr r5, %3\n"
			"ldr r1, %1\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"and r4, r5, r6\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "r3", "r4", "r5", "r6");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);
	
	pc.printf("-------------------------\n\r");
	pc.printf("Testing XOR--------------\n\r");
	pc.printf("-------------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		int64_t r1=rand64(), r2=rand64();
		if (r % 3  == 0) {} else r1 = -r1;
		if (r % 2  == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = 0;
		if (r % 37 != 0) {} else r2 = 1<<(rand64()%31);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r4, %2\n"
			"ldr r5, %3\n"
			"ldr r1, %1\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"eor r4, r5, r6\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "r3", "r4", "r5", "r6");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing vadd.f32---------\n\r");
	pc.printf("-------------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		float r1=(float)(rand32()/3), r2=(float)(rand32()/3);
		if (r % 31 != 0) {} else r2 = 0;
		if (r % 37 != 0) {} else r2 = (float)(1<<(rand32()%15));		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr s5, %2\n"
			"vldr s6, %3\n"
			"ldr r1, %1\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"vadd.f32 s4, s5, s6\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing vadd.f64---------\n\r");
	pc.printf("-------------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		double r1=(double)(rand64()/3), r2=(double)(rand64()/3);
//		if (r % 31 != 0) {} else r2 = 0;
//		if (r % 37 != 0) {} else r2 = (double)(1<<(rand64()%31));		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d5, %2\n"
			"vldr d6, %3\n"
			"dmb\n"
			"isb\n"
			"ldr r1, %1\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"vadd.f64 d4, d5, d6\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "r3", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1lf\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1lf\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing vsub.f32---------\n\r");
	pc.printf("-------------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		float r1=(float)(rand32()/3), r2=(float)(rand32()/3);
//		if (r % 31 != 0) {} else r2 = (float)0;
		if (r % 37 != 0) {} else r2 = (float)(1<<(rand32()%31));		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr s5, %2\n"
			"vldr s6, %3\n"
			"ldr r1, %1\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"vsub.f32 s4, s5, s6\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "s0", "s1", "s2", "s3", "s4", "s5", "s6");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing vsub.f64---------\n\r");
	pc.printf("-------------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		double r1=(double)rand64(), r2=(double)rand64();
//		if (r % 31 != 0) {} else r2 = 0;
//		if (r % 37 != 0) {} else r2 = (double)(1<<(rand64()%63));		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d4, %2\n"
			"vldr d5, %2\n"
			"vldr d6, %3\n"
			"vadd.f64 d4, d4\n"
			"vadd.f64 d5, d5\n"
			"vadd.f64 d6, d6\n"
			"dmb\n"
			"isb\n"
			"ldr r1, %1\n"
			"vsub.f64 d4, d5, d6\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"vsub.f64 d4, d5, d4\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "r3", "d4", "d5", "d6");
		cycles_total[r] = {cycles};
		pc.printf("Clock cycles:        %ld\n\r", cycles_total[r]);
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);
	
	// function for 32-bit integer multiplication
	pc.printf("-------------------\n\r");
	pc.printf("Testing mul--------\n\r");
	pc.printf("-------------------\n\r");
	
	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		int32_t r1=rand32(), r2=rand32();
		if (r % 3  == 0) {} else r1 = -r1;
		if (r % 2  == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = rand32();
		if (r % 37 != 0) {} else r2 = 1<<(rand32()%32);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r5, %2\n"
			"ldr r6, %3\n"
			"ldr r1, %1\n"
			"mul r4, r5, r6\n"
			"mul r4, r5, r6\n"
			"mul r4, r5, r6\n"
			"mul r4, r5, r6\n"
			"mul r4, r5, r6\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "r4", "r5", "r6");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	// function for 32-bit unsigned integer unsigned long multiply
	pc.printf("-------------------\n\r");
	pc.printf("Testing umull------\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1=rand32(), r2=rand32(), r3=rand32(), r4=rand32();
		if (r % 31 != 0) {} else r2 = rand32();
		if (r % 37 != 0) {} else r2 = 1<<(rand32()%32);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r5, %2\n"
			"ldr r6, %3\n"
			"ldr r7, %4\n"
			"ldr r8, %5\n"
			"ldr r1, %1\n"
			"umull r5, r6, r7, r8\n"
			"umull r5, r6, r7, r8\n"
			"umull r5, r6, r7, r8\n"
			"umull r5, r6, r7, r8\n"
			"umull r5, r6, r7, r8\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2), "m"(r3), "m"(r4) : 
			"r1", "r2", "r4", "r5", "r6", "r7", "r8");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);
	
	// function for 32-bit unsigned integer multiply and accumulate
	pc.printf("-------------------\n\r");
	pc.printf("Testing mla--------\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1=rand32(), r2=rand32(), r3=rand32(), r4=rand32();
		if (r % 3  == 0) {} else r1 = -r1;
		if (r % 2  == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = rand32();
		if (r % 37 != 0) {} else r2 = 1<<(rand32()%32);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r5, %2\n"
			"ldr r6, %3\n"
			"ldr r7, %4\n"
			"ldr r8, %5\n"
			"ldr r1, %1\n"
			"mla r5, r6, r7, r8\n"
			"mla r5, r6, r7, r8\n"
			"mla r5, r6, r7, r8\n"
			"mla r5, r6, r7, r8\n"
			"mla r5, r6, r7, r8\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2), "m"(r3), "m"(r4) : 
			"r1", "r2", "r4", "r5", "r6", "r7", "r8");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	// function for 32-bit floating point multiplication
	pc.printf("-------------------\n\r");
	pc.printf("Testing vmul.f32---\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		float r1=(float)rand32(), r2=(float)rand32();
		if (r % 3  == 0) {} else r1 = -r1;
		if (r % 2  == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = (float)rand32();
		if (r % 37 != 0) {} else r2 = (float)(1<<(rand32()%15));		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr.f32 s0, %2\n"
			"vldr.f32 s1, %3\n"
			"ldr r1, %1\n"
   			"vmul.f32 s0, s1, s2\n"
   			"vmul.f32 s0, s1, s2\n"
   			"vmul.f32 s0, s1, s2\n"
   			"vmul.f32 s0, s1, s2\n"
   			"vmul.f32 s0, s1, s2\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "s0", "s1", "s2");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	// function for 64-bit integer multiplication
	pc.printf("-------------------\n\r");
	pc.printf("Testing vmul.f64---\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		double r1=(double)rand64(), r2=(double)rand64();
		if (r % 3  == 0) {} else r1 = -r1;
		if (r % 2  == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = (double)rand64();
		if (r % 37 != 0) {} else r2 = (double)(1<<(rand64()%31));		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d0, %2\n"
			"vldr d1, %3\n"
			"ldr r1, %1\n"
   			"vmul.f64 d0, d1, d2\n"
   			"vmul.f64 d0, d1, d2\n"
   			"vmul.f64 d0, d1, d2\n"
   			"vmul.f64 d0, d1, d2\n"
   			"vmul.f64 d0, d1, d2\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "d0", "d1", "d2");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing division---------\n\r");
	pc.printf("-------------------------\n\r");

//	/// function for 32-bit division
//	pc.printf("-------------------\n\r");
//	pc.printf("Testing sdiv-------\n\r");
//	pc.printf("-------------------\n\r");
//	
//	memset(cycles_total, 0, sizeof(cycles_total));
//	for(size_t r=0; r<rounds; r++){ 
//		int32_t r1 = rand32(), r2 = rand32();
//		if (r % 3  == 0) {} else r1 = -r1;
//		if (r % 2  == 0) {} else r2 = -r2;
//		if (r % 31 != 0) {} else r2 = rand32();
//		if (r % 37 != 0) {} else r2 = (1<<(rand32()%15));
//		uint32_t cycles = 0;
//		timer.reset();
//		timer.start();
//		asm volatile (
//			"ldr r3, %2\n"
//			"ldr r4, %3\n"
//			"ldr r1, %1\n"                                                                   
//   			"sdiv r5, r3, r4\n"
//   			"sdiv r5, r3, r4\n"
//   			"sdiv r5, r3, r4\n"
//   			"sdiv r5, r3, r4\n"
//   			"sdiv r5, r3, r4\n"
//   			"ldr r2, %1\n"
//   			"subs %0, r2, r1\n" 
//   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r0", "r1", "r2", "r3", "r4", "r5");
//		cycles_total[r] = {cycles};	
//	}	
//	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
//	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
//	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
//	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
//	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
//	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

//	fflush(stdout);
//	wait(1);

//	/// function for 32-bit division
//	pc.printf("-------------------\n\r");
//	pc.printf("Testing udiv-------\n\r");
//	pc.printf("-------------------\n\r");
//	
//	memset(cycles_total, 0, sizeof(cycles_total));
//	for(size_t r=0; r<rounds; r++){ 
//		uint32_t r1 = rand32(), r2 = rand32();
////		if (r % 37 != 0) {} else r2 = (1<<(rand32()%15));
//		uint32_t cycles = 0;
//		timer.reset();
//		timer.start();
//		asm volatile (
//			"ldr r3, %2\n"
//			"ldr r4, %3\n"
//			"ldr r1, %1\n"                                                                   
//   			"udiv r5, r3, r4\n"
//   			"udiv r5, r3, r4\n"
//   			"udiv r5, r3, r4\n"
//   			"udiv r5, r3, r4\n"
//   			"udiv r5, r3, r4\n"
//   			"ldr r2, %1\n"
//   			"subs %0, r2, r1\n" 
//   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "r3", "r4", "r5");
//		cycles_total[r] = {cycles};	
//	}	
//	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
//	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
//	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
//	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
//	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
//	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

//	fflush(stdout);
//	wait(1);

	
	
	/// function for 32-bit division
	pc.printf("-------------------\n\r");
	pc.printf("Testing vdiv.f32---\n\r");
	pc.printf("-------------------\n\r");
	
	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		float r1 = (float)rand32(), r2 = (float)rand32();
		if (r % 31 != 0) {} else r2 = (float)rand32();
		if (r % 37 != 0) {} else r2 = (float)(1<<(rand32()%15));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr s1, %2\n"
			"vldr s2, %3\n"
			"ldr r1, %1\n"                                                                   
   			"vdiv.f32 s0, s1, s2\n"
   			"vdiv.f32 s0, s1, s2\n"
   			"vdiv.f32 s0, s1, s2\n"
   			"vdiv.f32 s0, s1, s2\n"
   			"vdiv.f32 s0, s1, s2\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "s0", "s1", "s2");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);
	
	/// function for 64-bit division
	pc.printf("-------------------\n\r");
	pc.printf("Testing vdiv.f64---\n\r");
	pc.printf("-------------------\n\r");
	
	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		double r1 = (double)rand64(), r2 = (double)rand64();
		if (r % 31 != 0) {} else r2 = (double)rand64();
		if (r % 37 != 0) {} else r2 = (double)(1<<(rand64()%31));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d1, %2\n"
			"vldr d2, %3\n"
			"ldr r1, %1\n"                                                                   
   			"vdiv.f64 d0, d1, d2\n"
   			"vdiv.f64 d0, d1, d2\n"
   			"vdiv.f64 d0, d1, d2\n"
   			"vdiv.f64 d0, d1, d2\n"
   			"vdiv.f64 d0, d1, d2\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "d0", "d1", "d2");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));
	
	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing right-shift------\n\r");
	pc.printf("-------------------------\n\r");
	
	// function for 32-bit unsigned integer right shift
	pc.printf("-------------------\n\r");
	pc.printf("Testing lsrs-------\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));	
	for(size_t r=0; r<rounds; r++){ 
		uint32_t r1=rand32(), r2=rand32();
		if (r % 31 != 0) {} else r2 = rand32();
		if (r % 37 != 0) {} else r2 = 1<<(rand32()%32);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r3, %2\n"
			"ldr r4, %3\n"
			"ldr r1, %1\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"	
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"
			"lsrs r5, r4, r3\n"	
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "r3", "r4", "r5");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	// function for 32-bit signed integer right shift
	pc.printf("-------------------\n\r");
	pc.printf("Testing asrs-------\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));	
	for(size_t r=0; r<rounds; r++){ 
		int32_t r1=rand32(), r2=rand32();
		if (r % 3 == 0) {} else r2 = -r2;
		if (r % 5 == 0) {} else r2 = -r2;
		if (r % 31 != 0) {} else r2 = rand32();
		if (r % 37 != 0) {} else r2 = 1<<(rand32()%32);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r3, %2\n"
			"ldr r4, %3\n"
			"ldr r1, %1\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"asrs r5, r4, r3\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "r3", "r4", "r5");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing left-shift-------\n\r");
	pc.printf("-------------------------\n\r");
	
	// function for 32-bit unsigned integer right shift
	pc.printf("-------------------\n\r");
	pc.printf("Testing lsls-------\n\r");
	pc.printf("-------------------\n\r");

	memset(cycles_total, 0, sizeof(cycles_total));	
	for(size_t r=0; r<rounds; r++){ 
		int32_t r1=rand32(), r2=rand32();
		if (r % 31 != 0) {} else r2 = rand32();
		if (r % 37 != 0) {} else r2 = 1<<(rand32()%32);		
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"ldr r3, %2\n"
			"ldr r4, %3\n"
			"ldr r1, %1\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"lsls r5, r4, r3\n"
			"ldr r2, %1\n"
			"subs %0, r2, r1\n"
			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1), "m"(r2) : "r1", "r2", "r3", "r4", "r5");
		cycles_total[r] = {cycles};
	}
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));

	fflush(stdout);
	wait(1);

	pc.printf("-------------------------\n\r");
	pc.printf("Testing square root------\n\r");
	pc.printf("-------------------------\n\r");

	/// function for sqrt root
	pc.printf("-------------------\n\r");
	pc.printf("Testing vsqrt.f32--\n\r");
	pc.printf("-------------------\n\r");
	
	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		float r1 = (float)rand32();
		if (r % 37 != 0) {} else r1 = (float)(1<<(rand32()%15));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr s1, %2\n"
			"ldr r1, %1\n"                                                                   
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"vsqrt.f32 s0, s1\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1) : "r1", "r2", "s0", "s1");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));
	
	fflush(stdout);
	wait(1);

	/// function for sqrt root
	pc.printf("-------------------\n\r");
	pc.printf("Testing vsqrt.f64--\n\r");
	pc.printf("-------------------\n\r");
	
	memset(cycles_total, 0, sizeof(cycles_total));
	for(size_t r=0; r<rounds; r++){ 
		double r1 = (double)rand64();
		if (r % 37 != 0) {} else r1 = (double)(1<<(rand64()%31));
		uint32_t cycles = 0;
		timer.reset();
		timer.start();
		asm volatile (
			"vldr d1, %2\n"
			"ldr r1, %1\n"                                                                   
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"vsqrt.f64 d0, d1\n"
   			"ldr r2, %1\n"
   			"subs %0, r2, r1\n" 
   			: "=r"(cycles) : "m"(DWT->CYCCNT), "m"(r1) : "r0", "r1", "r2", "d0", "d1");
		cycles_total[r] = {cycles};	
	}	
	memmove(&cycles_total[0], &cycles_total[dummy], (rounds -dummy) * sizeof(cycles_total[0]));
	pc.printf("Avg clock cycles:        %.1F\n\r", fmean(cycles_total, rounds-dummy));
	pc.printf("Min clock cycles:        %.1F\n\r", fmin(cycles_total,  rounds-dummy));
	pc.printf("Max clock cycles:        %.1F\n\r", fmax(cycles_total,  rounds-dummy));
	pc.printf("Std dev of clock cycles: %.1f\n\r", sqrt(fvar(cycles_total,rounds-dummy)));
	pc.printf("Std err of clock cycles: %.1f\n\r", fvar(cycles_total,rounds-dummy)/sqrt(rounds-dummy));
	
	fflush(stdout);
	wait(1);
	
}


