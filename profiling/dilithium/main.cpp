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

#include <stdint.h>
extern "C" {
#include "dilithium-pqm4/params.h"
#include "dilithium-pqm4/sign.h"
#include "dilithium-pqm4/packing.h"
#include "dilithium-pqm4/polyvec.h"
#include "dilithium-pqm4/poly.h"
#include "dilithium-pqm4/randombytes.h"
#include "dilithium-pqm4/symmetric.h"
}

//------------------------------------
// Hyperterminal configuration
// 115200 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalOut myled(LED1);
Timer timer;

///////////////////////////////////////////////////////
// code for taking timing---
///////////////////////////////////////////////////////

#define BENCHMARK_ROUND 125

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define CYCLES_VARS		 \
	uint64_t start, stop, delta; \
//	uint64_t scc_ms = SystemCoreClock / 1000; 
//	uint64_t ofl_ms = 4294967295 / scc_ms;    

#define CYCLES_START {		 \
	start = DWT->CYCCNT;	 \
}

#define CYCLES_ADD(cc) {			    \
	stop         = DWT->CYCCNT;     	    \
	delta        = stop - start;                \
	cc	    += delta;                       \
}

#define CYCLES_VARS_M		 \
	uint64_t start_m, stop_m, delta_m; \
//	uint64_t scc_ms = SystemCoreClock / 1000; 
//	uint64_t ofl_ms = 4294967295 / scc_ms;    

#define CYCLES_START_M {		 \
	start_m = DWT->CYCCNT;	 \
}

#define CYCLES_ADD_M(cc) {			    \
	stop_m       = DWT->CYCCNT;     	    \
	delta_m      = stop_m - start_m;                \
	cc	    += delta_m;                       \
}

//	ms = timer.read_ms() - ms;		    
//	ms -= ((uint64_t) delta) / scc_ms;			
//	cc += ((uint64_t) (ms / ofl_ms)) << 32;	

#define timer_read_ms(x)    chrono::duration_cast<chrono::milliseconds>((x).elapsed_time()).count()

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

int crypto_sign_keypair(uint8_t *pk, uint8_t *sk,
uint64_t *getrand, uint64_t *expmat, uint64_t *sampvec, 
uint64_t *matvecmult, uint64_t *adderr, uint64_t *extwritepk, 
uint64_t *comphsk) {
	uint8_t seedbuf[2*SEEDBYTES + CRHBYTES];
	uint8_t tr[SEEDBYTES];
	const uint8_t *rho, *rhoprime, *key;
	polyvecl mat[K];
	polyvecl s1, s1hat;
	polyveck s2, t1, t0;

	CYCLES_VARS
	timer.reset();
	timer.start();

	//set so that cycle counter can be read from  DWT->CYCCNT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Get randomness for rho, rhoprime and key */
	CYCLES_START
	randombytes(seedbuf, SEEDBYTES);
	shake256(seedbuf, 2*SEEDBYTES + CRHBYTES, seedbuf, SEEDBYTES);
	rho = seedbuf;
	rhoprime = rho + SEEDBYTES;
	key = rhoprime + CRHBYTES;
	CYCLES_ADD(*getrand)

	/* Expand matrix */
	CYCLES_START
	polyvec_matrix_expand(mat, rho);
	CYCLES_ADD(*expmat)

	/* Sample short vectors s1 and s2 */
	CYCLES_START
	polyvecl_uniform_eta(&s1, rhoprime, 0);
	polyveck_uniform_eta(&s2, rhoprime, L);
	CYCLES_ADD(*sampvec)

	/* Matrix-vector multiplication */
	CYCLES_START
	s1hat = s1;
	polyvecl_ntt(&s1hat);
	polyvec_matrix_pointwise_montgomery(&t1, mat, &s1hat);
	polyveck_reduce(&t1);
	polyveck_invntt_tomont(&t1);
	CYCLES_ADD(*matvecmult)

	/* Add error vector s2 */
	CYCLES_START
	polyveck_add(&t1, &t1, &s2);
	CYCLES_ADD(*adderr)

	/* Extract t1 and write public key */
	CYCLES_START
	polyveck_caddq(&t1);
	polyveck_power2round(&t1, &t0, &t1);
	pack_pk(pk, rho, &t1);
	CYCLES_ADD(*extwritepk)

	/* Compute H(rho, t1) and write secret key */
	CYCLES_START
	shake256(tr, SEEDBYTES, pk, CRYPTO_PUBLICKEYBYTES);
	pack_sk(sk, rho, tr, key, &t0, &s1, &s2);
	CYCLES_ADD(*comphsk)

	return 0;
}

int crypto_sign_signature(uint8_t *sig, size_t *siglen, const uint8_t *m,
                          size_t mlen, const uint8_t *sk,
                          uint64_t **compcrh, uint64_t **expmattv, uint64_t **sampy, 
                          uint64_t **matvecmults, uint64_t **decompw, uint64_t **compz, 
                          uint64_t **checkcs2, uint64_t **comphint)
{
	unsigned int n;
	uint8_t seedbuf[3*SEEDBYTES + 2*CRHBYTES];
	uint8_t *rho, *tr, *key, *mu, *rhoprime;
	uint16_t nonce = 0;
	polyvecl mat[K], s1, y, z;
	polyveck t0, s2, w1, w0, h;
	poly cp;
	shake256incctx state;

	CYCLES_VARS
	timer.reset();
	timer.start();

	rho = seedbuf;
	tr = rho + SEEDBYTES;
	key = tr + SEEDBYTES;
	mu = key + SEEDBYTES;
	rhoprime = mu + CRHBYTES;
	unpack_sk(rho, tr, key, &t0, &s1, &s2, sk);

	/* Compute CRH(tr, msg) */
	CYCLES_START
	shake256_inc_init(&state);
	shake256_inc_absorb(&state, tr, SEEDBYTES);
	shake256_inc_absorb(&state, m, mlen);
	shake256_inc_finalize(&state);
	shake256_inc_squeeze(mu, CRHBYTES, &state);
	CYCLES_ADD(**compcrh)
	
	#ifdef DILITHIUM_RANDOMIZED_SIGNING
	randombytes(rhoprime, CRHBYTES);
	#else
	shake256(rhoprime, CRHBYTES, key, SEEDBYTES + CRHBYTES);
	#endif

	/* Expand matrix and transform vectors */
	CYCLES_START
	polyvec_matrix_expand(mat, rho);
	polyvecl_ntt(&s1);
	polyveck_ntt(&s2);
	polyveck_ntt(&t0);
	CYCLES_ADD(**expmattv)
	
	rej:
	/* Sample intermediate vector y */
	CYCLES_START
	polyvecl_uniform_gamma1(&y, rhoprime, nonce++);
	CYCLES_ADD(**sampy)

	/* Matrix-vector multiplication */
	CYCLES_START
	z = y;
	polyvecl_ntt(&z);
	polyvec_matrix_pointwise_montgomery(&w1, mat, &z);
	polyveck_reduce(&w1);
	polyveck_invntt_tomont(&w1);
	CYCLES_ADD(**matvecmults)

	/* Decompose w and call the random oracle */
	CYCLES_START
	polyveck_caddq(&w1);
	polyveck_decompose(&w1, &w0, &w1);
	polyveck_pack_w1(sig, &w1);

	shake256_inc_init(&state);
	shake256_inc_absorb(&state, mu, CRHBYTES);
	shake256_inc_absorb(&state, sig, K*POLYW1_PACKEDBYTES);
	shake256_inc_finalize(&state);
	shake256_inc_squeeze(sig, SEEDBYTES, &state);
	poly_challenge(&cp, sig);
	poly_ntt(&cp);
	CYCLES_ADD(**decompw)

	/* Compute z, reject if it reveals secret */
	CYCLES_START
	polyvecl_pointwise_poly_montgomery(&z, &cp, &s1);
	polyvecl_invntt_tomont(&z);
	polyvecl_add(&z, &z, &y);
	polyvecl_reduce(&z);
	if(polyvecl_chknorm(&z, GAMMA1 - BETA))
	goto rej;
	CYCLES_ADD(**compz)

	/* Check that subtracting cs2 does not change high bits of w and low bits
	* do not reveal secret information */
	CYCLES_START
	polyveck_pointwise_poly_montgomery(&h, &cp, &s2);
	polyveck_invntt_tomont(&h);
	polyveck_sub(&w0, &w0, &h);
	polyveck_reduce(&w0);
	if(polyveck_chknorm(&w0, GAMMA2 - BETA))
	goto rej;
	CYCLES_ADD(**checkcs2)

	/* Compute hints for w1 */
	CYCLES_START
	polyveck_pointwise_poly_montgomery(&h, &cp, &t0);
	polyveck_invntt_tomont(&h);
	polyveck_reduce(&h);
	if(polyveck_chknorm(&h, GAMMA2))
	goto rej;

	polyveck_add(&w0, &w0, &h);
	n = polyveck_make_hint(&h, &w0, &w1);
	if(n > OMEGA)
	goto rej;
	CYCLES_ADD(**comphint)
	
	/* Write signature */
	pack_sig(sig, sig, &z, &h);
	*siglen = CRYPTO_BYTES;
	return 0;
}

/*************************************************
* Name:        crypto_sign
*
* Description: Compute signed message.
*
* Arguments:   - uint8_t *sm: pointer to output signed message (allocated
*                             array with CRYPTO_BYTES + mlen bytes),
*                             can be equal to m
*              - size_t *smlen: pointer to output length of signed
*                               message
*              - const uint8_t *m: pointer to message to be signed
*              - size_t mlen: length of message
*              - const uint8_t *sk: pointer to bit-packed secret key
*
* Returns 0 (success)
**************************************************/
int crypto_sign(uint8_t *sm, size_t *smlen, const uint8_t *m,
                size_t mlen, const uint8_t *sk,
                uint64_t *compcrh, uint64_t *expmattv, uint64_t *sampy, 
                uint64_t *matvecmults, uint64_t *decompw, uint64_t *compz, 
                uint64_t *checkcs2, uint64_t *comphint)
{
  size_t i;

  for(i = 0; i < mlen; ++i)
    sm[CRYPTO_BYTES + mlen - 1 - i] = m[mlen - 1 - i];
  crypto_sign_signature(sm, smlen, sm + CRYPTO_BYTES, mlen, sk,
  &compcrh, &expmattv, &sampy, &matvecmults, &decompw, &compz, &checkcs2, &comphint);
  *smlen += mlen;
  return 0;
}

/*************************************************
* Name:        crypto_sign_verify
*
* Description: Verifies signature.
*
* Arguments:   - uint8_t *m: pointer to input signature
*              - size_t siglen: length of signature
*              - const uint8_t *m: pointer to message
*              - size_t mlen: length of message
*              - const uint8_t *pk: pointer to bit-packed public key
*
* Returns 0 if signature could be verified correctly and -1 otherwise
**************************************************/
int crypto_sign_verify(const uint8_t *sig, size_t siglen, const uint8_t *m,
                       size_t mlen, const uint8_t *pk,
                       uint64_t *compcrhv, uint64_t *matvecmultv, 
                       uint64_t *reconstw1, uint64_t *rovc)
{
	unsigned int i;
	uint8_t buf[K*POLYW1_PACKEDBYTES];
	uint8_t rho[SEEDBYTES];
	uint8_t mu[CRHBYTES];
	uint8_t c[SEEDBYTES];
	uint8_t c2[SEEDBYTES];
	poly cp;
	polyvecl mat[K], z;
	polyveck t1, w1, h;
	shake256incctx state;

	CYCLES_VARS
	timer.reset();
	timer.start();

	if(siglen != CRYPTO_BYTES)
	return -1;

	unpack_pk(rho, &t1, pk);
	if(unpack_sig(c, &z, &h, sig))
	return -1;
	if(polyvecl_chknorm(&z, GAMMA1 - BETA))
	return -1;

	/* Compute CRH(h(rho, t1), msg) */
	CYCLES_START
	shake256(mu, SEEDBYTES, pk, CRYPTO_PUBLICKEYBYTES);
	shake256_inc_init(&state);
	shake256_inc_absorb(&state, mu, SEEDBYTES);
	shake256_inc_absorb(&state, m, mlen);
	shake256_inc_finalize(&state);
	shake256_inc_squeeze(mu, CRHBYTES, &state);
	CYCLES_ADD(*compcrhv)

	/* Matrix-vector multiplication; compute Az - c2^dt1 */
	CYCLES_START
	poly_challenge(&cp, c);
	polyvec_matrix_expand(mat, rho);

	polyvecl_ntt(&z);
	polyvec_matrix_pointwise_montgomery(&w1, mat, &z);

	poly_ntt(&cp);
	polyveck_shiftl(&t1);
	polyveck_ntt(&t1);
	polyveck_pointwise_poly_montgomery(&t1, &cp, &t1);

	polyveck_sub(&w1, &w1, &t1);
	polyveck_reduce(&w1);
	polyveck_invntt_tomont(&w1);
	CYCLES_ADD(*matvecmultv)

	/* Reconstruct w1 */
	CYCLES_START
	polyveck_caddq(&w1);
	polyveck_use_hint(&w1, &w1, &h);
	polyveck_pack_w1(buf, &w1);
	CYCLES_ADD(*reconstw1)
	
	/* Call random oracle and verify challenge */
	CYCLES_START
	shake256_inc_init(&state);
	shake256_inc_absorb(&state, mu, CRHBYTES);
	shake256_inc_absorb(&state, buf, K*POLYW1_PACKEDBYTES);
	shake256_inc_finalize(&state);
	shake256_inc_squeeze(c2, SEEDBYTES, &state);
	for(i = 0; i < SEEDBYTES; ++i)
	if(c[i] != c2[i])
	return -1;
	CYCLES_ADD(*rovc)
	
	return 0;
}

/*************************************************
* Name:        crypto_sign_open
*
* Description: Verify signed message.
*
* Arguments:   - uint8_t *m: pointer to output message (allocated
*                            array with smlen bytes), can be equal to sm
*              - size_t *mlen: pointer to output length of message
*              - const uint8_t *sm: pointer to signed message
*              - size_t smlen: length of signed message
*              - const uint8_t *pk: pointer to bit-packed public key
*
* Returns 0 if signed message could be verified correctly and -1 otherwise
**************************************************/
int crypto_sign_open(uint8_t *m,
                     size_t *mlen,
                     const uint8_t *sm,
                     size_t smlen,
                     const uint8_t *pk)
{
  size_t i;

  if(smlen < CRYPTO_BYTES)
    goto badsig;

  *mlen = smlen - CRYPTO_BYTES;
  if(crypto_sign_verify(sm, CRYPTO_BYTES, sm + CRYPTO_BYTES, *mlen, pk))
    goto badsig;
  else {
    /* All good, copy msg, return 0 */
    for(i = 0; i < *mlen; ++i)
      m[i] = sm[CRYPTO_BYTES + i];
    return 0;
  }

badsig:
  /* Signature verification failed */
  *mlen = -1;
  for(i = 0; i < smlen; ++i)
    m[i] = 0;

  return -1;
}

int main()
{

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

	unsigned logn = 9; // set to 9 for 512 parameters, 10 for 1024
	char seed[16] = {0};
	//shake256_context sc;
	//shake256_init_prng_from_seed(&sc, seed, 16);

	void *pubkey, *privkey, *expkey, *sig, *pubkey2, *sigct;
	size_t pubkey_len, privkey_len, sig_len, expkey_len;
	//shake256_context rng;
	//shake256_context hd;
	//uint8_t nonce[40];
	void *tmpkg, *tmpsd, *tmpmp, *tmpvv, *tmpek, *tmpst;
	size_t tmpkg_len, tmpvv_len, tmpmp_len, tmpsd_len, tmpek_len, tmpst_len;
	
	//set so that cycle counter can be read from  DWT->CYCCNT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	CYCLES_VARS_M
	// variable for main clock counting
	uint64_t kg, sg, vf;
	kg = sg = vf = 0;

	// keygen function ccounts
	uint64_t getrand, expmat, sampvec, matvecmult, adderr, extwritepk, comphsk;
	getrand = expmat = sampvec = matvecmult = adderr = extwritepk = comphsk = 0;
	
	// sign function ccounts
	uint64_t compcrh, expmattv, sampy, matvecmults, decompw, compz, checkcs2, comphint;
	compcrh = expmattv = sampy = matvecmults = decompw = compz = checkcs2 = comphint = 0;
	
	// verify function ccounts
	uint64_t compcrhv, matvecmultv, reconstw1, rovc;
	compcrhv = matvecmultv = reconstw1 = rovc = 0;
	
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
	

	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CYCLES_START_M
		ret_val += crypto_sign_keypair(pk, sk,
		&getrand, &expmat, &sampvec, &matvecmult, 
		&adderr, &extwritepk, &comphsk);
		CYCLES_ADD_M(kg)
	}
	
	uint64_t total = getrand + expmat + sampvec + matvecmult + adderr + extwritepk + comphsk;		

	pc.printf("get randomness clocks:       %lld (%f%%)\n\r", getrand,(double)((double)(getrand)/(double)total)*100);
	pc.printf("expand matrix clocks:        %lld (%f%%)\n\r", expmat/BENCHMARK_ROUND,(double)((double)expmat/(double)total)*100);
	pc.printf("sample vector clocks:        %lld (%f%%)\n\r", sampvec/BENCHMARK_ROUND,(double)((double)sampvec/(double)total)*100);
	pc.printf("matrix/vector mult clocks:   %lld (%f%%)\n\r", matvecmult/BENCHMARK_ROUND,(double)((double)matvecmult/(double)total)*100);
	pc.printf("add error clocks:            %lld (%f%%)\n\r", adderr/BENCHMARK_ROUND,(double)((double)adderr/(double)total)*100);
	pc.printf("expand/write pub key clocks: %lld (%f%%)\n\r", extwritepk/BENCHMARK_ROUND,(double)((double)extwritepk/(double)total)*100);
	pc.printf("get h/comp priv key clocks:  %lld (%f%%)\n\r", comphsk/BENCHMARK_ROUND,(double)((double)comphsk/(double)total)*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("total measured clocks:       %lld (%f%%)\n\r", total/BENCHMARK_ROUND,(double)((double)total/(double)total)*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("total keygen (%%meas) clocks: %lld (%f%%)\n\r", kg/BENCHMARK_ROUND, (double)((double)(total/BENCHMARK_ROUND)/(double)(kg/BENCHMARK_ROUND))*100);
	pc.printf("---------------------------------\n\r");
	
	pc.printf("-----------------\n\r");
	pc.printf("| Doing Signing |\n\r");
	pc.printf("-----------------\n\r");
	
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CYCLES_START_M
		ret_val += crypto_sign(sm, &smlen, m, MLEN, sk,
		&compcrh, &expmattv, &sampy, &matvecmults, &decompw, 
		&compz, &checkcs2, &comphint);
		CYCLES_ADD_M(sg)	
	}
	
	total = compcrh + expmattv + sampy + matvecmults + decompw + compz + checkcs2 + comphint;		

	pc.printf("compute crh clocks:          %lld (%f%%)\n\r", compcrh,(double)((double)(compcrh)/(double)total)*100);
	pc.printf("exp mat/transf vecs clocks:  %lld (%f%%)\n\r", expmattv/BENCHMARK_ROUND,(double)((double)expmattv/(double)total)*100);
	pc.printf("sample y vector clocks:      %lld (%f%%)\n\r", sampy/BENCHMARK_ROUND,(double)((double)sampy/(double)total)*100);
	pc.printf("matrix/vector mult clocks:   %lld (%f%%)\n\r", matvecmults/BENCHMARK_ROUND,(double)((double)matvecmults/(double)total)*100);
	pc.printf("decomp w/ call RO clocks:    %lld (%f%%)\n\r", decompw/BENCHMARK_ROUND,(double)((double)decompw/(double)total)*100);
	pc.printf("compute z clocks:            %lld (%f%%)\n\r", compz/BENCHMARK_ROUND,(double)((double)compz/(double)total)*100);
	pc.printf("check cs2 clocks:            %lld (%f%%)\n\r", checkcs2/BENCHMARK_ROUND,(double)((double)checkcs2/(double)total)*100);
	pc.printf("compute hint clocks:         %lld (%f%%)\n\r", comphint/BENCHMARK_ROUND,(double)((double)comphint/(double)total)*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("total measured clocks:       %lld (%f%%)\n\r", total/BENCHMARK_ROUND,(double)((double)total/(double)total)*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("total keygen (%%meas) clocks: %lld (%f%%)\n\r", sg/BENCHMARK_ROUND, (double)((double)(total/BENCHMARK_ROUND)/(double)(kg/BENCHMARK_ROUND))*100);
	pc.printf("---------------------------------\n\r");

	pc.printf("-------------------\n\r");
	pc.printf("| Doing Verifying |\n\r");
	pc.printf("-------------------\n\r");
	
	for (size_t r=0; r<BENCHMARK_ROUND; r++) {
		CYCLES_START_M	
		ret_val += crypto_sign_verify(sm, CRYPTO_BYTES, m, MLEN, pk,
		&compcrhv, &matvecmultv, &reconstw1, &rovc);
		CYCLES_ADD_M(vf)
	}
	
	total = compcrhv + matvecmultv + reconstw1 + rovc;		

	pc.printf("compute crh clocks:          %lld (%f%%)\n\r", compcrhv,(double)((double)(compcrhv)/(double)total)*100);
	pc.printf("matrix/vector mult clocks:   %lld (%f%%)\n\r", matvecmultv/BENCHMARK_ROUND,(double)((double)matvecmultv/(double)total)*100);
	pc.printf("reconstruct w1 clocks:       %lld (%f%%)\n\r", reconstw1/BENCHMARK_ROUND,(double)((double)reconstw1/(double)total)*100);
	pc.printf("call ro verify chall clocks: %lld (%f%%)\n\r", rovc/BENCHMARK_ROUND,(double)((double)rovc/(double)total)*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("total measured clocks:       %lld (%f%%)\n\r", total/BENCHMARK_ROUND,(double)((double)total/(double)total)*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("total keygen (%%meas) clocks: %lld (%f%%)\n\r", vf/BENCHMARK_ROUND, (double)((double)(total/BENCHMARK_ROUND)/(double)(vf/BENCHMARK_ROUND))*100);
	pc.printf("---------------------------------\n\r");
	pc.printf("Dilithium failed if nonzero: %d\n\r", ret_val);
	pc.printf("----------------------\n\r");
	pc.printf("| Dilithium Finished |\n\r");
	pc.printf("----------------------\n\r");
	
	fflush(stdout);
	
}
