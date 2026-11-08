/* Copyright  (C) 2026 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (ec_first_use_test.c).
 * ---------------------------------------------------------------------------------------
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* The P-256 and P-384 constants, set up on first use, from several
 * threads at once, under TSan: every thread's first key comes out the
 * same. Built with EC_TEST_HOLD_LANE, the thread setting the constants
 * up is held in the middle of it, and another thread's first use must
 * still finish: no thread waits on another. EC_TEST_WAIT builds the
 * old wait for the constants instead, and that must fail. */

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#ifdef EC_TEST_HOLD_LANE
static volatile int hold_on, held, let_go;
#define EC_TEST_HOLD() do { if (hold_on && !held) { held = 1; \
   while (!let_go) sched_yield(); } } while (0)
/* The library's own source, for the hook in the middle of the set-up */
#include "../../crypto/pk.c"
#else
#include <crypto/pk.h>
#endif

#define NTHREADS 8

static const uint8_t priv[32] = {
   0xc9, 0xaf, 0xa9, 0xd8, 0x45, 0xba, 0x75, 0x16, 0x6b, 0x5c, 0x21, 0x57,
   0x67, 0xb1, 0xd6, 0x93, 0x4e, 0x50, 0xc3, 0xdb, 0x36, 0xe8, 0x9b, 0x12,
   0x7b, 0x8a, 0x62, 0x2b, 0x12, 0x0f, 0x67, 0x21 };
static uint8_t pub256[NTHREADS][65];
static pthread_barrier_t go;

static void *first_use(void *arg)
{
   size_t n = (size_t)arg;
   pthread_barrier_wait(&go);
   if (p256_keygen(priv, pub256[n]) != 0)
      memset(pub256[n], 0xff, sizeof(pub256[n]));
   return NULL;
}

#ifdef EC_TEST_HOLD_LANE
static uint8_t held_pub[49 * 2 + 1], other_pub[49 * 2 + 1];
static void *held_use(void *arg)
{
   (void)arg;
   p384_keygen(priv, held_pub);
   return NULL;
}

static void *other_use(void *arg)
{
   (void)arg;
   p384_keygen(priv, other_pub);
   return NULL;
}
#endif

int main(void)
{
   pthread_t t[NTHREADS];
   size_t    i;
   int       fails = 0;

   pthread_barrier_init(&go, NULL, NTHREADS);
   for (i = 0; i < NTHREADS; i++)
      pthread_create(&t[i], NULL, first_use, (void*)i);
   for (i = 0; i < NTHREADS; i++)
      pthread_join(t[i], NULL);
   pthread_barrier_destroy(&go);
   for (i = 1; i < NTHREADS; i++)
      if (memcmp(pub256[i], pub256[0], sizeof(pub256[0])) != 0)
      {
         printf("FAIL: thread %u's first P-256 key differs\n", (unsigned)i);
         fails++;
      }
   if (!fails)
      printf("ok:   %d first uses of P-256 at once give one key\n", NTHREADS);

#ifdef EC_TEST_HOLD_LANE
   {
      pthread_t h, o;
      struct timespec ts;
      int done;
      hold_on = 1;
      pthread_create(&h, NULL, held_use, NULL);
      while (!held)
         sched_yield();
      pthread_create(&o, NULL, other_use, NULL);
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 2;
      done = pthread_timedjoin_np(o, NULL, &ts) == 0;
      let_go = 1;
      pthread_join(h, NULL);
      if (!done)
         pthread_join(o, NULL);
#ifdef EC_TEST_WAIT
      if (done)
      {
         printf("FAIL: a first use finished while waiting for a held set-up\n");
         fails++;
      }
      else
         printf("ok:   the waiting set-up kept a first use behind a held one\n");
#else
      if (!done || memcmp(held_pub, other_pub, sizeof(held_pub)) != 0)
      {
         printf("FAIL: a first use of P-384 waited on, or differed from, a held one\n");
         fails++;
      }
      else
         printf("ok:   a first use of P-384 finished while another's set-up was held\n");
#endif
   }
#endif
   return fails ? 1 : 0;
}
