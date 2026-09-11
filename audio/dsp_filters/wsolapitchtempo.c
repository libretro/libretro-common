/* Copyright  (C) 2010-2026 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (wsolapitchtempo.c).
 * ---------------------------------------------------------------------------------------
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* Independent pitch and tempo: a WSOLA duration change followed by a
 * polyphase windowed-sinc resampler. */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <libretro_dspfilter.h>

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define WSOLA_HAVE_SSE2 1
#else
#define WSOLA_HAVE_SSE2 0
#endif

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define WSOLA_HAVE_NEON 1
#else
#define WSOLA_HAVE_NEON 0
#endif

#define WSOLA_CH      2u
#define WSOLA_PI      3.141592653589793238462643383279502884
#define WSOLA_TAPS    16u
#define WSOLA_HALF    (WSOLA_TAPS / 2u)
#define WSOLA_PHASES  1024u
#define WSOLA_BLOCK   8192u
#define WSOLA_SIZE_MAX ((size_t)-1)

enum wsola_simd
{
   WSOLA_SIMD_SCALAR = 0,
   WSOLA_SIMD_SSE2,
   WSOLA_SIMD_NEON
};

struct wsola
{
   double pitch_st, tempo_pct, pitch_ratio, tempo_ratio, duration_ratio;
   double ha, next_analysis;
   double resamp_pos;

   float *input, *mono, *reference;
   float *ola;
   float *table, *resamp;
   float *output;
   /* int16 entry point: converted input and output copies. */
   float *in_f;
   int16_t *out_i16;

   uint64_t synth_start;
   uint64_t input_base, input_write;
   uint64_t ola_base, ola_read, ola_end;
   uint64_t resamp_base, resamp_write;

   size_t input_cap_samples, mono_cap_frames;
   size_t ola_cap_samples;
   size_t resamp_cap_samples;
   size_t output_cap_samples;
   size_t in_f_cap_samples, out_i16_cap_samples;

   float rate;
   unsigned seq, overlap, search, hs;
   unsigned output_frames;
   int first, wsola_bypass, resamp_bypass, failed;
   enum wsola_simd simd;
};

static double wsola_clampd(double x, double lo, double hi)
{
   return x < lo ? lo : (x > hi ? hi : x);
}

static size_t wsola_next_pow2(size_t n)
{
   size_t p = 1;
   while (p < n && p <= (WSOLA_SIZE_MAX >> 1))
      p <<= 1;
   return p < n ? n : p;
}

static int wsola_grow(void **ptr, size_t elem, size_t *cap, size_t need)
{
   void *p;
   size_t nc;
   if (need <= *cap)
      return 1;
   nc = wsola_next_pow2(need < 256 ? 256 : need);
   if (nc > WSOLA_SIZE_MAX / elem)
      return 0;
   if (!(p = realloc(*ptr, nc * elem)))
      return 0;
   *ptr = p;
   *cap = nc;
   return 1;
}

static unsigned wsola_align_up(unsigned v, unsigned a)
{
   return ((v + a - 1u) / a) * a;
}

static int wsola_append_output(struct wsola *d, const float *src,
      unsigned frames)
{
   size_t old = (size_t)d->output_frames * WSOLA_CH;
   size_t add = (size_t)frames * WSOLA_CH;
   if (!frames)
      return 1;
   if (!wsola_grow((void**)&d->output, sizeof(float),
            &d->output_cap_samples, old + add))
      return 0;
   memcpy(d->output + old, src, add * sizeof(float));
   d->output_frames += frames;
   return 1;
}

static double wsola_corr_scalar(const float *a, const float *b,
      unsigned n, double ae)
{
   unsigned i;
   double dot = 0.0, be = 0.0;
   for (i = 0; i < n; ++i)
   {
      double x = a[i], y = b[i];
      dot     += x * y;
      be      += y * y;
   }
   return dot / sqrt(ae * be + 1.0e-30);
}

#if WSOLA_HAVE_SSE2
static double wsola_corr_sse2(const float *a, const float *b,
      unsigned n, double ae)
{
   float td[4], te[4];
   double dot, be;
   unsigned i = 0;
   __m128 vd  = _mm_setzero_ps();
   __m128 ve  = _mm_setzero_ps();
   for (; i + 4u <= n; i += 4u)
   {
      __m128 x = _mm_loadu_ps(a + i);
      __m128 y = _mm_loadu_ps(b + i);
      vd       = _mm_add_ps(vd, _mm_mul_ps(x, y));
      ve       = _mm_add_ps(ve, _mm_mul_ps(y, y));
   }
   _mm_storeu_ps(td, vd);
   _mm_storeu_ps(te, ve);
   dot = (double)td[0] + td[1] + td[2] + td[3];
   be  = (double)te[0] + te[1] + te[2] + te[3];
   for (; i < n; ++i)
   {
      double x = a[i], y = b[i];
      dot     += x * y;
      be      += y * y;
   }
   return dot / sqrt(ae * be + 1.0e-30);
}
#endif

#if WSOLA_HAVE_NEON
static double wsola_corr_neon(const float *a, const float *b,
      unsigned n, double ae)
{
   float td[4], te[4];
   double dot, be;
   unsigned i     = 0;
   float32x4_t vd = vdupq_n_f32(0.0f);
   float32x4_t ve = vdupq_n_f32(0.0f);
   for (; i + 4u <= n; i += 4u)
   {
      float32x4_t x = vld1q_f32(a + i);
      float32x4_t y = vld1q_f32(b + i);
      vd            = vmlaq_f32(vd, x, y);
      ve            = vmlaq_f32(ve, y, y);
   }
   vst1q_f32(td, vd);
   vst1q_f32(te, ve);
   dot = (double)td[0] + td[1] + td[2] + td[3];
   be  = (double)te[0] + te[1] + te[2] + te[3];
   for (; i < n; ++i)
   {
      double x = a[i], y = b[i];
      dot     += x * y;
      be      += y * y;
   }
   return dot / sqrt(ae * be + 1.0e-30);
}
#endif

static double wsola_corr(const struct wsola *d, const float *a,
      const float *b, unsigned n, double ae)
{
#if WSOLA_HAVE_SSE2
   if (d->simd == WSOLA_SIMD_SSE2)
      return wsola_corr_sse2(a, b, n, ae);
#endif
#if WSOLA_HAVE_NEON
   if (d->simd == WSOLA_SIMD_NEON)
      return wsola_corr_neon(a, b, n, ae);
#endif
   (void)d;
   return wsola_corr_scalar(a, b, n, ae);
}

static void wsola_compact_input(struct wsola *d)
{
   uint64_t prediction, keep, discard;
   size_t remain;
   if (d->first)
      return;
   prediction = d->next_analysis > 0.0
      ? (uint64_t)floor(d->next_analysis) : 0u;
   keep       = prediction > d->search + 8u
      ? prediction - d->search - 8u : 0u;
   if (keep < d->input_base)
      keep = d->input_base;
   if (keep > d->input_write)
      keep = d->input_write;
   discard = keep - d->input_base;
   if (discard < (uint64_t)d->seq * 8u)
      return;
   remain = (size_t)(d->input_write - keep);
   if (remain)
   {
      memmove(d->input, d->input + (size_t)discard * WSOLA_CH,
            remain * WSOLA_CH * sizeof(float));
      memmove(d->mono, d->mono + (size_t)discard, remain * sizeof(float));
   }
   d->input_base = keep;
}

static int wsola_append_input(struct wsola *d, const float *src,
      unsigned frames)
{
   size_t existing, need;
   unsigned f;
   wsola_compact_input(d);
   existing = (size_t)(d->input_write - d->input_base);
   need     = existing + frames;
   if (!wsola_grow((void**)&d->input, sizeof(float),
            &d->input_cap_samples, need * WSOLA_CH))
      return 0;
   if (!wsola_grow((void**)&d->mono, sizeof(float),
            &d->mono_cap_frames, need))
      return 0;
   for (f = 0; f < frames; ++f)
   {
      float l = src[f * 2u];
      float r = src[f * 2u + 1u];
      d->input[(existing + f) * 2u]      = l;
      d->input[(existing + f) * 2u + 1u] = r;
      d->mono[existing + f]              = 0.5f * (l + r);
   }
   d->input_write += frames;
   return 1;
}

static void wsola_compact_ola(struct wsola *d)
{
   size_t remain;
   uint64_t used = d->ola_read - d->ola_base;
   if (used < (uint64_t)d->seq * 8u)
      return;
   remain = (size_t)(d->ola_end - d->ola_read);
   if (remain)
      memmove(d->ola, d->ola + (size_t)used * WSOLA_CH,
            remain * WSOLA_CH * sizeof(float));
   d->ola_base = d->ola_read;
   d->ola_end  = d->ola_base + remain;
}

static int wsola_ensure_ola(struct wsola *d, uint64_t end_abs)
{
   size_t oldf, needf;
   if (end_abs <= d->ola_end)
      return 1;
   wsola_compact_ola(d);
   oldf  = (size_t)(d->ola_end - d->ola_base);
   needf = (size_t)(end_abs - d->ola_base);
   if (!wsola_grow((void**)&d->ola, sizeof(float),
            &d->ola_cap_samples, needf * WSOLA_CH))
      return 0;
   memset(d->ola + oldf * WSOLA_CH, 0,
         (needf - oldf) * WSOLA_CH * sizeof(float));
   d->ola_end = end_abs;
   return 1;
}

static void wsola_compact_resamp(struct wsola *d)
{
   uint64_t discard;
   size_t remain;
   uint64_t center = d->resamp_pos > 0.0
      ? (uint64_t)floor(d->resamp_pos) : 0u;
   uint64_t keep   = center > WSOLA_HALF + 8u
      ? center - WSOLA_HALF - 8u : 0u;
   if (keep < d->resamp_base)
      keep = d->resamp_base;
   if (keep > d->resamp_write)
      keep = d->resamp_write;
   discard = keep - d->resamp_base;
   if (discard < 8192u)
      return;
   remain = (size_t)(d->resamp_write - keep);
   if (remain)
      memmove(d->resamp, d->resamp + (size_t)discard * WSOLA_CH,
            remain * WSOLA_CH * sizeof(float));
   d->resamp_base = keep;
}

static int wsola_append_resamp(struct wsola *d, const float *src,
      unsigned frames)
{
   size_t existing, need;
   wsola_compact_resamp(d);
   existing = (size_t)(d->resamp_write - d->resamp_base);
   need     = (existing + frames) * WSOLA_CH;
   if (!wsola_grow((void**)&d->resamp, sizeof(float),
            &d->resamp_cap_samples, need))
      return 0;
   memcpy(d->resamp + existing * WSOLA_CH, src,
         (size_t)frames * WSOLA_CH * sizeof(float));
   d->resamp_write += frames;
   return 1;
}

static int wsola_append_resamp_zero(struct wsola *d, unsigned frames)
{
   size_t existing = (size_t)(d->resamp_write - d->resamp_base);
   size_t need     = (existing + frames) * WSOLA_CH;
   if (!wsola_grow((void**)&d->resamp, sizeof(float),
            &d->resamp_cap_samples, need))
      return 0;
   memset(d->resamp + existing * WSOLA_CH, 0,
         (size_t)frames * WSOLA_CH * sizeof(float));
   d->resamp_write += frames;
   return 1;
}

static int wsola_resamp_can_output(const struct wsola *d)
{
   uint64_t c = d->resamp_pos > 0.0 ? (uint64_t)floor(d->resamp_pos) : 0u;
   return c >= WSOLA_HALF && c + WSOLA_HALF < d->resamp_write;
}

static int wsola_process_resamp(struct wsola *d)
{
   while (wsola_resamp_can_output(d))
   {
      unsigned made = 0;
      size_t old    = (size_t)d->output_frames * WSOLA_CH;
      if (!wsola_grow((void**)&d->output, sizeof(float),
               &d->output_cap_samples,
               old + (size_t)WSOLA_BLOCK * WSOLA_CH))
         return 0;
      while (made < WSOLA_BLOCK && wsola_resamp_can_output(d))
      {
         unsigned ch, t;
         const float *coef;
         size_t first;
         uint64_t center = (uint64_t)floor(d->resamp_pos);
         double frac     = d->resamp_pos - (double)center;
         unsigned ph     = (unsigned)(frac * WSOLA_PHASES);
         if (ph >= WSOLA_PHASES)
            ph = WSOLA_PHASES - 1u;
         coef  = d->table + (size_t)ph * WSOLA_TAPS;
         first = (size_t)((center - WSOLA_HALF) - d->resamp_base);
         for (ch = 0; ch < WSOLA_CH; ++ch)
         {
            double sum = 0.0;
            for (t = 0; t < WSOLA_TAPS; ++t)
               sum += (double)d->resamp[(first + t) * WSOLA_CH + ch] * coef[t];
            d->output[old + (size_t)made * WSOLA_CH + ch] = (float)sum;
         }
         ++made;
         d->resamp_pos += d->pitch_ratio;
      }
      d->output_frames += made;
      wsola_compact_resamp(d);
      if (!made)
         break;
   }
   return 1;
}

static int wsola_feed_audio(struct wsola *d, const float *src,
      unsigned frames)
{
   if (d->resamp_bypass)
      return wsola_append_output(d, src, frames);
   return wsola_append_resamp(d, src, frames) && wsola_process_resamp(d);
}

static int wsola_feed_until(struct wsola *d, uint64_t safe)
{
   uint64_t end = safe < d->ola_end ? safe : d->ola_end;
   while (d->ola_read < end)
   {
      size_t off;
      unsigned frames = (unsigned)(end - d->ola_read);
      if (frames > WSOLA_BLOCK)
         frames = WSOLA_BLOCK;
      off = (size_t)(d->ola_read - d->ola_base);
      if (!wsola_feed_audio(d, d->ola + off * WSOLA_CH, frames))
         return 0;
      d->ola_read += frames;
   }
   wsola_compact_ola(d);
   return 1;
}

static void wsola_build_reference(struct wsola *d, uint64_t output_start)
{
   unsigned f;
   size_t off = (size_t)(output_start - d->ola_base);
   for (f = 0; f < d->overlap; ++f)
   {
      size_t i        = (off + f) * WSOLA_CH;
      d->reference[f] = 0.5f * (d->ola[i] + d->ola[i + 1u]);
   }
}

static uint64_t wsola_best_candidate(struct wsola *d, uint64_t prediction,
      uint64_t maxcand)
{
   unsigned i;
   uint64_t best, c, rlo, rhi;
   double ae         = 0.0;
   double best_score = -2.0;
   uint64_t lo       = prediction > d->search ? prediction - d->search : 0u;
   uint64_t hi       = prediction + d->search;
   if (lo < d->input_base)
      lo = d->input_base;
   if (hi > maxcand)
      hi = maxcand;
   if (hi < lo)
      return lo;
   for (i = 0; i < d->overlap; ++i)
   {
      double x = d->reference[i];
      ae      += x * x;
   }
   best = prediction < lo ? lo : (prediction > hi ? hi : prediction);
   if (ae <= 1.0e-20)
      return best;

   /* Coarse pass on an 8-frame grid, then refine around the winner. */
   c = lo + ((8u - (lo & 7u)) & 7u);
   while (c <= hi)
   {
      double s = wsola_corr(d, d->reference,
            d->mono + (size_t)(c - d->input_base), d->overlap, ae);
      if (s > best_score)
      {
         best_score = s;
         best       = c;
      }
      if (hi - c < 8u)
         break;
      c += 8u;
   }

   rlo = best > 8u ? best - 8u : lo;
   rhi = best + 8u;
   if (rlo < lo)
      rlo = lo;
   if (rhi > hi)
      rhi = hi;
   for (c = rlo; c <= rhi; ++c)
   {
      double s = wsola_corr(d, d->reference,
            d->mono + (size_t)(c - d->input_base), d->overlap, ae);
      if (s > best_score)
      {
         best_score = s;
         best       = c;
      }
      if (c == rhi)
         break;
   }
   return best;
}

static int wsola_first_segment(struct wsola *d)
{
   if (!wsola_ensure_ola(d, d->seq))
      return 0;
   memcpy(d->ola, d->input, (size_t)d->seq * WSOLA_CH * sizeof(float));
   d->synth_start   = 0;
   d->next_analysis = d->ha;
   d->first         = 0;
   return wsola_feed_until(d, d->hs);
}

static int wsola_overlap_segment(struct wsola *d, uint64_t cand,
      uint64_t outstart)
{
   unsigned f;
   const float *src;
   float *dst;
   size_t outoff;
   size_t inoff = (size_t)(cand - d->input_base);
   if (!wsola_ensure_ola(d, outstart + d->seq))
      return 0;
   outoff = (size_t)(outstart - d->ola_base);
   src    = d->input + inoff * WSOLA_CH;
   dst    = d->ola + outoff * WSOLA_CH;
   for (f = 0; f < d->overlap; ++f)
   {
      float fi = (float)(f + 1u) / (float)(d->overlap + 1u);
      float fo = 1.0f - fi;
      dst[f * 2u]      = dst[f * 2u]      * fo + src[f * 2u]      * fi;
      dst[f * 2u + 1u] = dst[f * 2u + 1u] * fo + src[f * 2u + 1u] * fi;
   }
   memcpy(dst + (size_t)d->overlap * WSOLA_CH,
         src + (size_t)d->overlap * WSOLA_CH,
         (size_t)(d->seq - d->overlap) * WSOLA_CH * sizeof(float));
   return 1;
}

static int wsola_run(struct wsola *d)
{
   for (;;)
   {
      uint64_t prediction, outstart, maxcand, cand;
      if (d->first)
      {
         if (d->input_write < d->seq)
            return 1;
         if (!wsola_first_segment(d))
            return 0;
         continue;
      }
      prediction = d->next_analysis > 0.0
         ? (uint64_t)floor(d->next_analysis + 0.5) : 0u;
      if (prediction + d->search + d->seq > d->input_write)
         return 1;
      outstart = d->synth_start + d->hs;
      wsola_build_reference(d, outstart);
      maxcand  = d->input_write - d->seq;
      cand     = wsola_best_candidate(d, prediction, maxcand);
      if (!wsola_overlap_segment(d, cand, outstart))
         return 0;
      d->synth_start    = outstart;
      d->next_analysis += d->ha;
      if (!wsola_feed_until(d, outstart + d->hs))
         return 0;
      wsola_compact_input(d);
   }
}

static int wsola_prepare_resampler(struct wsola *d)
{
   unsigned ph, t;
   double cutoff;
   if (d->resamp_bypass)
      return 1;
   if (!(d->table = (float*)malloc(
               (size_t)WSOLA_PHASES * WSOLA_TAPS * sizeof(float))))
      return 0;
   cutoff = 0.95 / (d->pitch_ratio > 1.0 ? d->pitch_ratio : 1.0);
   for (ph = 0; ph < WSOLA_PHASES; ++ph)
   {
      double frac = (double)ph / WSOLA_PHASES;
      double sum  = 0.0;
      for (t = 0; t < WSOLA_TAPS; ++t)
      {
         double dist = (double)((int)t - (int)WSOLA_HALF) - frac;
         double norm = dist / WSOLA_HALF;
         double win  = fabs(norm) <= 1.0
            ? 0.42 + 0.50 * cos(WSOLA_PI * norm)
                   + 0.08 * cos(2.0 * WSOLA_PI * norm)
            : 0.0;
         double x    = cutoff * dist;
         double sinc = fabs(x) < 1.0e-12
            ? 1.0 : sin(WSOLA_PI * x) / (WSOLA_PI * x);
         double c    = cutoff * sinc * win;
         d->table[(size_t)ph * WSOLA_TAPS + t] = (float)c;
         sum        += c;
      }
      if (fabs(sum) > 1.0e-15)
      {
         float inv = (float)(1.0 / sum);
         for (t = 0; t < WSOLA_TAPS; ++t)
            d->table[(size_t)ph * WSOLA_TAPS + t] *= inv;
      }
   }
   if (!wsola_append_resamp_zero(d, WSOLA_HALF))
      return 0;
   d->resamp_pos = WSOLA_HALF;
   return 1;
}

static void wsola_free(void *opaque)
{
   struct wsola *d = (struct wsola*)opaque;
   if (!d)
      return;
   free(d->input);
   free(d->mono);
   free(d->reference);
   free(d->ola);
   free(d->table);
   free(d->resamp);
   free(d->output);
   free(d->in_f);
   free(d->out_i16);
   free(d);
}

static void *wsola_init_common(const struct dspfilter_info *info,
      const struct dspfilter_config *config, void *userdata,
      enum wsola_simd simd)
{
   unsigned rate;
   float pitch, tempo;
   struct wsola *d;

   if (!info || info->input_rate <= 0.0f)
      return NULL;
   if (!(d = (struct wsola*)calloc(1, sizeof(*d))))
      return NULL;

   config->get_float(userdata, "pitch", &pitch, 0.0f);
   config->get_float(userdata, "tempo", &tempo, 0.0f);

   d->rate           = info->input_rate;
   d->pitch_st       = wsola_clampd(pitch, -12.0, 12.0);
   d->tempo_pct      = wsola_clampd(tempo, -50.0, 100.0);
   d->pitch_ratio    = pow(2.0, d->pitch_st / 12.0);
   d->tempo_ratio    = 1.0 + d->tempo_pct * 0.01;
   d->duration_ratio = d->pitch_ratio / d->tempo_ratio;
   d->wsola_bypass   = fabs(d->duration_ratio - 1.0) < 1.0e-9;
   d->resamp_bypass  = fabs(d->pitch_ratio - 1.0) < 1.0e-9;
   d->simd           = simd;

   /* 40 ms segments, 8 ms cross-fade, +/-12 ms search, 8-frame aligned. */
   rate       = (unsigned)(d->rate + 0.5f);
   d->seq     = wsola_align_up((unsigned)(rate * 0.040 + 0.5), 8u);
   if (d->seq < 64u)
      d->seq = 64u;
   d->overlap = wsola_align_up((unsigned)(rate * 0.008 + 0.5), 8u);
   if (d->overlap < 32u)
      d->overlap = 32u;
   if (d->overlap >= d->seq)
      d->overlap = d->seq / 4u;
   d->search  = wsola_align_up((unsigned)(rate * 0.012 + 0.5), 8u);
   if (d->search < 16u)
      d->search = 16u;
   d->hs      = d->seq - d->overlap;
   d->ha      = (double)d->hs / d->duration_ratio;
   d->first   = 1;

   if (!d->wsola_bypass)
   {
      if (!(d->reference = (float*)malloc((size_t)d->overlap * sizeof(float))))
      {
         wsola_free(d);
         return NULL;
      }
   }
   if (!wsola_prepare_resampler(d))
   {
      wsola_free(d);
      return NULL;
   }
   return d;
}

static void *wsola_init_scalar(const struct dspfilter_info *info,
      const struct dspfilter_config *config, void *userdata)
{
   return wsola_init_common(info, config, userdata, WSOLA_SIMD_SCALAR);
}

#if WSOLA_HAVE_SSE2
static void *wsola_init_sse2(const struct dspfilter_info *info,
      const struct dspfilter_config *config, void *userdata)
{
   return wsola_init_common(info, config, userdata, WSOLA_SIMD_SSE2);
}
#endif

#if WSOLA_HAVE_NEON
static void *wsola_init_neon(const struct dspfilter_info *info,
      const struct dspfilter_config *config, void *userdata)
{
   return wsola_init_common(info, config, userdata, WSOLA_SIMD_NEON);
}
#endif

static void wsola_process(void *opaque, struct dspfilter_output *out,
      const struct dspfilter_input *in)
{
   struct wsola *d = (struct wsola*)opaque;
   if (!out)
      return;
   out->samples = NULL;
   out->frames  = 0;
   if (!d || !in || !in->samples || !in->frames || d->failed)
      return;
   if (d->pitch_st == 0.0 && d->tempo_pct == 0.0)
   {
      out->samples = in->samples;
      out->frames  = in->frames;
      return;
   }
   d->output_frames = 0;
   if (d->wsola_bypass)
   {
      if (     !wsola_append_resamp(d, in->samples, in->frames)
            || !wsola_process_resamp(d))
         d->failed = 1;
   }
   else
   {
      if (     !wsola_append_input(d, in->samples, in->frames)
            || !wsola_run(d))
         d->failed = 1;
   }
   if (!d->failed)
   {
      out->samples = d->output;
      out->frames  = d->output_frames;
   }
}

static int16_t wsola_float_to_s16(float v)
{
   if (v >= 1.0f)
      return 32767;
   if (v > -1.0f)
   {
      int32_t s = (v >= 0.0f) ? (int32_t)(v * 32768.0f + 0.5f)
                              : (int32_t)(v * 32768.0f - 0.5f);
      return (int16_t)((s > 32767) ? 32767 : s);
   }
   if (v <= -1.0f)
      return -32768;
   return 0;
}

/* WSOLA and the resampler are floating-point; the int16 entry point runs
 * them on a converted copy so the chain can stay on the int16 path. */
static void wsola_process_i16(void *opaque,
      struct dspfilter_output_i16 *out,
      const struct dspfilter_input_i16 *in)
{
   unsigned i;
   struct dspfilter_input  fin;
   struct dspfilter_output fout;
   struct wsola *d = (struct wsola*)opaque;

   out->samples    = in->samples;
   out->frames     = in->frames;

   if (     !in->frames
         || d->failed
         || (d->pitch_st == 0.0 && d->tempo_pct == 0.0))
      return;

   if (!wsola_grow((void**)&d->in_f, sizeof(float),
            &d->in_f_cap_samples, (size_t)in->frames * WSOLA_CH))
      return;
   for (i = 0; i < in->frames * WSOLA_CH; i++)
      d->in_f[i] = (float)in->samples[i] * (1.0f / 32768.0f);

   fin.samples = d->in_f;
   fin.frames  = in->frames;
   wsola_process(d, &fout, &fin);

   if (d->failed)
      return;
   out->frames = 0;
   if (!fout.samples || !fout.frames)
      return;
   if (!wsola_grow((void**)&d->out_i16, sizeof(int16_t),
            &d->out_i16_cap_samples, (size_t)fout.frames * WSOLA_CH))
   {
      d->failed = 1;
      out->frames = in->frames;
      return;
   }
   for (i = 0; i < fout.frames * WSOLA_CH; i++)
      d->out_i16[i] = wsola_float_to_s16(fout.samples[i]);
   out->samples = d->out_i16;
   out->frames  = fout.frames;
}

static const struct dspfilter_implementation wsola_plug_scalar = {
   wsola_init_scalar,
   wsola_process,
   wsola_free,

   DSPFILTER_API_VERSION,
   "WSOLA Pitch / Tempo",
   "wsolapitchtempo",

   wsola_process_i16,
};

#if WSOLA_HAVE_SSE2
static const struct dspfilter_implementation wsola_plug_sse2 = {
   wsola_init_sse2,
   wsola_process,
   wsola_free,

   DSPFILTER_API_VERSION,
   "WSOLA Pitch / Tempo (SSE2)",
   "wsolapitchtempo",

   wsola_process_i16,
};
#endif

#if WSOLA_HAVE_NEON
static const struct dspfilter_implementation wsola_plug_neon = {
   wsola_init_neon,
   wsola_process,
   wsola_free,

   DSPFILTER_API_VERSION,
   "WSOLA Pitch / Tempo (NEON)",
   "wsolapitchtempo",

   wsola_process_i16,
};
#endif

#ifdef HAVE_FILTERS_BUILTIN
#define dspfilter_get_implementation wsolapitchtempo_dspfilter_get_implementation
#endif

const struct dspfilter_implementation *dspfilter_get_implementation(dspfilter_simd_mask_t mask)
{
#if WSOLA_HAVE_NEON
   if (mask & DSPFILTER_SIMD_NEON)
      return &wsola_plug_neon;
#endif
#if WSOLA_HAVE_SSE2
   if (mask & DSPFILTER_SIMD_SSE2)
      return &wsola_plug_sse2;
#endif
   (void)mask;
   return &wsola_plug_scalar;
}

#undef dspfilter_get_implementation
