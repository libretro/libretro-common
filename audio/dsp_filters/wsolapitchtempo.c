/*
 * wsolapitchtempo.c
 * Self-contained C libretro DSP filter: independent pitch + tempo.
 * WSOLA duration change followed by polyphase sinc resampling.
 */
#include <libretro_dspfilter.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#define NPT_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#define NPT_EXPORT __attribute__((visibility("default")))
#else
#define NPT_EXPORT
#endif

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define NPT_SSE2 1
#else
#define NPT_SSE2 0
#endif

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define NPT_NEON 1
#else
#define NPT_NEON 0
#endif

#define CH 2u
#define PI 3.141592653589793238462643383279502884
#define TAPS 16u
#define HALF (TAPS / 2u)
#define PHASES 1024u
#define BLOCK 8192u

enum simd_kind { SIMD_SCALAR = 0, SIMD_SSE2 = 1, SIMD_NEON = 2 };

struct wsola {
   float rate;
   double pitch_st, tempo_pct, pitch_ratio, tempo_ratio, duration_ratio;
   unsigned seq, overlap, search, hs;
   double ha, next_analysis;
   uint64_t synth_start;
   int first, wsola_bypass, resamp_bypass, failed;
   enum simd_kind simd;

   float *input, *mono, *reference;
   size_t input_cap_samples, mono_cap_frames;
   uint64_t input_base, input_write;

   float *ola;
   size_t ola_cap_samples;
   uint64_t ola_base, ola_read, ola_end;

   float *table, *resamp;
   size_t resamp_cap_samples;
   uint64_t resamp_base, resamp_write;
   double resamp_pos;

   float *output;
   size_t output_cap_samples;
   unsigned output_frames;
};

static double clampd(double x, double lo, double hi)
{
   return x < lo ? lo : (x > hi ? hi : x);
}

static size_t next_pow2(size_t n)
{
   size_t p = 1;
   while (p < n && p <= (SIZE_MAX >> 1)) p <<= 1;
   return p < n ? n : p;
}

static int grow(void **ptr, size_t elem, size_t *cap, size_t need)
{
   void *p;
   size_t nc;
   if (need <= *cap) return 1;
   nc = next_pow2(need < 256 ? 256 : need);
   if (nc > SIZE_MAX / elem) return 0;
   p = realloc(*ptr, nc * elem);
   if (!p) return 0;
   *ptr = p;
   *cap = nc;
   return 1;
}

static unsigned align_up(unsigned v, unsigned a)
{
   return ((v + a - 1u) / a) * a;
}

static int append_output(struct wsola *d, const float *src, unsigned frames)
{
   size_t old = (size_t)d->output_frames * CH;
   size_t add = (size_t)frames * CH;
   if (!frames) return 1;
   if (!grow((void**)&d->output, sizeof(float), &d->output_cap_samples, old + add)) return 0;
   memcpy(d->output + old, src, add * sizeof(float));
   d->output_frames += frames;
   return 1;
}

static double corr_scalar(const float *a, const float *b, unsigned n, double ae)
{
   double dot = 0.0, be = 0.0;
   unsigned i;
   for (i = 0; i < n; ++i) { double x = a[i], y = b[i]; dot += x * y; be += y * y; }
   return dot / sqrt(ae * be + 1.0e-30);
}

#if NPT_SSE2
static double corr_sse2(const float *a, const float *b, unsigned n, double ae)
{
   __m128 vd = _mm_setzero_ps(), ve = _mm_setzero_ps();
   float td[4], te[4];
   double dot, be;
   unsigned i = 0;
   for (; i + 4u <= n; i += 4u) {
      __m128 x = _mm_loadu_ps(a + i), y = _mm_loadu_ps(b + i);
      vd = _mm_add_ps(vd, _mm_mul_ps(x, y));
      ve = _mm_add_ps(ve, _mm_mul_ps(y, y));
   }
   _mm_storeu_ps(td, vd); _mm_storeu_ps(te, ve);
   dot = (double)td[0] + td[1] + td[2] + td[3];
   be = (double)te[0] + te[1] + te[2] + te[3];
   for (; i < n; ++i) { double x = a[i], y = b[i]; dot += x * y; be += y * y; }
   return dot / sqrt(ae * be + 1.0e-30);
}
#endif

#if NPT_NEON
static double corr_neon(const float *a, const float *b, unsigned n, double ae)
{
   float32x4_t vd = vdupq_n_f32(0.0f), ve = vdupq_n_f32(0.0f);
   float td[4], te[4];
   double dot, be;
   unsigned i = 0;
   for (; i + 4u <= n; i += 4u) {
      float32x4_t x = vld1q_f32(a + i), y = vld1q_f32(b + i);
      vd = vmlaq_f32(vd, x, y); ve = vmlaq_f32(ve, y, y);
   }
   vst1q_f32(td, vd); vst1q_f32(te, ve);
   dot = (double)td[0] + td[1] + td[2] + td[3];
   be = (double)te[0] + te[1] + te[2] + te[3];
   for (; i < n; ++i) { double x = a[i], y = b[i]; dot += x * y; be += y * y; }
   return dot / sqrt(ae * be + 1.0e-30);
}
#endif

static double corr(struct wsola *d, const float *a, const float *b, unsigned n, double ae)
{
#if NPT_SSE2
   if (d->simd == SIMD_SSE2) return corr_sse2(a, b, n, ae);
#endif
#if NPT_NEON
   if (d->simd == SIMD_NEON) return corr_neon(a, b, n, ae);
#endif
   return corr_scalar(a, b, n, ae);
}

static void compact_input(struct wsola *d)
{
   uint64_t prediction, keep, discard;
   size_t remain;
   if (d->first) return;
   prediction = d->next_analysis > 0.0 ? (uint64_t)floor(d->next_analysis) : 0u;
   keep = prediction > d->search + 8u ? prediction - d->search - 8u : 0u;
   if (keep < d->input_base) keep = d->input_base;
   if (keep > d->input_write) keep = d->input_write;
   discard = keep - d->input_base;
   if (discard < (uint64_t)d->seq * 8u) return;
   remain = (size_t)(d->input_write - keep);
   if (remain) {
      memmove(d->input, d->input + (size_t)discard * CH, remain * CH * sizeof(float));
      memmove(d->mono, d->mono + (size_t)discard, remain * sizeof(float));
   }
   d->input_base = keep;
}

static int append_input(struct wsola *d, const float *src, unsigned frames)
{
   size_t existing, need;
   unsigned f;
   compact_input(d);
   existing = (size_t)(d->input_write - d->input_base);
   need = existing + frames;
   if (!grow((void**)&d->input, sizeof(float), &d->input_cap_samples, need * CH)) return 0;
   if (!grow((void**)&d->mono, sizeof(float), &d->mono_cap_frames, need)) return 0;
   for (f = 0; f < frames; ++f) {
      float l = src[f * 2u], r = src[f * 2u + 1u];
      d->input[(existing + f) * 2u] = l;
      d->input[(existing + f) * 2u + 1u] = r;
      d->mono[existing + f] = 0.5f * (l + r);
   }
   d->input_write += frames;
   return 1;
}

static void compact_ola(struct wsola *d)
{
   uint64_t used = d->ola_read - d->ola_base;
   size_t remain;
   if (used < (uint64_t)d->seq * 8u) return;
   remain = (size_t)(d->ola_end - d->ola_read);
   if (remain) memmove(d->ola, d->ola + (size_t)used * CH, remain * CH * sizeof(float));
   d->ola_base = d->ola_read;
   d->ola_end = d->ola_base + remain;
}

static int ensure_ola(struct wsola *d, uint64_t end_abs)
{
   size_t oldf, needf;
   if (end_abs <= d->ola_end) return 1;
   compact_ola(d);
   oldf = (size_t)(d->ola_end - d->ola_base);
   needf = (size_t)(end_abs - d->ola_base);
   if (!grow((void**)&d->ola, sizeof(float), &d->ola_cap_samples, needf * CH)) return 0;
   memset(d->ola + oldf * CH, 0, (needf - oldf) * CH * sizeof(float));
   d->ola_end = end_abs;
   return 1;
}

static void compact_resamp(struct wsola *d)
{
   uint64_t center = d->resamp_pos > 0.0 ? (uint64_t)floor(d->resamp_pos) : 0u;
   uint64_t keep = center > HALF + 8u ? center - HALF - 8u : 0u;
   uint64_t discard;
   size_t remain;
   if (keep < d->resamp_base) keep = d->resamp_base;
   if (keep > d->resamp_write) keep = d->resamp_write;
   discard = keep - d->resamp_base;
   if (discard < 8192u) return;
   remain = (size_t)(d->resamp_write - keep);
   if (remain) memmove(d->resamp, d->resamp + (size_t)discard * CH, remain * CH * sizeof(float));
   d->resamp_base = keep;
}

static int append_resamp(struct wsola *d, const float *src, unsigned frames)
{
   size_t existing, need;
   compact_resamp(d);
   existing = (size_t)(d->resamp_write - d->resamp_base);
   need = (existing + frames) * CH;
   if (!grow((void**)&d->resamp, sizeof(float), &d->resamp_cap_samples, need)) return 0;
   memcpy(d->resamp + existing * CH, src, (size_t)frames * CH * sizeof(float));
   d->resamp_write += frames;
   return 1;
}

static int append_resamp_zero(struct wsola *d, unsigned frames)
{
   size_t existing = (size_t)(d->resamp_write - d->resamp_base);
   size_t need = (existing + frames) * CH;
   if (!grow((void**)&d->resamp, sizeof(float), &d->resamp_cap_samples, need)) return 0;
   memset(d->resamp + existing * CH, 0, (size_t)frames * CH * sizeof(float));
   d->resamp_write += frames;
   return 1;
}

static int resamp_can_output(const struct wsola *d)
{
   uint64_t c = d->resamp_pos > 0.0 ? (uint64_t)floor(d->resamp_pos) : 0u;
   return c >= HALF && c + HALF < d->resamp_write;
}

static int process_resamp(struct wsola *d)
{
   while (resamp_can_output(d)) {
      unsigned made = 0;
      size_t old = (size_t)d->output_frames * CH;
      if (!grow((void**)&d->output, sizeof(float), &d->output_cap_samples,
               old + (size_t)BLOCK * CH)) return 0;
      while (made < BLOCK && resamp_can_output(d)) {
         uint64_t center = (uint64_t)floor(d->resamp_pos);
         double frac = d->resamp_pos - (double)center;
         unsigned ph = (unsigned)(frac * PHASES);
         const float *coef;
         size_t first;
         unsigned ch, t;
         if (ph >= PHASES) ph = PHASES - 1u;
         coef = d->table + (size_t)ph * TAPS;
         first = (size_t)((center - HALF) - d->resamp_base);
         for (ch = 0; ch < CH; ++ch) {
            double sum = 0.0;
            for (t = 0; t < TAPS; ++t)
               sum += (double)d->resamp[(first + t) * CH + ch] * coef[t];
            d->output[old + (size_t)made * CH + ch] = (float)sum;
         }
         ++made;
         d->resamp_pos += d->pitch_ratio;
      }
      d->output_frames += made;
      compact_resamp(d);
      if (!made) break;
   }
   return 1;
}

static int feed_audio(struct wsola *d, const float *src, unsigned frames)
{
   if (d->resamp_bypass) return append_output(d, src, frames);
   return append_resamp(d, src, frames) && process_resamp(d);
}

static int feed_until(struct wsola *d, uint64_t safe)
{
   uint64_t end = safe < d->ola_end ? safe : d->ola_end;
   while (d->ola_read < end) {
      unsigned frames = (unsigned)(end - d->ola_read);
      size_t off;
      if (frames > BLOCK) frames = BLOCK;
      off = (size_t)(d->ola_read - d->ola_base);
      if (!feed_audio(d, d->ola + off * CH, frames)) return 0;
      d->ola_read += frames;
   }
   compact_ola(d);
   return 1;
}

static void build_reference(struct wsola *d, uint64_t output_start)
{
   size_t off = (size_t)(output_start - d->ola_base);
   unsigned f;
   for (f = 0; f < d->overlap; ++f) {
      size_t i = (off + f) * CH;
      d->reference[f] = 0.5f * (d->ola[i] + d->ola[i + 1u]);
   }
}

static uint64_t best_candidate(struct wsola *d, uint64_t prediction, uint64_t maxcand)
{
   uint64_t lo = prediction > d->search ? prediction - d->search : 0u;
   uint64_t hi = prediction + d->search;
   uint64_t best, c;
   double ae = 0.0, best_score = -2.0;
   unsigned i;
   if (lo < d->input_base) lo = d->input_base;
   if (hi > maxcand) hi = maxcand;
   if (hi < lo) return lo;
   for (i = 0; i < d->overlap; ++i) { double x = d->reference[i]; ae += x * x; }
   best = prediction < lo ? lo : (prediction > hi ? hi : prediction);
   if (ae <= 1.0e-20) return best;

   c = lo + ((8u - (lo & 7u)) & 7u);
   while (c <= hi) {
      double s = corr(d, d->reference, d->mono + (size_t)(c - d->input_base), d->overlap, ae);
      if (s > best_score) { best_score = s; best = c; }
      if (hi - c < 8u) break;
      c += 8u;
   }
   {
      uint64_t rlo = best > 8u ? best - 8u : lo;
      uint64_t rhi = best + 8u;
      if (rlo < lo) rlo = lo;
      if (rhi > hi) rhi = hi;
      for (c = rlo; c <= rhi; ++c) {
         double s = corr(d, d->reference, d->mono + (size_t)(c - d->input_base), d->overlap, ae);
         if (s > best_score) { best_score = s; best = c; }
         if (c == rhi) break;
      }
   }
   return best;
}

static int first_segment(struct wsola *d)
{
   if (!ensure_ola(d, d->seq)) return 0;
   memcpy(d->ola, d->input, (size_t)d->seq * CH * sizeof(float));
   d->synth_start = 0;
   d->next_analysis = d->ha;
   d->first = 0;
   return feed_until(d, d->hs);
}

static int overlap_segment(struct wsola *d, uint64_t cand, uint64_t outstart)
{
   size_t inoff = (size_t)(cand - d->input_base), outoff;
   const float *src;
   float *dst;
   unsigned f;
   if (!ensure_ola(d, outstart + d->seq)) return 0;
   outoff = (size_t)(outstart - d->ola_base);
   src = d->input + inoff * CH;
   dst = d->ola + outoff * CH;
   for (f = 0; f < d->overlap; ++f) {
      float fi = (float)(f + 1u) / (float)(d->overlap + 1u);
      float fo = 1.0f - fi;
      dst[f * 2u] = dst[f * 2u] * fo + src[f * 2u] * fi;
      dst[f * 2u + 1u] = dst[f * 2u + 1u] * fo + src[f * 2u + 1u] * fi;
   }
   memcpy(dst + (size_t)d->overlap * CH, src + (size_t)d->overlap * CH,
         (size_t)(d->seq - d->overlap) * CH * sizeof(float));
   return 1;
}

static int process_wsola(struct wsola *d)
{
   for (;;) {
      uint64_t prediction, outstart, maxcand, cand;
      if (d->first) {
         if (d->input_write < d->seq) return 1;
         if (!first_segment(d)) return 0;
         continue;
      }
      prediction = d->next_analysis > 0.0 ? (uint64_t)llround(d->next_analysis) : 0u;
      if (prediction + d->search + d->seq > d->input_write) return 1;
      outstart = d->synth_start + d->hs;
      build_reference(d, outstart);
      maxcand = d->input_write - d->seq;
      cand = best_candidate(d, prediction, maxcand);
      if (!overlap_segment(d, cand, outstart)) return 0;
      d->synth_start = outstart;
      d->next_analysis += d->ha;
      if (!feed_until(d, outstart + d->hs)) return 0;
      compact_input(d);
   }
}

static int prepare_resampler(struct wsola *d)
{
   unsigned ph, t;
   double cutoff;
   if (d->resamp_bypass) return 1;
   d->table = (float*)malloc((size_t)PHASES * TAPS * sizeof(float));
   if (!d->table) return 0;
   cutoff = 0.95 / (d->pitch_ratio > 1.0 ? d->pitch_ratio : 1.0);
   for (ph = 0; ph < PHASES; ++ph) {
      double frac = (double)ph / PHASES, sum = 0.0;
      for (t = 0; t < TAPS; ++t) {
         double dist = (double)((int)t - (int)HALF) - frac;
         double norm = dist / HALF;
         double win = fabs(norm) <= 1.0 ?
            0.42 + 0.50 * cos(PI * norm) + 0.08 * cos(2.0 * PI * norm) : 0.0;
         double x = cutoff * dist;
         double sinc = fabs(x) < 1.0e-12 ? 1.0 : sin(PI * x) / (PI * x);
         double c = cutoff * sinc * win;
         d->table[(size_t)ph * TAPS + t] = (float)c;
         sum += c;
      }
      if (fabs(sum) > 1.0e-15) {
         float inv = (float)(1.0 / sum);
         for (t = 0; t < TAPS; ++t) d->table[(size_t)ph * TAPS + t] *= inv;
      }
   }
   if (!append_resamp_zero(d, HALF)) return 0;
   d->resamp_pos = HALF;
   return 1;
}

static void wsola_free(void *opaque)
{
   struct wsola *d = (struct wsola*)opaque;
   if (!d) return;
   free(d->input); free(d->mono); free(d->reference); free(d->ola);
   free(d->table); free(d->resamp); free(d->output); free(d);
}

static void *init_common(const struct dspfilter_info *info,
      const struct dspfilter_config *config, void *userdata, enum simd_kind simd)
{
   struct wsola *d;
   float pitch = 0.0f, tempo = 0.0f;
   unsigned rate;
   if (!info || info->input_rate <= 0.0f) return NULL;
   d = (struct wsola*)calloc(1, sizeof(*d));
   if (!d) return NULL;
   if (config) {
      config->get_float(userdata, "pitch", &pitch, 0.0f);
      config->get_float(userdata, "tempo", &tempo, 0.0f);
   }
   d->rate = info->input_rate;
   d->pitch_st = clampd(pitch, -12.0, 12.0);
   d->tempo_pct = clampd(tempo, -50.0, 100.0);
   d->pitch_ratio = pow(2.0, d->pitch_st / 12.0);
   d->tempo_ratio = 1.0 + d->tempo_pct * 0.01;
   d->duration_ratio = d->pitch_ratio / d->tempo_ratio;
   d->wsola_bypass = fabs(d->duration_ratio - 1.0) < 1.0e-9;
   d->resamp_bypass = fabs(d->pitch_ratio - 1.0) < 1.0e-9;
   d->simd = simd;
   rate = (unsigned)(d->rate + 0.5f);
   d->seq = align_up((unsigned)(rate * 0.040 + 0.5), 8u); if (d->seq < 64u) d->seq = 64u;
   d->overlap = align_up((unsigned)(rate * 0.008 + 0.5), 8u); if (d->overlap < 32u) d->overlap = 32u;
   if (d->overlap >= d->seq) d->overlap = d->seq / 4u;
   d->search = align_up((unsigned)(rate * 0.012 + 0.5), 8u); if (d->search < 16u) d->search = 16u;
   d->hs = d->seq - d->overlap;
   d->ha = (double)d->hs / d->duration_ratio;
   d->first = 1;
   if (!d->wsola_bypass) {
      d->reference = (float*)malloc((size_t)d->overlap * sizeof(float));
      if (!d->reference) { wsola_free(d); return NULL; }
   }
   if (!prepare_resampler(d)) { wsola_free(d); return NULL; }
   return d;
}

static void *init_scalar(const struct dspfilter_info *i, const struct dspfilter_config *c, void *u)
{ return init_common(i, c, u, SIMD_SCALAR); }
#if NPT_SSE2
static void *init_sse2(const struct dspfilter_info *i, const struct dspfilter_config *c, void *u)
{ return init_common(i, c, u, SIMD_SSE2); }
#endif
#if NPT_NEON
static void *init_neon(const struct dspfilter_info *i, const struct dspfilter_config *c, void *u)
{ return init_common(i, c, u, SIMD_NEON); }
#endif

static void wsola_process(void *opaque, struct dspfilter_output *out,
      const struct dspfilter_input *in)
{
   struct wsola *d = (struct wsola*)opaque;
   if (!out) return;
   out->samples = NULL; out->frames = 0;
   if (!d || !in || !in->samples || !in->frames || d->failed) return;
   if (d->pitch_st == 0.0 && d->tempo_pct == 0.0) {
      out->samples = in->samples; out->frames = in->frames; return;
   }
   d->output_frames = 0;
   if (d->wsola_bypass) {
      if (!append_resamp(d, in->samples, in->frames) || !process_resamp(d)) d->failed = 1;
   } else {
      if (!append_input(d, in->samples, in->frames) || !process_wsola(d)) d->failed = 1;
   }
   if (!d->failed) { out->samples = d->output; out->frames = d->output_frames; }
}

static const struct dspfilter_implementation plug_scalar = {
   init_scalar, wsola_process, wsola_free, 1u,
   "WSOLA Pitch / Tempo", "wsolapitchtempo"
#if DSPFILTER_API_VERSION >= 2
   , NULL
#endif
};
#if NPT_SSE2
static const struct dspfilter_implementation plug_sse2 = {
   init_sse2, wsola_process, wsola_free, 1u,
   "WSOLA Pitch / Tempo (SSE2)", "wsolapitchtempo"
#if DSPFILTER_API_VERSION >= 2
   , NULL
#endif
};
#endif
#if NPT_NEON
static const struct dspfilter_implementation plug_neon = {
   init_neon, wsola_process, wsola_free, 1u,
   "WSOLA Pitch / Tempo (NEON)", "wsolapitchtempo"
#if DSPFILTER_API_VERSION >= 2
   , NULL
#endif
};
#endif

NPT_EXPORT const struct dspfilter_implementation *
#ifdef HAVE_FILTERS_BUILTIN
wsolapitchtempo_dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#else
dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#endif
{
#if NPT_NEON
   if (mask & DSPFILTER_SIMD_NEON) return &plug_neon;
#endif
#if NPT_SSE2
   if (mask & DSPFILTER_SIMD_SSE2) return &plug_sse2;
#endif
   return &plug_scalar;
}
