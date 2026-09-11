/* Copyright  (C) 2010-2026 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (overdrive.c).
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

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include <retro_inline.h>
#include <libretro_dspfilter.h>

/* Frames converted per step by the int16 entry point. */
#define OVERDRIVE_I16_CHUNK 256

/* Filter state below this magnitude is flushed to zero, well under
 * audibility and well above the denormal range. */
#define OVERDRIVE_FLUSH 1.0e-15f

struct overdrive_data
{
   float scratch[OVERDRIVE_I16_CHUNK * 2];
   /* Asymmetry is a bias into the tanh, with the bias's own output
    * subtracted and the small-signal gain restored, so the curve stays
    * monotonic at any drive. Kept in double so silence maps to exactly
    * zero. */
   double bias;
   double bias_out;
   double bias_norm;
   float drive;
   float tone_alpha;
   float mix;
   float level;
   float dc_r;
   float prev_in[2];
   float prev_hp[2];
   float tone_lp[2];
};

static INLINE float overdrive_flush(float v)
{
   return (fabs(v) < OVERDRIVE_FLUSH) ? 0.0f : v;
}

static void overdrive_run(struct overdrive_data *od,
      float *samples, unsigned frames)
{
   unsigned i, ch;

   for (i = 0; i < frames; i++, samples += 2)
   {
      for (ch = 0; ch < 2; ch++)
      {
         float hp, wet;
         float dry    = samples[ch];
         float driven = dry * od->drive;
         float shaped;

         if (driven < -12.0f)
            driven = -12.0f;
         else if (driven > 12.0f)
            driven = 12.0f;
         shaped = (float)((tanh((double)driven + od->bias) - od->bias_out)
               * od->bias_norm);

         /* DC blocker for the offset the asymmetric curve introduces. */
         hp               = shaped - od->prev_in[ch]
                          + od->dc_r * od->prev_hp[ch];
         hp               = overdrive_flush(hp);
         od->prev_in[ch]  = shaped;
         od->prev_hp[ch]  = hp;

         /* One-pole low-pass tone stage with a small direct component. */
         od->tone_lp[ch] += od->tone_alpha * (hp - od->tone_lp[ch]);
         od->tone_lp[ch]  = overdrive_flush(od->tone_lp[ch]);
         wet              = 0.30f * hp + 0.70f * od->tone_lp[ch];
         wet             *= od->level;

         samples[ch]      = dry + (wet - dry) * od->mix;
      }
   }
}

static void overdrive_process(void *data, struct dspfilter_output *output,
      const struct dspfilter_input *input)
{
   output->samples = input->samples;
   output->frames  = input->frames;
   overdrive_run((struct overdrive_data*)data, input->samples, input->frames);
}

static int16_t overdrive_float_to_s16(float v)
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

/* The waveshaper is inherently floating-point; the int16 entry point
 * runs it on a converted copy so the chain can stay on the int16 path. */
static void overdrive_process_i16(void *data,
      struct dspfilter_output_i16 *output,
      const struct dspfilter_input_i16 *input)
{
   struct overdrive_data *od = (struct overdrive_data*)data;
   int16_t *samples          = input->samples;
   unsigned remaining        = input->frames;

   output->samples           = input->samples;
   output->frames            = input->frames;

   while (remaining)
   {
      unsigned i;
      unsigned chunk = remaining;
      if (chunk > OVERDRIVE_I16_CHUNK)
         chunk = OVERDRIVE_I16_CHUNK;

      for (i = 0; i < chunk * 2; i++)
         od->scratch[i] = (float)samples[i] * (1.0f / 32768.0f);
      overdrive_run(od, od->scratch, chunk);
      for (i = 0; i < chunk * 2; i++)
         samples[i] = overdrive_float_to_s16(od->scratch[i]);

      samples   += chunk * 2;
      remaining -= chunk;
   }
}

static void overdrive_free(void *data)
{
   free(data);
}

static float overdrive_clampf(float x, float lo, float hi)
{
   return x < lo ? lo : (x > hi ? hi : x);
}

static float overdrive_db_to_gain(float db)
{
   return (float)pow(10.0, db * 0.05f);
}

static void *overdrive_init(const struct dspfilter_info *info,
      const struct dspfilter_config *config, void *userdata)
{
   float drive_db, tone_hz, mix, level_db, asymmetry, sr, omega;
   double t;
   struct overdrive_data *od;

   if (!info || info->input_rate <= 1.0f)
      return NULL;

   if (!(od = (struct overdrive_data*)calloc(1, sizeof(*od))))
      return NULL;

   config->get_float(userdata, "drive_db", &drive_db, 18.0f);
   config->get_float(userdata, "tone_hz", &tone_hz, 6500.0f);
   config->get_float(userdata, "drywet", &mix, 1.0f);
   config->get_float(userdata, "level_db", &level_db, -4.0f);
   config->get_float(userdata, "asymmetry", &asymmetry, 0.12f);

   sr             = info->input_rate;
   tone_hz        = overdrive_clampf(tone_hz, 200.0f, sr * 0.45f);
   omega          = 2.0f * 3.14159265358979323846f * tone_hz / sr;

   od->drive      = overdrive_db_to_gain(overdrive_clampf(drive_db, 0.0f, 42.0f));
   od->tone_alpha = 1.0f - (float)exp(-omega);
   od->mix        = overdrive_clampf(mix, 0.0f, 1.0f);
   od->level      = overdrive_db_to_gain(overdrive_clampf(level_db, -30.0f, 12.0f));
   od->dc_r       = 0.995f;

   /* tanh(x + b) - tanh(b), scaled by 1 / (1 - tanh(b)^2), expands to
    * x - tanh(b) x^2 + ..., so tanh(b) = -0.15 * asymmetry keeps the
    * even-harmonic balance of the asymmetry control. */
   t              = -0.15 * overdrive_clampf(asymmetry, 0.0f, 1.0f);
   od->bias       = 0.5 * log((1.0 + t) / (1.0 - t));
   od->bias_out   = tanh(od->bias);
   od->bias_norm  = 1.0 / (1.0 - od->bias_out * od->bias_out);

   return od;
}

static const struct dspfilter_implementation overdrive_plug = {
   overdrive_init,
   overdrive_process,
   overdrive_free,

   DSPFILTER_API_VERSION,
   "Distortion / Overdrive",
   "overdrive",

   overdrive_process_i16,
};

#ifdef HAVE_FILTERS_BUILTIN
#define dspfilter_get_implementation overdrive_dspfilter_get_implementation
#endif

const struct dspfilter_implementation *dspfilter_get_implementation(dspfilter_simd_mask_t mask)
{
   (void)mask;
   return &overdrive_plug;
}

#undef dspfilter_get_implementation
