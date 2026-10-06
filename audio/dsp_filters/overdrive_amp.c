#include "libretro_dspfilter.h"
#include "dsp_common.h"

#include <stdint.h>

#define OD_Q16_ONE 65536
#define OD_Q16_028 18350
#define OD_Q16_030 19661
#define OD_Q16_020 13107
#define OD_Q16_N045 (-29491)
#define OD_Q16_036 23593
#define OD_Q16_064 41943
#define OD_Q16_THREE (3 * OD_Q16_ONE)

struct overdrive_i16_shape
{
    int32_t bias;
    int32_t zero;
    int32_t pos_gain;
    int32_t neg_gain;
};

struct overdrive_i16_state
{
    int32_t drive;
    int32_t low_drive;
    int32_t drive_delta;
    int32_t stage2_drive;
    int32_t bass_alpha;
    int32_t tone_alpha;
    int32_t mix;
    int32_t level;
    int32_t presence;
    int32_t dc_r;
    struct overdrive_i16_shape stage1_shape;
    struct overdrive_i16_shape stage2_shape;
    int32_t bass_lp[2];
    int32_t dc_prev_in[2];
    int32_t dc_prev_out[2];
    int32_t tone_lp1[2];
    int32_t tone_lp2[2];
};

struct overdrive_data
{
    float drive;
    float low_drive;
    float stage2_drive;
    float bass_alpha;
    float tone_alpha;
    float mix;
    float level;
    float asymmetry;
    float presence;
    float dc_r;
    float bass_lp[2];
    float dc_prev_in[2];
    float dc_prev_out[2];
    float tone_lp1[2];
    float tone_lp2[2];
    struct overdrive_i16_state fixed;
};

static int32_t overdrive_sat_i32(int64_t x)
{
    if (x > INT32_MAX)
        return INT32_MAX;
    if (x < INT32_MIN)
        return INT32_MIN;
    return (int32_t)x;
}

static int32_t overdrive_round_shift(int64_t x, unsigned shift)
{
    const int64_t half = (int64_t)1 << (shift - 1u);

    if (x >= 0)
        return overdrive_sat_i32((x + half) >> shift);

    return overdrive_sat_i32(-(((-x) + half) >> shift));
}

static int32_t overdrive_sub_i32(int32_t a, int32_t b)
{
    return overdrive_sat_i32((int64_t)a - (int64_t)b);
}

static int32_t overdrive_mul_q16(int32_t a, int32_t b)
{
    return overdrive_round_shift((int64_t)a * (int64_t)b, 16u);
}

static int32_t overdrive_mul_q30(int32_t value, int32_t coefficient)
{
    return overdrive_round_shift((int64_t)value * (int64_t)coefficient, 30u);
}

static int32_t overdrive_float_q16(float x)
{
    const float scaled = x * 65536.0f;

    if (scaled >= 2147483520.0f)
        return INT32_MAX;
    if (scaled <= -2147483648.0f)
        return INT32_MIN;

    return scaled >= 0.0f ? (int32_t)(scaled + 0.5f)
                          : (int32_t)(scaled - 0.5f);
}

static int32_t overdrive_float_q30(float x)
{
    const float scaled = x * 1073741824.0f;

    if (scaled >= 2147483520.0f)
        return INT32_MAX;
    if (scaled <= -2147483648.0f)
        return INT32_MIN;

    return scaled >= 0.0f ? (int32_t)(scaled + 0.5f)
                          : (int32_t)(scaled - 0.5f);
}

static int32_t overdrive_div_round_i64(int64_t numerator, int32_t denominator)
{
    int64_t magnitude;
    int64_t result;

    if (numerator >= 0)
        return overdrive_sat_i32((numerator + denominator / 2) / denominator);

    magnitude = -numerator;
    result = (magnitude + denominator / 2) / denominator;
    return overdrive_sat_i32(-result);
}

static int32_t overdrive_tanh_q16(int32_t x)
{
    int32_t x2;
    int32_t numerator_factor;
    int32_t denominator;
    int64_t numerator;

    if (x >= OD_Q16_THREE)
        return OD_Q16_ONE;
    if (x <= -OD_Q16_THREE)
        return -OD_Q16_ONE;

    x2 = overdrive_mul_q16(x, x);
    numerator_factor = 27 * OD_Q16_ONE + x2;
    denominator = 27 * OD_Q16_ONE + 9 * x2;
    numerator = (int64_t)x * (int64_t)numerator_factor;

    return overdrive_div_round_i64(numerator, denominator);
}

static void overdrive_setup_i16_shape(struct overdrive_i16_shape *shape,
    int32_t asymmetry)
{
    shape->bias = overdrive_mul_q16(asymmetry, OD_Q16_028);
    shape->pos_gain = OD_Q16_ONE +
        overdrive_mul_q16(asymmetry, OD_Q16_030);
    shape->neg_gain = OD_Q16_ONE -
        overdrive_mul_q16(asymmetry, OD_Q16_020);
    shape->zero = overdrive_tanh_q16(shape->bias);
}

static int32_t overdrive_shape_q16(int32_t x,
    const struct overdrive_i16_shape *shape)
{
    const int32_t gain = x >= 0 ? shape->pos_gain : shape->neg_gain;
    int32_t biased;
    int32_t saturated;

    biased = overdrive_sat_i32((int64_t)overdrive_mul_q16(x, gain) +
        shape->bias);
    saturated = overdrive_tanh_q16(biased);

    return overdrive_sat_i32((int64_t)saturated - shape->zero);
}

static int32_t overdrive_tick_i16(struct overdrive_i16_state *fx,
    unsigned ch, int32_t x)
{
    int32_t hp;
    int32_t driven;
    int32_t stage1;
    int32_t stage2;
    int32_t shaped;
    int32_t dc;
    int32_t delta;

    delta = overdrive_sub_i32(x, fx->bass_lp[ch]);
    fx->bass_lp[ch] = overdrive_sat_i32((int64_t)fx->bass_lp[ch] +
        overdrive_mul_q30(delta, fx->bass_alpha));
    hp = overdrive_sub_i32(x, fx->bass_lp[ch]);

    driven = overdrive_sat_i32(
        (int64_t)overdrive_mul_q16(x, fx->low_drive) +
        overdrive_mul_q16(hp, fx->drive_delta));


    stage1 = overdrive_shape_q16(driven, &fx->stage1_shape);
    stage2 = overdrive_shape_q16(
        overdrive_mul_q16(stage1, fx->stage2_drive),
        &fx->stage2_shape);
    shaped = overdrive_sat_i32(
        (int64_t)overdrive_mul_q16(stage1, OD_Q16_036) +
        overdrive_mul_q16(stage2, OD_Q16_064));


    dc = overdrive_sat_i32((int64_t)shaped - fx->dc_prev_in[ch] +
        overdrive_mul_q30(fx->dc_prev_out[ch], fx->dc_r));
    fx->dc_prev_in[ch] = shaped;
    fx->dc_prev_out[ch] = dc;

    delta = overdrive_sub_i32(dc, fx->tone_lp1[ch]);
    fx->tone_lp1[ch] = overdrive_sat_i32((int64_t)fx->tone_lp1[ch] +
        overdrive_mul_q30(delta, fx->tone_alpha));

    delta = overdrive_sub_i32(fx->tone_lp1[ch], fx->tone_lp2[ch]);
    fx->tone_lp2[ch] = overdrive_sat_i32((int64_t)fx->tone_lp2[ch] +
        overdrive_mul_q30(delta, fx->tone_alpha));

    return overdrive_sat_i32((int64_t)fx->tone_lp2[ch] +
        overdrive_mul_q16(
            overdrive_sub_i32(fx->tone_lp1[ch], fx->tone_lp2[ch]),
            fx->presence));
}

static int16_t overdrive_q16_to_i16(int32_t x)
{
    int64_t sample = x;

    if (sample >= 0)
        sample = (sample + 1) / 2;
    else
        sample = -(((-sample) + 1) / 2);

    if (sample > 32767)
        sample = 32767;
    else if (sample < -32768)
        sample = -32768;

    return (int16_t)sample;
}

static float overdrive_shape(float x, float asymmetry)
{
    const float bias = asymmetry * 0.28f;
    const float polarity_gain =
        x >= 0.0f ? 1.0f + 0.30f * asymmetry
                  : 1.0f - 0.20f * asymmetry;
    const float zero = tanhf(bias);

    x = dsp_clampf(x * polarity_gain + bias, -12.0f, 12.0f);
    return tanhf(x) - zero;
}

static float overdrive_tick(struct overdrive_data *od, unsigned ch, float x)
{
    float hp;
    float driven;
    float stage1;
    float stage2;
    float shaped;
    float dc;

    od->bass_lp[ch] += od->bass_alpha * (x - od->bass_lp[ch]);
    hp = x - od->bass_lp[ch];
    driven = x * od->low_drive + hp * (od->drive - od->low_drive);

    stage1 = overdrive_shape(driven, od->asymmetry);
    stage2 = overdrive_shape(stage1 * od->stage2_drive,
        -0.45f * od->asymmetry);
    shaped = 0.36f * stage1 + 0.64f * stage2;

    dc = shaped - od->dc_prev_in[ch] + od->dc_r * od->dc_prev_out[ch];
    od->dc_prev_in[ch] = shaped;
    od->dc_prev_out[ch] = dc;

    od->tone_lp1[ch] += od->tone_alpha * (dc - od->tone_lp1[ch]);
    od->tone_lp2[ch] += od->tone_alpha *
        (od->tone_lp1[ch] - od->tone_lp2[ch]);

    return od->tone_lp2[ch] +
        od->presence * (od->tone_lp1[ch] - od->tone_lp2[ch]);
}

static void overdrive_process(void *data, struct dspfilter_output *output,
    const struct dspfilter_input *input)
{
    struct overdrive_data *od = (struct overdrive_data *)data;
    float *samples = input->samples;
    unsigned i;

    output->samples = samples;
    output->frames = input->frames;

    if (!samples || input->frames == 0u)
        return;

    for (i = 0; i < input->frames; ++i)
    {
        unsigned ch;

        for (ch = 0; ch < 2u; ++ch)
        {
            const unsigned idx = i * 2u + ch;
            const float dry = samples[idx];
            const float wet = overdrive_tick(od, ch, dry) * od->level;

            samples[idx] = dsp_lerp(dry, wet, od->mix);
        }
    }
}

static void overdrive_process_i16(void *data,
    struct dspfilter_output_i16 *output,
    const struct dspfilter_input_i16 *input)
{
    struct overdrive_data *od = (struct overdrive_data *)data;
    struct overdrive_i16_state *fx = &od->fixed;
    int16_t *samples = input->samples;
    unsigned i;

    output->samples = samples;
    output->frames = input->frames;

    if (!samples || input->frames == 0u)
        return;

    for (i = 0; i < input->frames; ++i)
    {
        unsigned ch;

        for (ch = 0; ch < 2u; ++ch)
        {
            const unsigned idx = i * 2u + ch;
            const int32_t dry = (int32_t)samples[idx] * 2;
            int32_t wet = overdrive_tick_i16(fx, ch, dry);
            int32_t mixed;

            wet = overdrive_mul_q16(wet, fx->level);
            mixed = overdrive_sat_i32((int64_t)dry +
                overdrive_mul_q16(overdrive_sub_i32(wet, dry), fx->mix));
            samples[idx] = overdrive_q16_to_i16(mixed);
        }
    }
}

static void overdrive_free(void *data)
{
    if (data)
        free(data);
}

static void *overdrive_init(const struct dspfilter_info *info,
    const struct dspfilter_config *config, void *userdata)
{
    struct overdrive_data *od;
    struct overdrive_i16_state *fx;
    float drive_db = 18.0f;
    float bass_hz = 180.0f;
    float bass_tight = 0.78f;
    float tone_hz = 5600.0f;
    float presence = 0.18f;
    float mix = 1.0f;
    float level_db = -5.0f;
    float asymmetry = 0.20f;
    float drive_norm;
    float sr;
    float omega;
    int32_t asymmetry_q16;
    int32_t stage2_asymmetry_q16;

    if (!info || !config || info->input_rate <= 1.0f)
        return NULL;

    od = (struct overdrive_data *)calloc(1, sizeof(*od));
    if (!od)
        return NULL;

    config->get_float(userdata, "drive_db", &drive_db, 18.0f);
    config->get_float(userdata, "bass_hz", &bass_hz, 180.0f);
    config->get_float(userdata, "bass_tight", &bass_tight, 0.78f);
    config->get_float(userdata, "tone_hz", &tone_hz, 5600.0f);
    config->get_float(userdata, "presence", &presence, 0.18f);
    config->get_float(userdata, "drywet", &mix, 1.0f);
    config->get_float(userdata, "level_db", &level_db, -5.0f);
    config->get_float(userdata, "asymmetry", &asymmetry, 0.20f);

    sr = info->input_rate;
    drive_db = dsp_clampf(drive_db, 0.0f, 42.0f);
    bass_hz = dsp_clampf(bass_hz, 35.0f, 1400.0f);
    bass_tight = dsp_clampf(bass_tight, 0.0f, 1.0f);
    tone_hz = dsp_clampf(tone_hz, 1200.0f, sr * 0.45f);

    od->drive = dsp_db_to_gain(drive_db);
    od->low_drive = 1.0f + (od->drive - 1.0f) * (1.0f - bass_tight);

    drive_norm = drive_db * (1.0f / 42.0f);
    od->stage2_drive = 1.10f + 0.85f * drive_norm;

    omega = 2.0f * (float)M_PI * bass_hz / sr;
    od->bass_alpha = 1.0f - expf(-omega);

    omega = 2.0f * (float)M_PI * tone_hz / sr;
    od->tone_alpha = 1.0f - expf(-omega);

    od->mix = dsp_clampf(mix, 0.0f, 1.0f);
    od->level = dsp_db_to_gain(dsp_clampf(level_db, -30.0f, 12.0f));
    od->asymmetry = dsp_clampf(asymmetry, -1.0f, 1.0f);
    od->presence = dsp_clampf(presence, 0.0f, 1.0f);
    od->dc_r = expf(-2.0f * (float)M_PI * 7.0f / sr);

    fx = &od->fixed;
    fx->drive = overdrive_float_q16(od->drive);
    fx->low_drive = overdrive_float_q16(od->low_drive);
    fx->drive_delta = overdrive_sub_i32(fx->drive, fx->low_drive);
    fx->stage2_drive = overdrive_float_q16(od->stage2_drive);
    fx->bass_alpha = overdrive_float_q30(od->bass_alpha);
    fx->tone_alpha = overdrive_float_q30(od->tone_alpha);
    fx->mix = overdrive_float_q16(od->mix);
    fx->level = overdrive_float_q16(od->level);
    fx->presence = overdrive_float_q16(od->presence);
    fx->dc_r = overdrive_float_q30(od->dc_r);

    asymmetry_q16 = overdrive_float_q16(od->asymmetry);
    stage2_asymmetry_q16 = overdrive_mul_q16(asymmetry_q16,
        OD_Q16_N045);
    overdrive_setup_i16_shape(&fx->stage1_shape, asymmetry_q16);
    overdrive_setup_i16_shape(&fx->stage2_shape, stage2_asymmetry_q16);

    return od;
}

static const struct dspfilter_implementation overdrive_plug =
{
    overdrive_init,
    overdrive_process,
    overdrive_free,
    DSPFILTER_API_VERSION,
    "Guitar Distortion / Tube Overdrive",
    "overdrive_guitar",
    overdrive_process_i16
};

#ifdef HAVE_FILTERS_BUILTIN
#define dspfilter_get_implementation overdrive_guitar_dspfilter_get_implementation
#endif

const struct dspfilter_implementation *
dspfilter_get_implementation(dspfilter_simd_mask_t mask)
{
    (void)mask;
    return &overdrive_plug;
}

#undef dspfilter_get_implementation
