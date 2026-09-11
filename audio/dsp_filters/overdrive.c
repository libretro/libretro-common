#include "libretro_dspfilter.h"
#include "dsp_common.h"

struct overdrive_data
{
    float drive;
    float tone_alpha;
    float mix;
    float level;
    float asymmetry;
    float dc_r;
    float prev_in[2];
    float prev_hp[2];
    float tone_lp[2];
};

static float overdrive_shape(float x)
{
    return tanhf(x);
}

static void overdrive_process(void *data, struct dspfilter_output *output,
    const struct dspfilter_input *input)
{
    struct overdrive_data *od = (struct overdrive_data *)data;
    float *samples = input->samples;
    unsigned i;

    output->samples = samples;
    output->frames = input->frames;

    for (i = 0; i < input->frames; ++i)
    {
        unsigned ch;
        for (ch = 0; ch < 2u; ++ch)
        {
            unsigned idx = i * 2u + ch;
            float dry = samples[idx];
            float driven = dry * od->drive;
            float shaped;
            float hp;
            float wet;

            driven += od->asymmetry * driven * driven * 0.15f;
            driven = dsp_clampf(driven, -12.0f, 12.0f);
            shaped = overdrive_shape(driven);

            /* DC blocker, important when asymmetric drive is used. */
            hp = shaped - od->prev_in[ch] + od->dc_r * od->prev_hp[ch];
            od->prev_in[ch] = shaped;
            od->prev_hp[ch] = hp;

            /* One-pole low-pass tone stage with a small direct component. */
            od->tone_lp[ch] += od->tone_alpha * (hp - od->tone_lp[ch]);
            wet = 0.30f * hp + 0.70f * od->tone_lp[ch];
            wet *= od->level;

            samples[idx] = dsp_lerp(dry, wet, od->mix);
        }
    }
}

static void overdrive_free(void *data)
{
    free(data);
}

static void *overdrive_init(const struct dspfilter_info *info,
    const struct dspfilter_config *config, void *userdata)
{
    struct overdrive_data *od;
    float drive_db = 18.0f;
    float tone_hz = 6500.0f;
    float mix = 1.0f;
    float level_db = -4.0f;
    float asymmetry = 0.12f;
    float sr;
    float omega;

    if (!info || info->input_rate <= 1.0f)
        return NULL;

    od = (struct overdrive_data *)calloc(1, sizeof(*od));
    if (!od)
        return NULL;

    config->get_float(userdata, "drive_db", &drive_db, 18.0f);
    config->get_float(userdata, "tone_hz", &tone_hz, 6500.0f);
    config->get_float(userdata, "drywet", &mix, 1.0f);
    config->get_float(userdata, "level_db", &level_db, -4.0f);
    config->get_float(userdata, "asymmetry", &asymmetry, 0.12f);

    sr = info->input_rate;
    tone_hz = dsp_clampf(tone_hz, 200.0f, sr * 0.45f);
    omega = 2.0f * (float)M_PI * tone_hz / sr;

    od->drive = dsp_db_to_gain(dsp_clampf(drive_db, 0.0f, 42.0f));
    od->tone_alpha = 1.0f - expf(-omega);
    od->mix = dsp_clampf(mix, 0.0f, 1.0f);
    od->level = dsp_db_to_gain(dsp_clampf(level_db, -30.0f, 12.0f));
    od->asymmetry = dsp_clampf(asymmetry, 0.0f, 1.0f);
    od->dc_r = 0.995f;

    return od;
}

static const struct dspfilter_implementation overdrive_plug =
{
    overdrive_init,
    overdrive_process,
    overdrive_free,
    DSPFILTER_API_VERSION,
    "Distortion / Overdrive",
    "overdrive"
};

DSPFILTER_EXPORT const struct dspfilter_implementation *
#ifdef HAVE_FILTERS_BUILTIN
overdrive_dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#else
dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#endif
{
    (void)mask;
    return &overdrive_plug;
}
