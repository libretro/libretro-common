#include "libretro_dspfilter.h"
#include "dsp_common.h"

#define RV_COMBS 4
#define RV_ALLPASS 2
#define RV_EARLY_TAPS 8

struct rv_delay
{
    float *buf;
    unsigned len;
    unsigned pos;
    float filter_store;
};

struct reverb_data
{
    float sample_rate;
    float dry;
    float wet;
    float early_mix;
    float late_mix;
    float width;
    float damping;
    float decay_sec;
    float diffusion;

    float *predelay[2];
    unsigned predelay_len;
    unsigned predelay_pos;

    float *early[2];
    unsigned early_len;
    unsigned early_pos;
    unsigned early_tap[RV_EARLY_TAPS];
    float early_gain_l[RV_EARLY_TAPS];
    float early_gain_r[RV_EARLY_TAPS];

    struct rv_delay comb[2][RV_COMBS];
    struct rv_delay allpass[2][RV_ALLPASS];
    float comb_feedback[2][RV_COMBS];
};

static int rv_alloc_delay(struct rv_delay *d, unsigned len)
{
    d->len = len < 2u ? 2u : len;
    d->pos = 0u;
    d->filter_store = 0.0f;
    d->buf = (float *)calloc(d->len, sizeof(float));
    return d->buf != NULL;
}

static void rv_free_delay(struct rv_delay *d)
{
    free(d->buf);
    d->buf = NULL;
    d->len = 0u;
    d->pos = 0u;
    d->filter_store = 0.0f;
}

static float rv_comb_process(struct rv_delay *d, float input,
    float feedback, float damping)
{
    float out = d->buf[d->pos];
    d->filter_store = out * (1.0f - damping) + d->filter_store * damping;
    d->buf[d->pos] = input + d->filter_store * feedback;
    d->pos++;
    if (d->pos >= d->len)
        d->pos = 0u;
    return out;
}

static float rv_allpass_process(struct rv_delay *d, float input, float g)
{
    float delayed = d->buf[d->pos];
    float out = delayed - input;
    d->buf[d->pos] = input + delayed * g;
    d->pos++;
    if (d->pos >= d->len)
        d->pos = 0u;
    return out;
}

static void reverb_free(void *data)
{
    struct reverb_data *rv = (struct reverb_data *)data;
    unsigned ch;
    unsigned i;

    if (!rv)
        return;

    for (ch = 0; ch < 2u; ++ch)
    {
        free(rv->predelay[ch]);
        free(rv->early[ch]);
        for (i = 0; i < RV_COMBS; ++i)
            rv_free_delay(&rv->comb[ch][i]);
        for (i = 0; i < RV_ALLPASS; ++i)
            rv_free_delay(&rv->allpass[ch][i]);
    }
    free(rv);
}

static void reverb_process(void *data, struct dspfilter_output *output,
    const struct dspfilter_input *input)
{
    struct reverb_data *rv = (struct reverb_data *)data;
    float *samples = input->samples;
    unsigned f;

    output->samples = samples;
    output->frames = input->frames;

    for (f = 0; f < input->frames; ++f)
    {
        float dry_l = samples[f * 2u + 0u];
        float dry_r = samples[f * 2u + 1u];
        float pre_l;
        float pre_r;
        float early_l = 0.0f;
        float early_r = 0.0f;
        float late_l = 0.0f;
        float late_r = 0.0f;
        float wet_l;
        float wet_r;
        unsigned t;
        unsigned i;

        pre_l = rv->predelay[0][rv->predelay_pos];
        pre_r = rv->predelay[1][rv->predelay_pos];
        rv->predelay[0][rv->predelay_pos] = dry_l;
        rv->predelay[1][rv->predelay_pos] = dry_r;

        rv->early[0][rv->early_pos] = pre_l;
        rv->early[1][rv->early_pos] = pre_r;

        for (t = 0; t < RV_EARLY_TAPS; ++t)
        {
            unsigned p = (rv->early_pos + rv->early_len - rv->early_tap[t]) %
                rv->early_len;
            float el = rv->early[0][p];
            float er = rv->early[1][p];

            early_l += el * rv->early_gain_l[t] + er * rv->early_gain_r[t] * 0.30f;
            early_r += er * rv->early_gain_r[t] + el * rv->early_gain_l[t] * 0.30f;
        }

        {
            float mono_feed = 0.5f * (pre_l + pre_r) + 0.20f * (early_l + early_r);
            for (i = 0; i < RV_COMBS; ++i)
            {
                late_l += rv_comb_process(&rv->comb[0][i], mono_feed,
                    rv->comb_feedback[0][i], rv->damping);
                late_r += rv_comb_process(&rv->comb[1][i], mono_feed,
                    rv->comb_feedback[1][i], rv->damping);
            }
            late_l *= 0.25f;
            late_r *= 0.25f;
        }

        for (i = 0; i < RV_ALLPASS; ++i)
        {
            float g = 0.45f + 0.30f * rv->diffusion;
            late_l = rv_allpass_process(&rv->allpass[0][i], late_l, g);
            late_r = rv_allpass_process(&rv->allpass[1][i], late_r, g);
        }

        wet_l = rv->early_mix * early_l + rv->late_mix * late_l;
        wet_r = rv->early_mix * early_r + rv->late_mix * late_r;

        {
            float mid = 0.5f * (wet_l + wet_r);
            float side = 0.5f * (wet_l - wet_r) * rv->width;
            wet_l = mid + side;
            wet_r = mid - side;
        }

        samples[f * 2u + 0u] = dry_l * rv->dry + wet_l * rv->wet;
        samples[f * 2u + 1u] = dry_r * rv->dry + wet_r * rv->wet;

        rv->predelay_pos++;
        if (rv->predelay_pos >= rv->predelay_len)
            rv->predelay_pos = 0u;

        rv->early_pos++;
        if (rv->early_pos >= rv->early_len)
            rv->early_pos = 0u;
    }
}

static void *reverb_init(const struct dspfilter_info *info,
    const struct dspfilter_config *config, void *userdata)
{
    static const float early_ms[RV_EARLY_TAPS] =
        { 5.3f, 8.7f, 13.1f, 17.9f, 24.7f, 31.3f, 41.1f, 53.7f };
    static const float early_gain[RV_EARLY_TAPS] =
        { 0.62f, 0.49f, 0.39f, 0.31f, 0.25f, 0.20f, 0.16f, 0.12f };
    static const float comb_ms[RV_COMBS] =
        { 25.31f, 26.94f, 28.96f, 30.75f };
    static const float ap_ms[RV_ALLPASS] =
        { 12.61f, 10.00f };

    struct reverb_data *rv;
    float predelay_ms = 18.0f;
    float room_size = 1.0f;
    float decay_sec = 2.6f;
    float damping = 0.42f;
    float diffusion = 0.72f;
    float early_mix = 0.42f;
    float late_mix = 0.78f;
    float width = 1.15f;
    float drywet = 0.35f;
    unsigned ch;
    unsigned i;
    float sr;

    if (!info || info->input_rate <= 1.0f)
        return NULL;

    config->get_float(userdata, "predelay_ms", &predelay_ms, 18.0f);
    config->get_float(userdata, "room_size", &room_size, 1.0f);
    config->get_float(userdata, "decay_sec", &decay_sec, 2.6f);
    config->get_float(userdata, "damping", &damping, 0.42f);
    config->get_float(userdata, "diffusion", &diffusion, 0.72f);
    config->get_float(userdata, "early_mix", &early_mix, 0.42f);
    config->get_float(userdata, "late_mix", &late_mix, 0.78f);
    config->get_float(userdata, "width", &width, 1.15f);
    config->get_float(userdata, "drywet", &drywet, 0.35f);

    predelay_ms = dsp_clampf(predelay_ms, 0.0f, 250.0f);
    room_size = dsp_clampf(room_size, 0.45f, 2.25f);
    decay_sec = dsp_clampf(decay_sec, 0.20f, 20.0f);
    damping = dsp_clampf(damping, 0.0f, 0.98f);
    diffusion = dsp_clampf(diffusion, 0.0f, 1.0f);
    early_mix = dsp_clampf(early_mix, 0.0f, 1.5f);
    late_mix = dsp_clampf(late_mix, 0.0f, 1.5f);
    width = dsp_clampf(width, 0.0f, 2.0f);
    drywet = dsp_clampf(drywet, 0.0f, 1.0f);

    rv = (struct reverb_data *)calloc(1, sizeof(*rv));
    if (!rv)
        return NULL;

    rv->sample_rate = info->input_rate;
    rv->dry = 1.0f - drywet;
    rv->wet = drywet;
    rv->early_mix = early_mix;
    rv->late_mix = late_mix;
    rv->width = width;
    rv->damping = damping;
    rv->decay_sec = decay_sec;
    rv->diffusion = diffusion;

    sr = info->input_rate;
    rv->predelay_len = (unsigned)(predelay_ms * 0.001f * sr + 0.5f);
    if (rv->predelay_len < 1u)
        rv->predelay_len = 1u;

    rv->early_len = (unsigned)(0.090f * room_size * sr + 8.0f);
    if (rv->early_len < 64u)
        rv->early_len = 64u;

    for (ch = 0; ch < 2u; ++ch)
    {
        rv->predelay[ch] = (float *)calloc(rv->predelay_len, sizeof(float));
        rv->early[ch] = (float *)calloc(rv->early_len, sizeof(float));
        if (!rv->predelay[ch] || !rv->early[ch])
        {
            reverb_free(rv);
            return NULL;
        }
    }

    for (i = 0; i < RV_EARLY_TAPS; ++i)
    {
        unsigned tap = (unsigned)(early_ms[i] * 0.001f * room_size * sr + 0.5f);
        if (tap >= rv->early_len)
            tap = rv->early_len - 1u;
        if (tap < 1u)
            tap = 1u;
        rv->early_tap[i] = tap;

        /* Alternating energy distribution creates decorrelated stereo ERs. */
        if ((i & 1u) == 0u)
        {
            rv->early_gain_l[i] = early_gain[i];
            rv->early_gain_r[i] = early_gain[i] * 0.72f;
        }
        else
        {
            rv->early_gain_l[i] = early_gain[i] * 0.72f;
            rv->early_gain_r[i] = early_gain[i];
        }
    }

    for (ch = 0; ch < 2u; ++ch)
    {
        for (i = 0; i < RV_COMBS; ++i)
        {
            float spread_ms = ch ? (0.73f + 0.17f * (float)i) : 0.0f;
            float delay_ms = (comb_ms[i] + spread_ms) * room_size;
            unsigned len = (unsigned)(delay_ms * 0.001f * sr + 0.5f);
            float delay_sec;

            if (!rv_alloc_delay(&rv->comb[ch][i], len))
            {
                reverb_free(rv);
                return NULL;
            }

            delay_sec = (float)rv->comb[ch][i].len / sr;
            rv->comb_feedback[ch][i] =
                powf(10.0f, (-3.0f * delay_sec) / decay_sec);
            rv->comb_feedback[ch][i] =
                dsp_clampf(rv->comb_feedback[ch][i], 0.0f, 0.985f);
        }

        for (i = 0; i < RV_ALLPASS; ++i)
        {
            float spread_ms = ch ? (0.31f + 0.11f * (float)i) : 0.0f;
            unsigned len = (unsigned)((ap_ms[i] + spread_ms) * 0.001f *
                room_size * sr + 0.5f);
            if (!rv_alloc_delay(&rv->allpass[ch][i], len))
            {
                reverb_free(rv);
                return NULL;
            }
        }
    }

    return rv;
}

static const struct dspfilter_implementation reverb_plug =
{
    reverb_init,
    reverb_process,
    reverb_free,
    DSPFILTER_API_VERSION,
    "Early Reflection Reverb",
    "earlyreverb"
};

DSPFILTER_EXPORT const struct dspfilter_implementation *
#ifdef HAVE_FILTERS_BUILTIN
earlyreverb_dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#else
dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#endif
{
    (void)mask;
    return &reverb_plug;
}
