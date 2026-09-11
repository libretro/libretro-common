#include "libretro_dspfilter.h"
#include "dsp_common.h"

struct bitcrusher_data
{
    int bits;
    int downsample;
    float mix;
    unsigned hold_count;
    float held_l;
    float held_r;
    float quant_scale;
};

static float bitcrusher_quantize(float x, float scale)
{
    float q;
    x = dsp_clampf(x, -1.0f, 1.0f);
    if (x >= 0.0f)
        q = floorf(x * scale + 0.5f);
    else
        q = ceilf(x * scale - 0.5f);
    return q / scale;
}

static void bitcrusher_process(void *data, struct dspfilter_output *output,
    const struct dspfilter_input *input)
{
    struct bitcrusher_data *bc = (struct bitcrusher_data *)data;
    float *samples = input->samples;
    unsigned i;

    output->samples = samples;
    output->frames = input->frames;

    for (i = 0; i < input->frames; ++i)
    {
        float in_l = samples[i * 2u + 0u];
        float in_r = samples[i * 2u + 1u];

        if (bc->hold_count == 0u)
        {
            bc->held_l = bitcrusher_quantize(in_l, bc->quant_scale);
            bc->held_r = bitcrusher_quantize(in_r, bc->quant_scale);
        }

        samples[i * 2u + 0u] = dsp_lerp(in_l, bc->held_l, bc->mix);
        samples[i * 2u + 1u] = dsp_lerp(in_r, bc->held_r, bc->mix);

        bc->hold_count++;
        if (bc->hold_count >= (unsigned)bc->downsample)
            bc->hold_count = 0u;
    }
}

static void bitcrusher_free(void *data)
{
    free(data);
}

static void *bitcrusher_init(const struct dspfilter_info *info,
    const struct dspfilter_config *config, void *userdata)
{
    struct bitcrusher_data *bc;
    int bits = 8;
    int downsample = 4;
    float mix = 1.0f;

    (void)info;

    bc = (struct bitcrusher_data *)calloc(1, sizeof(*bc));
    if (!bc)
        return NULL;

    if (config)
    {
        config->get_int(userdata, "bits", &bits, 8);
        config->get_int(userdata, "downsample", &downsample, 4);
        config->get_float(userdata, "drywet", &mix, 1.0f);
    }

    bc->bits = dsp_clampi(bits, 2, 24);
    bc->downsample = dsp_clampi(downsample, 1, 64);
    bc->mix = dsp_clampf(mix, 0.0f, 1.0f);
    bc->hold_count = 0u;
    bc->quant_scale = (float)((1u << (unsigned)(bc->bits - 1)) - 1u);
    if (bc->quant_scale < 1.0f)
        bc->quant_scale = 1.0f;

    return bc;
}

static const struct dspfilter_implementation bitcrusher_plug =
{
    bitcrusher_init,
    bitcrusher_process,
    bitcrusher_free,
    DSPFILTER_API_VERSION,
    "Bitcrusher",
    "bitcrusher"
};

DSPFILTER_EXPORT const struct dspfilter_implementation *
#ifdef HAVE_FILTERS_BUILTIN
bitcrusher_dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#else
dspfilter_get_implementation(dspfilter_simd_mask_t mask)
#endif
{
    (void)mask;
    return &bitcrusher_plug;
}
