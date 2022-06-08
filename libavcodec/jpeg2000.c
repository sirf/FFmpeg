/*
 * JPEG 2000 encoder and decoder common functions
 * Copyright (c) 2007 Kamil Nowosad
 * Copyright (c) 2013 Nicolas Bertrand <nicoinattendu@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * JPEG 2000 image encoder and decoder common functions
 */

#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/thread.h"
#include "avcodec.h"
#include "internal.h"
#include "jpeg2000.h"

#define SHL(a, n) ((n) >= 0 ? (a) << (n) : (a) >> -(n))

/* tag tree routines */

static int32_t tag_tree_size(int w, int h)
{
    int64_t res = 0;
    while (w > 1 || h > 1) {
        res += w * (int64_t)h;
        av_assert0(res + 1 < INT32_MAX);
        w = (w + 1) >> 1;
        h = (h + 1) >> 1;
    }
    return (int32_t)(res + 1);
}

#define T(x) (x*sizeof(Jpeg2000TgtNode))

static const size_t tt_sizes[16] = {
    T(1),T(3),T(6),T(7),T(3),T(5),T(9),T(11),T(6),T(9),T(14),T(17),T(7),T(11),T(17),T(21),
};

static const Jpeg2000TgtNode tt_stereotypes[16][21] = {
    {{-1},},
    {{2},{2},{-1},},
    {{3},{3},{4},{5},{5},{-1},},
    {{4},{4},{5},{5},{6},{6},{-1},},
    {{2},{2},{-1},},
    {{4},{4},{4},{4},{-1},},
    {{6},{6},{7},{6},{6},{7},{8},{8},{-1},},
    {{8},{8},{9},{9},{8},{8},{9},{9},{10},{10},{-1},},
    {{3},{3},{4},{5},{5},{-1},},
    {{6},{6},{6},{6},{7},{7},{8},{8},{-1},},
    {{9},{9},{10},{9},{9},{10},{11},{11},{12},{13},{13},{13},{13},{-1},},
    {{12},{12},{13},{13},{12},{12},{13},{13},{14},{14},{15},{15},{16},{16},{16},{16},{-1},},
    {{4},{4},{5},{5},{6},{6},{-1},},
    {{8},{8},{8},{8},{9},{9},{9},{9},{10},{10},{-1},},
    {{12},{12},{13},{12},{12},{13},{14},{14},{15},{14},{14},{15},{16},{16},{16},{16},{-1},},
    {{16},{16},{17},{17},{16},{16},{17},{17},{18},{18},{19},{19},{18},{18},{19},{19},{20},{20},{20},{20},{-1},},
};

/* allocate the memory for tag tree */
static int ff_jpeg2000_tag_tree_init(Jpeg2000TgtNode **old, unsigned int *size, int w, int h)
{
    int pw = w, ph = h;
    Jpeg2000TgtNode *t;
    int32_t tt_size, ofs = 0;
    size_t prod;

    if (w <= 4 && h <= 4) {
        int idx = w-1 + (h-1)*4;
        size_t sz = tt_sizes[idx];
        av_fast_malloc(old, size, sz);
        if (*old) {
            memcpy(*old, tt_stereotypes[idx], sz);
        }
        return 0;
    } else {
    tt_size = tag_tree_size(w, h);

    if (av_size_mult(tt_size, sizeof(*t), &prod))
        return AVERROR(ENOMEM);

    av_fast_malloc(old, size, prod);
    if (!*old)
        return AVERROR(ENOMEM);
    t = *old;
    memset(*old, 0, prod);

    while (w > 1 || h > 1) {
        int i, j;
        pw = w;
        ph = h;

        w  = (w + 1) >> 1;
        h  = (h + 1) >> 1;
        ofs += pw * ph;

        for (i = 0; i < ph; i++)
            for (j = 0; j < pw; j++)
                t[i * pw + j].parent = (i >> 1) * w + (j >> 1) + ofs;

        t += pw * ph;
    }
    t[0].parent = -1;
    return 0;
    }
}

void ff_tag_tree_zero(Jpeg2000TgtNode *t, int w, int h, int val)
{
    int i, siz = tag_tree_size(w, h);

    for (i = 0; i < siz; i++) {
        t[i].val = val;
        t[i].temp_val = 0;
        t[i].vis = 0;
    }
}

uint8_t ff_jpeg2000_sigctxno_lut[256][4];

static int getsigctxno(int flag, int bandno)
{
    int h, v, d;

    h = ((flag & JPEG2000_T1_SIG_E)  ? 1 : 0) +
        ((flag & JPEG2000_T1_SIG_W)  ? 1 : 0);
    v = ((flag & JPEG2000_T1_SIG_N)  ? 1 : 0) +
        ((flag & JPEG2000_T1_SIG_S)  ? 1 : 0);
    d = ((flag & JPEG2000_T1_SIG_NE) ? 1 : 0) +
        ((flag & JPEG2000_T1_SIG_NW) ? 1 : 0) +
        ((flag & JPEG2000_T1_SIG_SE) ? 1 : 0) +
        ((flag & JPEG2000_T1_SIG_SW) ? 1 : 0);

    if (bandno < 3) {
        if (bandno == 1)
            FFSWAP(int, h, v);
        if (h == 2) return 8;
        if (h == 1) {
            if (v >= 1) return 7;
            if (d >= 1) return 6;
            return 5;
        }
        if (v == 2) return 4;
        if (v == 1) return 3;
        if (d >= 2) return 2;
        if (d == 1) return 1;
    } else {
        if (d >= 3) return 8;
        if (d == 2) {
            if (h+v >= 1) return 7;
            return 6;
        }
        if (d == 1) {
            if (h+v >= 2) return 5;
            if (h+v == 1) return 4;
            return 3;
        }
        if (h+v >= 2) return 2;
        if (h+v == 1) return 1;
    }
    return 0;
}

uint8_t ff_jpeg2000_sgnctxno_lut[16][16], ff_jpeg2000_xorbit_lut[16][16];

static const int contribtab[3][3] = { {  0, -1,  1 }, { -1, -1,  0 }, {  1,  0,  1 } };
static const int  ctxlbltab[3][3] = { { 13, 12, 11 }, { 10,  9, 10 }, { 11, 12, 13 } };
static const int  xorbittab[3][3] = { {  1,  1,  1 }, {  1,  0,  0 }, {  0,  0,  0 } };

static int getsgnctxno(int flag, uint8_t *xorbit)
{
    int vcontrib, hcontrib;

    hcontrib = contribtab[flag & JPEG2000_T1_SIG_E ? flag & JPEG2000_T1_SGN_E ? 1 : 2 : 0]
                         [flag & JPEG2000_T1_SIG_W ? flag & JPEG2000_T1_SGN_W ? 1 : 2 : 0] + 1;
    vcontrib = contribtab[flag & JPEG2000_T1_SIG_S ? flag & JPEG2000_T1_SGN_S ? 1 : 2 : 0]
                         [flag & JPEG2000_T1_SIG_N ? flag & JPEG2000_T1_SGN_N ? 1 : 2 : 0] + 1;
    *xorbit = xorbittab[hcontrib][vcontrib];

    return ctxlbltab[hcontrib][vcontrib];
}

static void av_cold jpeg2000_init_tier1_luts(void)
{
    int i, j;
    for (i = 0; i < 256; i++)
        for (j = 0; j < 4; j++)
            ff_jpeg2000_sigctxno_lut[i][j] = getsigctxno(i, j);
    for (i = 0; i < 16; i++)
        for (j = 0; j < 16; j++)
            ff_jpeg2000_sgnctxno_lut[i][j] =
                getsgnctxno(i + (j << 8), &ff_jpeg2000_xorbit_lut[i][j]);
}

void av_cold ff_jpeg2000_init_tier1_luts(void)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    ff_thread_once(&init_static_once, jpeg2000_init_tier1_luts);
}

void ff_jpeg2000_set_significance(Jpeg2000T1Context *t1, int x, int y,
                                  int negative)
{
    x++;
    y++;
    t1->flags[(y) * t1->stride + x] |= JPEG2000_T1_SIG;
    if (negative) {
        t1->flags[(y) * t1->stride + x + 1] |= JPEG2000_T1_SIG_W | JPEG2000_T1_SGN_W;
        t1->flags[(y) * t1->stride + x - 1] |= JPEG2000_T1_SIG_E | JPEG2000_T1_SGN_E;
        t1->flags[(y + 1) * t1->stride + x] |= JPEG2000_T1_SIG_N | JPEG2000_T1_SGN_N;
        t1->flags[(y - 1) * t1->stride + x] |= JPEG2000_T1_SIG_S | JPEG2000_T1_SGN_S;
    } else {
        t1->flags[(y) * t1->stride + x + 1] |= JPEG2000_T1_SIG_W;
        t1->flags[(y) * t1->stride + x - 1] |= JPEG2000_T1_SIG_E;
        t1->flags[(y + 1) * t1->stride + x] |= JPEG2000_T1_SIG_N;
        t1->flags[(y - 1) * t1->stride + x] |= JPEG2000_T1_SIG_S;
    }
    t1->flags[(y + 1) * t1->stride + x + 1] |= JPEG2000_T1_SIG_NW;
    t1->flags[(y + 1) * t1->stride + x - 1] |= JPEG2000_T1_SIG_NE;
    t1->flags[(y - 1) * t1->stride + x + 1] |= JPEG2000_T1_SIG_SW;
    t1->flags[(y - 1) * t1->stride + x - 1] |= JPEG2000_T1_SIG_SE;
}

// static const uint8_t lut_gain[2][4] = { { 0, 0, 0, 0 }, { 0, 1, 1, 2 } }; (unused)

static void init_band_stepsize(AVCodecContext *avctx,
                               Jpeg2000Band *band,
                               Jpeg2000CodingStyle *codsty,
                               Jpeg2000QuantStyle *qntsty,
                               int bandno, int gbandno, int reslevelno,
                               int cbps)
{
    /* TODO: Implementation of quantization step not finished,
     * see ISO/IEC 15444-1:2002 E.1 and A.6.4. */
    switch (qntsty->quantsty) {
        uint8_t gain;
    case JPEG2000_QSTY_NONE:
        /* TODO: to verify. No quantization in this case */
        band->f_stepsize = 1;
        break;
    case JPEG2000_QSTY_SI:
        /*TODO: Compute formula to implement. */
//         numbps = cbps +
//                  lut_gain[codsty->transform == FF_DWT53][bandno + (reslevelno > 0)];
//         band->f_stepsize = SHL(2048 + qntsty->mant[gbandno],
//                                2 + numbps - qntsty->expn[gbandno]);
//         break;
    case JPEG2000_QSTY_SE:
        /* Exponent quantization step.
         * Formula:
         * delta_b = 2 ^ (R_b - expn_b) * (1 + (mant_b / 2 ^ 11))
         * R_b = R_I + log2 (gain_b )
         * see ISO/IEC 15444-1:2002 E.1.1 eqn. E-3 and E-4 */
        gain            = cbps;
        band->f_stepsize  = ff_exp2fi(gain - qntsty->expn[gbandno]);
        band->f_stepsize *= qntsty->mant[gbandno] / 2048.0 + 1.0;
        break;
    default:
        band->f_stepsize = 0;
        av_log(avctx, AV_LOG_ERROR, "Unknown quantization format\n");
        break;
    }
    if (codsty->transform != FF_DWT53) {
        int lband = 0;
        switch (bandno + (reslevelno > 0)) {
            case 1:
            case 2:
                band->f_stepsize *= F_LFTG_X * 2;
                lband = 1;
                break;
            case 3:
                band->f_stepsize *= F_LFTG_X * F_LFTG_X * 4;
                break;
        }
        if (codsty->transform == FF_DWT97) {
            band->f_stepsize *= pow(F_LFTG_K, 2*(codsty->nreslevels2decode - reslevelno) + lband - 2);
        }
    }

    if (band->f_stepsize > (INT_MAX >> 15)) {
        band->f_stepsize = 0;
        av_log(avctx, AV_LOG_ERROR, "stepsize out of range\n");
    }

    band->i_stepsize = band->f_stepsize * (1 << 15);

    /* FIXME: In OpenJPEG code stepsize = stepsize * 0.5. Why?
     * If not set output of entropic decoder is not correct. */
    if (!av_codec_is_encoder(avctx->codec))
        band->f_stepsize *= 0.5;
}

static int init_prec(AVCodecContext *avctx,
                     Jpeg2000Band *band,
                     Jpeg2000ResLevel *reslevel,
                     Jpeg2000Component *comp,
                     Jpeg2000CodingStyle *codsty,
                     int precno, int bandno, int reslevelno,
                     int log2_band_prec_width,
                     int log2_band_prec_height)
{
    Jpeg2000Prec *prec = band->prec + precno;
    int nb_codeblocks, cblkno;

    prec->decoded_layers = 0;

    /* TODO: Explain formula for JPEG200 DCINEMA. */
    /* TODO: Verify with previous count of codeblocks per band */

    /* Compute P_x0 */
    prec->coord[0][0] = ((reslevel->coord[0][0] >> reslevel->log2_prec_width) + precno % reslevel->num_precincts_x) *
                        (1 << log2_band_prec_width);

    /* Compute P_y0 */
    prec->coord[1][0] = ((reslevel->coord[1][0] >> reslevel->log2_prec_height) + precno / reslevel->num_precincts_x) *
                        (1 << log2_band_prec_height);

    /* Compute P_x1 */
    prec->coord[0][1] = prec->coord[0][0] +
                        (1 << log2_band_prec_width);
    prec->coord[0][0] = FFMAX(prec->coord[0][0], band->coord[0][0]);
    prec->coord[0][1] = FFMIN(prec->coord[0][1], band->coord[0][1]);

    /* Compute P_y1 */
    prec->coord[1][1] = prec->coord[1][0] +
                        (1 << log2_band_prec_height);
    prec->coord[1][0] = FFMAX(prec->coord[1][0], band->coord[1][0]);
    prec->coord[1][1] = FFMIN(prec->coord[1][1], band->coord[1][1]);

    prec->nb_codeblocks_width =
        ff_jpeg2000_ceildivpow2(prec->coord[0][1],
                                band->log2_cblk_width)
        - (prec->coord[0][0] >> band->log2_cblk_width);
    prec->nb_codeblocks_height =
        ff_jpeg2000_ceildivpow2(prec->coord[1][1],
                                band->log2_cblk_height)
        - (prec->coord[1][0] >> band->log2_cblk_height);

    /* \sum_{i=0}^\inf 4^-i = 4/3 */
    if (prec->nb_codeblocks_width * (uint64_t)prec->nb_codeblocks_height > INT32_MAX / 4 * 3) {
        return AVERROR(ENOMEM);
    }

    /* Tag trees initialization */
    if (ff_jpeg2000_tag_tree_init(&prec->cblkincl,
                                  &prec->cblkincl_size,
                                  prec->nb_codeblocks_width,
                                  prec->nb_codeblocks_height) ||
        ff_jpeg2000_tag_tree_init(&prec->zerobits,
                                  &prec->zerobits_size,
                                  prec->nb_codeblocks_width,
                                  prec->nb_codeblocks_height))
        return AVERROR(ENOMEM);

    nb_codeblocks = prec->nb_codeblocks_width * prec->nb_codeblocks_height;
    if (ff_fast_recalloc(&prec->cblk, &prec->cblk_size, nb_codeblocks, sizeof(*prec->cblk)))
        return AVERROR(ENOMEM);
    for (cblkno = 0; cblkno < nb_codeblocks; cblkno++) {
        Jpeg2000Cblk *cblk = prec->cblk + cblkno;
        int Cx0, Cy0;

        /* Compute coordinates of codeblocks */
        /* Compute Cx0*/
        Cx0 = ((prec->coord[0][0]) >> band->log2_cblk_width) << band->log2_cblk_width;
        Cx0 = Cx0 + ((cblkno % prec->nb_codeblocks_width)  << band->log2_cblk_width);
        cblk->coord[0][0] = FFMAX(Cx0, prec->coord[0][0]);

        /* Compute Cy0*/
        Cy0 = ((prec->coord[1][0]) >> band->log2_cblk_height) << band->log2_cblk_height;
        Cy0 = Cy0 + ((cblkno / prec->nb_codeblocks_width)   << band->log2_cblk_height);
        cblk->coord[1][0] = FFMAX(Cy0, prec->coord[1][0]);

        /* Compute Cx1 */
        cblk->coord[0][1] = FFMIN(Cx0 + (1 << band->log2_cblk_width),
                                  prec->coord[0][1]);

        /* Compute Cy1 */
        cblk->coord[1][1] = FFMIN(Cy0 + (1 << band->log2_cblk_height),
                                  prec->coord[1][1]);
        /* Update code-blocks coordinates according sub-band position */
        if ((bandno + !!reslevelno) & 1) {
            cblk->coord[0][0] += comp->reslevel[reslevelno-1].coord[0][1] -
                                 comp->reslevel[reslevelno-1].coord[0][0];
            cblk->coord[0][1] += comp->reslevel[reslevelno-1].coord[0][1] -
                                 comp->reslevel[reslevelno-1].coord[0][0];
        }
        if ((bandno + !!reslevelno) & 2) {
            cblk->coord[1][0] += comp->reslevel[reslevelno-1].coord[1][1] -
                                 comp->reslevel[reslevelno-1].coord[1][0];
            cblk->coord[1][1] += comp->reslevel[reslevelno-1].coord[1][1] -
                                 comp->reslevel[reslevelno-1].coord[1][0];
        }

        cblk->lblock    = 3;
        cblk->length    = 0;
        cblk->npasses   = 0;
        if (av_codec_is_encoder(avctx->codec)) {
            av_freep(&cblk->layers);
            cblk->layers = av_calloc(codsty->nlayers, sizeof(*cblk->layers));
            if (!cblk->layers)
                return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static int init_band(AVCodecContext *avctx,
                     Jpeg2000ResLevel *reslevel,
                     Jpeg2000Component *comp,
                     Jpeg2000CodingStyle *codsty,
                     Jpeg2000QuantStyle *qntsty,
                     int bandno, int gbandno, int reslevelno,
                     int cbps, int dx, int dy)
{
    Jpeg2000Band *band = reslevel->band + bandno;
    uint8_t log2_band_prec_width, log2_band_prec_height;
    int declvl = codsty->nreslevels - reslevelno;    // N_L -r see  ISO/IEC 15444-1:2002 B.5
    int precno;
    int nb_precincts;
    int i, j, ret;

    init_band_stepsize(avctx, band, codsty, qntsty, bandno, gbandno, reslevelno, cbps);

    /* computation of tbx_0, tbx_1, tby_0, tby_1
     * see ISO/IEC 15444-1:2002 B.5 eq. B-15 and tbl B.1
     * codeblock width and height is computed for
     * DCI JPEG 2000 codeblock_width = codeblock_width = 32 = 2 ^ 5 */
    if (reslevelno == 0) {
        /* for reslevelno = 0, only one band, x0_b = y0_b = 0 */
        for (i = 0; i < 2; i++)
            for (j = 0; j < 2; j++)
                band->coord[i][j] =
                    ff_jpeg2000_ceildivpow2(comp->coord_o[i][j],
                                            declvl - 1);
        log2_band_prec_width  = reslevel->log2_prec_width;
        log2_band_prec_height = reslevel->log2_prec_height;
        /* see ISO/IEC 15444-1:2002 eq. B-17 and eq. B-15 */
        band->log2_cblk_width  = FFMIN(codsty->log2_cblk_width,
                                       reslevel->log2_prec_width);
        band->log2_cblk_height = FFMIN(codsty->log2_cblk_height,
                                       reslevel->log2_prec_height);
    } else {
        /* 3 bands x0_b = 1 y0_b = 0; x0_b = 0 y0_b = 1; x0_b = y0_b = 1 */
        /* x0_b and y0_b are computed with ((bandno + 1 >> i) & 1) */
        for (i = 0; i < 2; i++)
            for (j = 0; j < 2; j++)
                /* Formula example for tbx_0 = ceildiv((tcx_0 - 2 ^ (declvl - 1) * x0_b) / declvl) */
                band->coord[i][j] =
                    ff_jpeg2000_ceildivpow2(comp->coord_o[i][j] -
                                            (((bandno + 1 >> i) & 1LL) << declvl - 1),
                                            declvl);
        /* TODO: Manage case of 3 band offsets here or
         * in coding/decoding function? */

        /* see ISO/IEC 15444-1:2002 eq. B-17 and eq. B-15 */
        band->log2_cblk_width  = FFMIN(codsty->log2_cblk_width,
                                       reslevel->log2_prec_width - 1);
        band->log2_cblk_height = FFMIN(codsty->log2_cblk_height,
                                       reslevel->log2_prec_height - 1);

        log2_band_prec_width  = reslevel->log2_prec_width  - 1;
        log2_band_prec_height = reslevel->log2_prec_height - 1;
    }

    if (reslevel->num_precincts_x * (uint64_t)reslevel->num_precincts_y > INT_MAX) {
        band->prec = NULL;
        return AVERROR(ENOMEM);
    }
    nb_precincts = reslevel->num_precincts_x * reslevel->num_precincts_y;
    if (ff_fast_recalloc(&band->prec, &band->prec_size, nb_precincts, sizeof(*band->prec)))
        return AVERROR(ENOMEM);

    for (precno = 0; precno < nb_precincts; precno++) {
        ret = init_prec(avctx, band, reslevel, comp, codsty,
                        precno, bandno, reslevelno,
                        log2_band_prec_width, log2_band_prec_height);
        if (ret < 0)
            return ret;
    }

    return 0;
}

int ff_jpeg2000_init_component(Jpeg2000Component *comp,
                               Jpeg2000CodingStyle *codsty,
                               Jpeg2000QuantStyle *qntsty,
                               int cbps, int dx, int dy,
                               AVCodecContext *avctx, int max_slices)
{
    int reslevelno, bandno, gbandno = 0, ret, i, j;
    uint32_t csize;
    size_t prod;

    if (codsty->nreslevels2decode <= 0) {
        av_log(avctx, AV_LOG_ERROR, "nreslevels2decode %d invalid or uninitialized\n", codsty->nreslevels2decode);
        return AVERROR_INVALIDDATA;
    }

    if (ret = ff_jpeg2000_dwt_init(&comp->dwt, comp->coord,
                                   codsty->nreslevels2decode - 1,
                                   codsty->transform,
                                   max_slices))
        return ret;

    if (av_image_check_size(comp->coord[0][1] - comp->coord[0][0],
                            comp->coord[1][1] - comp->coord[1][0], 0, avctx))
        return AVERROR_INVALIDDATA;
    csize = (comp->coord[0][1] - comp->coord[0][0]) *
            (comp->coord[1][1] - comp->coord[1][0]);
    if (comp->coord[0][1] - comp->coord[0][0] > 32768 ||
        comp->coord[1][1] - comp->coord[1][0] > 32768) {
        av_log(avctx, AV_LOG_ERROR, "component size too large\n");
        return AVERROR_PATCHWELCOME;
    }

    if (codsty->transform == FF_DWT97) {
        csize += AV_INPUT_BUFFER_PADDING_SIZE / sizeof(*comp->f_data);
        if (av_size_mult(csize, sizeof(*comp->f_data), &prod))
            return AVERROR(ENOMEM);
        av_fast_malloc(&comp->f_data, &comp->f_data_size, prod);
        if (!comp->f_data)
            return AVERROR(ENOMEM);
        memset(comp->f_data, 0, prod);
    } else {
        csize += AV_INPUT_BUFFER_PADDING_SIZE / sizeof(*comp->i_data);
        if (av_size_mult(csize, sizeof(*comp->i_data), &prod))
            return AVERROR(ENOMEM);
        av_fast_malloc(&comp->i_data, &comp->i_data_size, prod);
        if (!comp->i_data)
            return AVERROR(ENOMEM);
        memset(comp->i_data, 0, prod);
    }
    if (ff_fast_recalloc(&comp->reslevel, &comp->reslevel_size, codsty->nreslevels, sizeof(*comp->reslevel)))
        return AVERROR(ENOMEM);
    /* LOOP on resolution levels */
    for (reslevelno = 0; reslevelno < codsty->nreslevels; reslevelno++) {
        int declvl = codsty->nreslevels - reslevelno;    // N_L -r see  ISO/IEC 15444-1:2002 B.5
        Jpeg2000ResLevel *reslevel = comp->reslevel + reslevelno;

        /* Compute borders for each resolution level.
         * Computation of trx_0, trx_1, try_0 and try_1.
         * see ISO/IEC 15444-1:2002 eq. B.5 and B-14 */
        for (i = 0; i < 2; i++)
            for (j = 0; j < 2; j++)
                reslevel->coord[i][j] =
                    ff_jpeg2000_ceildivpow2(comp->coord_o[i][j], declvl - 1);
        // update precincts size: 2^n value
        reslevel->log2_prec_width  = codsty->log2_prec_widths[reslevelno];
        reslevel->log2_prec_height = codsty->log2_prec_heights[reslevelno];

        /* Number of bands for each resolution level */
        if (reslevelno == 0)
            reslevel->nbands = 1;
        else
            reslevel->nbands = 3;

        /* Number of precincts which span the tile for resolution level reslevelno
         * see B.6 in ISO/IEC 15444-1:2002 eq. B-16
         * num_precincts_x = |- trx_1 / 2 ^ log2_prec_width) -| - (trx_0 / 2 ^ log2_prec_width)
         * num_precincts_y = |- try_1 / 2 ^ log2_prec_width) -| - (try_0 / 2 ^ log2_prec_width)
         * for Dcinema profiles in JPEG 2000
         * num_precincts_x = |- trx_1 / 2 ^ log2_prec_width) -|
         * num_precincts_y = |- try_1 / 2 ^ log2_prec_width) -| */
        if (reslevel->coord[0][1] == reslevel->coord[0][0])
            reslevel->num_precincts_x = 0;
        else
            reslevel->num_precincts_x =
                ff_jpeg2000_ceildivpow2(reslevel->coord[0][1],
                                        reslevel->log2_prec_width) -
                (reslevel->coord[0][0] >> reslevel->log2_prec_width);

        if (reslevel->coord[1][1] == reslevel->coord[1][0])
            reslevel->num_precincts_y = 0;
        else
            reslevel->num_precincts_y =
                ff_jpeg2000_ceildivpow2(reslevel->coord[1][1],
                                        reslevel->log2_prec_height) -
                (reslevel->coord[1][0] >> reslevel->log2_prec_height);

        if (ff_fast_recalloc(&reslevel->band, &reslevel->band_size, reslevel->nbands, sizeof(*reslevel->band)))
            return AVERROR(ENOMEM);

        if (reslevel->num_precincts_x * (uint64_t)reslevel->num_precincts_y * reslevel->nbands > avctx->max_pixels / sizeof(*reslevel->band->prec))
            return AVERROR(ENOMEM);

        for (bandno = 0; bandno < reslevel->nbands; bandno++, gbandno++) {
            ret = init_band(avctx, reslevel,
                            comp, codsty, qntsty,
                            bandno, gbandno, reslevelno,
                            cbps, dx, dy);
            if (ret < 0)
                return ret;
        }
    }
    return 0;
}

void ff_jpeg2000_reinit(Jpeg2000Component *comp, Jpeg2000CodingStyle *codsty)
{
    int reslevelno, bandno, cblkno, precno;
    for (reslevelno = 0; reslevelno < codsty->nreslevels; reslevelno++) {
        Jpeg2000ResLevel *rlevel = comp->reslevel + reslevelno;
        for (bandno = 0; bandno < rlevel->nbands; bandno++) {
            Jpeg2000Band *band = rlevel->band + bandno;
            for(precno = 0; precno < rlevel->num_precincts_x * rlevel->num_precincts_y; precno++) {
                Jpeg2000Prec *prec = band->prec + precno;
                ff_tag_tree_zero(prec->zerobits, prec->nb_codeblocks_width, prec->nb_codeblocks_height, 0);
                ff_tag_tree_zero(prec->cblkincl, prec->nb_codeblocks_width, prec->nb_codeblocks_height, 0);
                for (cblkno = 0; cblkno < prec->nb_codeblocks_width * prec->nb_codeblocks_height; cblkno++) {
                    Jpeg2000Cblk *cblk = prec->cblk + cblkno;
                    cblk->length = 0;
                    cblk->lblock = 3;
                }
            }
        }
    }
}

void ff_jpeg2000_cleanup(Jpeg2000Component *comp, Jpeg2000CodingStyle *codsty)
{
    int reslevelno, bandno, precno;
    for (reslevelno = 0;
         comp->reslevel && reslevelno < comp->reslevel_size/sizeof(*comp->reslevel);
         reslevelno++) {
        Jpeg2000ResLevel *reslevel;

        if (!comp->reslevel)
            continue;

        reslevel = comp->reslevel + reslevelno;
        for (bandno = 0; bandno < reslevel->band_size/sizeof(*reslevel->band); bandno++) {
            Jpeg2000Band *band;

            if (!reslevel->band)
                continue;

            band = reslevel->band + bandno;
            for (precno = 0; precno < band->prec_size/sizeof(*band->prec); precno++) {
                if (band->prec) {
                    Jpeg2000Prec *prec = band->prec + precno;
                    av_freep(&prec->zerobits);
                    av_freep(&prec->cblkincl);
                    if (prec->cblk) {
                        int cblkno;
                        for (cblkno = 0; cblkno < prec->cblk_size/sizeof(*prec->cblk); cblkno ++) {
                            Jpeg2000Cblk *cblk = &prec->cblk[cblkno];
                            av_freep(&cblk->data);
                            av_freep(&cblk->passes);
                            av_freep(&cblk->lengthinc);
                            av_freep(&cblk->data_start);
                            av_freep(&cblk->layers);
                        }
                        av_freep(&prec->cblk);
                    }
                }
            }

            av_freep(&band->prec);
        }
        av_freep(&reslevel->band);
    }

    ff_dwt_destroy(&comp->dwt);
    av_freep(&comp->reslevel);
    av_freep(&comp->i_data);
    av_freep(&comp->f_data);
}
