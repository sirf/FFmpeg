/*
 * JPEG 2000 common defines, structures and functions
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

#ifndef AVCODEC_JPEG2000_H
#define AVCODEC_JPEG2000_H

/**
 * @file
 * JPEG 2000 structures and defines common
 * to encoder and decoder
 */

#include <stdint.h>

#include "avcodec.h"
#include "mqc.h"
#include "bytestream.h"
#include "jpeg2000dwt.h"
#include "jpeg2000dsp.h"
enum Jpeg2000Markers {
    JPEG2000_SOC = 0xff4f, // start of codestream
    JPEG2000_SIZ = 0xff51, // image and tile size
    JPEG2000_COD,          // coding style default
    JPEG2000_COC,          // coding style component
    JPEG2000_TLM = 0xff55, // tile-part length, main header
    JPEG2000_PLM = 0xff57, // packet length, main header
    JPEG2000_PLT,          // packet length, tile-part header
    JPEG2000_QCD = 0xff5c, // quantization default
    JPEG2000_QCC,          // quantization component
    JPEG2000_RGN,          // region of interest
    JPEG2000_POC,          // progression order change
    JPEG2000_PPM,          // packed packet headers, main header
    JPEG2000_PPT,          // packed packet headers, tile-part header
    JPEG2000_CRG = 0xff63, // component registration
    JPEG2000_COM,          // comment
    JPEG2000_SOT = 0xff90, // start of tile-part
    JPEG2000_SOP,          // start of packet
    JPEG2000_EPH,          // end of packet header
    JPEG2000_SOD,          // start of data
    JPEG2000_EOC = 0xffd9, // end of codestream
};

#define JPEG2000_SOP_FIXED_BYTES 0xFF910004
#define JPEG2000_SOP_BYTE_LENGTH 6

enum Jpeg2000Quantsty { // quantization style
    JPEG2000_QSTY_NONE, // no quantization
    JPEG2000_QSTY_SI,   // scalar derived
    JPEG2000_QSTY_SE    // scalar expounded
};

#define JPEG2000_MAX_DECLEVELS 33
#define JPEG2000_MAX_RESLEVELS (JPEG2000_MAX_DECLEVELS + 1)

#define JPEG2000_MAX_PASSES 100

// T1 flags
// flags determining significance of neighbor coefficients
#define JPEG2000_T1_SIG_N  0x0001
#define JPEG2000_T1_SIG_E  0x0002
#define JPEG2000_T1_SIG_W  0x0004
#define JPEG2000_T1_SIG_S  0x0008
#define JPEG2000_T1_SIG_NE 0x0010
#define JPEG2000_T1_SIG_NW 0x0020
#define JPEG2000_T1_SIG_SE 0x0040
#define JPEG2000_T1_SIG_SW 0x0080
#define JPEG2000_T1_SIG_NB (JPEG2000_T1_SIG_N  | JPEG2000_T1_SIG_E  |   \
                            JPEG2000_T1_SIG_S  | JPEG2000_T1_SIG_W  |   \
                            JPEG2000_T1_SIG_NE | JPEG2000_T1_SIG_NW |   \
                            JPEG2000_T1_SIG_SE | JPEG2000_T1_SIG_SW)
// flags determining sign bit of neighbor coefficients
#define JPEG2000_T1_SGN_N  0x0100
#define JPEG2000_T1_SGN_S  0x0200
#define JPEG2000_T1_SGN_W  0x0400
#define JPEG2000_T1_SGN_E  0x0800

#define JPEG2000_T1_VIS    0x1000
#define JPEG2000_T1_SIG    0x2000
#define JPEG2000_T1_REF    0x4000

#define JPEG2000_T1_SGN    0x8000

// Codeblock coding styles
#define JPEG2000_CBLK_BYPASS    0x01 // Selective arithmetic coding bypass
#define JPEG2000_CBLK_RESET     0x02 // Reset context probabilities
#define JPEG2000_CBLK_TERMALL   0x04 // Terminate after each coding pass
#define JPEG2000_CBLK_VSC       0x08 // Vertical stripe causal context formation
#define JPEG2000_CBLK_PREDTERM  0x10 // Predictable termination
#define JPEG2000_CBLK_SEGSYM    0x20 // Segmentation symbols present

// Coding styles
#define JPEG2000_CSTY_PREC      0x01 // Precincts defined in coding style
#define JPEG2000_CSTY_SOP       0x02 // SOP marker present
#define JPEG2000_CSTY_EPH       0x04 // EPH marker present
#define JPEG2000_CTSY_HTJ2K_F   0x40 // Only HT code-blocks (Rec. ITU-T T.814 | ISO/IEC 15444-15) are present
#define JPEG2000_CTSY_HTJ2K_M   0xC0 // HT code blocks (Rec. ITU-T T.814 | ISO/IEC 15444-15) can be present

// Progression orders
#define JPEG2000_PGOD_LRCP      0x00  // Layer-resolution level-component-position progression
#define JPEG2000_PGOD_RLCP      0x01  // Resolution level-layer-component-position progression
#define JPEG2000_PGOD_RPCL      0x02  // Resolution level-position-component-layer progression
#define JPEG2000_PGOD_PCRL      0x03  // Position-component-resolution level-layer progression
#define JPEG2000_PGOD_CPRL      0x04  // Component-position-resolution level-layer progression

typedef struct Jpeg2000T1Context {
    int data[6144];
    uint16_t flags[6156];
    MqcState mqc;
    int stride;
} Jpeg2000T1Context;

typedef struct Jpeg2000TgtNode {
    int32_t parent;
    uint8_t val;
    uint8_t temp_val;
    uint8_t vis;
} Jpeg2000TgtNode;

typedef struct Jpeg2000CodingStyle {
    int nreslevels;           // number of resolution levels
    int nreslevels2decode;    // number of resolution levels to decode
    uint8_t log2_cblk_width,
            log2_cblk_height; // exponent of codeblock size
    uint8_t transform;        // DWT type
    uint8_t csty;             // coding style
    uint8_t nlayers;          // number of layers
    uint8_t mct;              // multiple component transformation
    uint8_t cblk_style;       // codeblock coding style
    uint8_t prog_order;       // progression order
    uint8_t log2_prec_widths[JPEG2000_MAX_RESLEVELS];  // precincts size according resolution levels
    uint8_t log2_prec_heights[JPEG2000_MAX_RESLEVELS]; // TODO: initialize prec_size array with 0?
    uint8_t init;
} Jpeg2000CodingStyle;

typedef struct Jpeg2000QuantStyle {
    uint8_t expn[JPEG2000_MAX_DECLEVELS * 3];  // quantization exponent
    uint16_t mant[JPEG2000_MAX_DECLEVELS * 3]; // quantization mantissa
    uint8_t quantsty;      // quantization style
    uint8_t nguardbits;    // number of guard bits
} Jpeg2000QuantStyle;

typedef struct Jpeg2000Pass {
    uint16_t rate;
    int64_t disto;
    uint8_t flushed[4];
    int flushed_len;
} Jpeg2000Pass;

typedef struct Jpeg2000Layer {
    uint8_t *data_start;
    int data_len;
    int npasses;
    double disto;
    int cum_passes;
} Jpeg2000Layer;
typedef struct Jpeg2000Cblk {
    uint8_t npasses;
    uint8_t ninclpasses; // number coding of passes included in codestream
    uint8_t nonzerobits;
    uint8_t incl;
    uint16_t length;
    uint16_t *lengthinc;
    unsigned int lengthinc_size;
    uint8_t nb_lengthinc;
    uint8_t lblock;
    uint8_t zbp;         // Zero bit planes
    uint8_t *data;
    size_t data_allocated;
    int nb_terminations;
    int nb_terminationsinc;
    int *data_start;
    Jpeg2000Pass *passes;
    Jpeg2000Layer *layers;
    int coord[2][2]; // border coordinates {{x0, x1}, {y0, y1}}
    int pass_lengths[2];
} Jpeg2000Cblk; // code block

typedef struct Jpeg2000Prec {
    int nb_codeblocks_width;
    int nb_codeblocks_height;
    Jpeg2000TgtNode *zerobits;
    unsigned int zerobits_size;
    Jpeg2000TgtNode *cblkincl;
    unsigned int cblkincl_size;
    Jpeg2000Cblk *cblk;
    unsigned int cblk_size;
    int decoded_layers;
    int coord[2][2]; // border coordinates {{x0, x1}, {y0, y1}}
} Jpeg2000Prec; // precinct

typedef struct Jpeg2000Band {
    int coord[2][2]; // border coordinates {{x0, x1}, {y0, y1}}
    uint16_t log2_cblk_width, log2_cblk_height;
    int i_stepsize; // quantization stepsize
    float f_stepsize; // quantization stepsize
    Jpeg2000Prec *prec;
    unsigned int prec_size;
} Jpeg2000Band; // subband

typedef struct Jpeg2000ResLevel {
    uint8_t nbands;
    int coord[2][2]; // border coordinates {{x0, x1}, {y0, y1}}
    int num_precincts_x, num_precincts_y; // number of precincts in x/y direction
    uint8_t log2_prec_width, log2_prec_height; // exponent of precinct size
    Jpeg2000Band *band;
    unsigned int band_size;
} Jpeg2000ResLevel; // resolution level

typedef struct Jpeg2000Component {
    Jpeg2000ResLevel *reslevel;
    unsigned int reslevel_size;
    DWTContext dwt;
    float *f_data;
    unsigned int f_data_size;
    int *i_data;
    unsigned int i_data_size;
    int coord[2][2];   // border coordinates {{x0, x1}, {y0, y1}} -- can be reduced with lowres option
    int coord_o[2][2]; // border coordinates {{x0, x1}, {y0, y1}} -- original values from jpeg2000 headers
    uint8_t roi_shift; // ROI scaling value for the component
} Jpeg2000Component;

#define JP2_SIG_TYPE    0x6A502020
#define JP2_SIG_VALUE   0x0D0A870A
#define JP2_CODESTREAM  0x6A703263
#define JP2_HEADER      0x6A703268

#define HAD_COC 0x01
#define HAD_QCC 0x02

#define MAX_POCS 32

typedef struct Jpeg2000POCEntry {
    uint16_t LYEpoc;
    uint16_t CSpoc;
    uint16_t CEpoc;
    uint8_t RSpoc;
    uint8_t REpoc;
    uint8_t Ppoc;
} Jpeg2000POCEntry;

typedef struct Jpeg2000POC {
    Jpeg2000POCEntry poc[MAX_POCS];
    int nb_poc;
    int is_default;
} Jpeg2000POC;

typedef struct Jpeg2000TilePart {
    uint8_t tile_index;                 // Tile index who refers the tile-part
    const uint8_t *tp_end;
    GetByteContext header_tpg;          // bit stream of header if PPM header is used
    GetByteContext tpg;                 // bit stream in tile-part
} Jpeg2000TilePart;

/* RMK: For JPEG2000 DCINEMA 3 tile-parts in a tile
 * one per component, so tile_part elements have a size of 3 */
typedef struct Jpeg2000Tile {
    Jpeg2000Component   *comp;
    unsigned int        comp_size;
    uint8_t             properties[4];
    Jpeg2000CodingStyle codsty[4];
    Jpeg2000QuantStyle  qntsty[4];
    Jpeg2000POC         poc;
    Jpeg2000TilePart    tile_part[32];
    uint8_t             has_ppt;                // whether this tile has a ppt marker
    uint8_t             *packed_headers;        // contains packed headers. Used only along with PPT marker
    int                 packed_headers_size;    // size in bytes of the packed headers
    GetByteContext      packed_headers_stream;  // byte context corresponding to packed headers
    uint16_t            tp_idx;                  // Tile-part index
    int                 coord[2][2];             // border coordinates {{x0, x1}, {y0, y1}}
} Jpeg2000DecTile;

typedef struct Jpeg2000IdwtThread {
    int cb_start, cb_end;
} Jpeg2000IdwtThread;

typedef struct Jpeg2000CodeblockThread {
    int tileno, compno, reslevelno, bandno, precno, cblkno;
    int coded;
    int subbandno;
} Jpeg2000CodeblockThread;

typedef struct Jpeg2000DecoderContext {
    AVClass         *class;
    AVCodecContext  *avctx;
    GetByteContext  g;

    int             width, height;
    int             image_offset_x, image_offset_y;
    int             tile_offset_x, tile_offset_y;
    uint8_t         cbps[4];    // bits per sample in particular components
    uint8_t         sgnd[4];    // if a component is signed
    uint8_t         properties[4];

    uint8_t         has_ppm;
    uint8_t         *packed_headers; // contains packed headers. Used only along with PPM marker
    int             packed_headers_size;
    GetByteContext  packed_headers_stream;
    uint8_t         in_tile_headers;

    int             cdx[4], cdy[4];
    int             precision;
    int             ncomponents;
    int             colour_space;
    uint32_t        palette[256];
    int8_t          pal8;
    int             cdef[4];
    int             tile_width, tile_height;
    unsigned        numXtiles, numYtiles;
    int             maxtilelen;
    AVRational      sar;

    Jpeg2000CodingStyle codsty[4];
    Jpeg2000QuantStyle  qntsty[4];
    Jpeg2000POC         poc;
    uint8_t             roi_shift[4];

    int             bit_index;

    int             curtileno;

    Jpeg2000DecTile *tile;
    unsigned int    tile_size;
    Jpeg2000DSPContext dsp;

    /*options parameters*/
    int             reduction_factor;
    /*HTJ2K params*/
    uint8_t         is_htj2k;

    Jpeg2000IdwtThread *idwt;
    unsigned int idwt_size;
    Jpeg2000CodeblockThread *cb;
    unsigned int cb_size;

    // used for idwt slicing
    int reslevel, dir, slices;
    int have_dwt97_int; // 1 if any coding style is FF_DWT97_INT
    int have_mct;
} Jpeg2000DecoderContext;

/* misc tools */
static inline int ff_jpeg2000_ceildivpow2(int a, int b)
{
    return -((-(int64_t)a) >> b);
}

static inline int ff_jpeg2000_ceildiv(int a, int64_t b)
{
    return (a + b - 1) / b;
}

/* TIER-1 routines */

/* Set up lookup tables used in TIER-1. */
void ff_jpeg2000_init_tier1_luts(void);

/* Update significance of a coefficient at current position (x,y) and
 * for neighbors. */
void ff_jpeg2000_set_significance(Jpeg2000T1Context *t1,
                                  int x, int y, int negative);

extern uint8_t ff_jpeg2000_sigctxno_lut[256][4];

/* Get context label (number in range[0..8]) of a coefficient for significance
 * propagation and cleanup coding passes. */
static inline int ff_jpeg2000_getsigctxno(int flag, int bandno)
{
    return ff_jpeg2000_sigctxno_lut[flag & 255][bandno];
}

static const uint8_t refctxno_lut[2][2] = { { 14, 15 }, { 16, 16 } };

/* Get context label (number in range[14..16]) of a coefficient for magnitude
 * refinement pass. */
static inline int ff_jpeg2000_getrefctxno(int flag)
{
    return refctxno_lut[(flag >> 14) & 1][(flag & 255) != 0];
}

extern uint8_t ff_jpeg2000_sgnctxno_lut[16][16];
extern uint8_t ff_jpeg2000_xorbit_lut[16][16];

/* Get context label (number in range[9..13]) for sign decoding. */
static inline int ff_jpeg2000_getsgnctxno(int flag, int *xorbit)
{
    *xorbit = ff_jpeg2000_xorbit_lut[flag & 15][(flag >> 8) & 15];
    return ff_jpeg2000_sgnctxno_lut[flag & 15][(flag >> 8) & 15];
}

int ff_jpeg2000_init_component(Jpeg2000Component *comp,
                               Jpeg2000CodingStyle *codsty,
                               Jpeg2000QuantStyle *qntsty,
                               int cbps, int dx, int dy,
                               AVCodecContext *ctx, int max_slices);

void ff_jpeg2000_reinit(Jpeg2000Component *comp, Jpeg2000CodingStyle *codsty);

void ff_jpeg2000_cleanup(Jpeg2000Component *comp, Jpeg2000CodingStyle *codsty);

static inline int needs_termination(int style, int passno) {
    if (style & JPEG2000_CBLK_BYPASS) {
        int type = passno % 3;
        passno /= 3;
        if (type == 0 && passno > 2)
            return 2;
        if (type == 2 && passno > 2)
            return 1;
        if (style & JPEG2000_CBLK_TERMALL) {
            return passno > 2 ? 2 : 1;
        }
    }
    if (style & JPEG2000_CBLK_TERMALL)
        return 1;
    return 0;
}

void ff_tag_tree_zero(Jpeg2000TgtNode *t, int w, int h, int val);

#endif /* AVCODEC_JPEG2000_H */
