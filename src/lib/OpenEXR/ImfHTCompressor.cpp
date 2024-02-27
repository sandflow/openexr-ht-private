//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//	class HTCompressor
//
//-----------------------------------------------------------------------------

#include "ImfHTCompressor.h"
#include "ImfAutoArray.h"
#include "ImfChannelList.h"
#include "ImfCheckedArithmetic.h"
#include "ImfHeader.h"
#include "ImfIO.h"
#include "ImfMisc.h"
#include "ImfNamespace.h"
#include "ImfXdr.h"
#include <assert.h>
#include <string.h>

#include <ImathBox.h>

#include <ojph_arch.h>
#include <ojph_file.h>
#include <ojph_params.h>
#include <ojph_mem.h>

using IMATH_NAMESPACE::Box2i;

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_ENTER

HTCompressor::HTCompressor (const Header& hdr, int numScanLines)
    : Compressor (hdr), _num_comps (0), _buffer (NULL), _numScanLines ()
{
    this->_numScanLines = numScanLines > 0 ? numScanLines : 16000;

    /* generate channel map */

    const ChannelList& channels = header ().channels ();

    int r_index = -1;
    int g_index = -1;
    int b_index = -1;

    for (ChannelList::ConstIterator c = channels.begin (); c != channels.end ();
         ++c)
    {
        assert (c.channel ().type == HALF);
        assert (c.channel ().xSampling == 1);
        assert (c.channel ().ySampling == 1);

        if (c.name () == "R") { r_index = this->_num_comps; }
        else if (c.name () == "G")
        {
            g_index = this->_num_comps;
        }
        else if (c.name () == "B")
        {
            b_index = this->_num_comps;
        }

        this->_num_comps++;
    }

    this->_cs_to_file_ch.resize (this->_num_comps);

    if (r_index > 0 && g_index > 0 && b_index > 0)
    {
        this->_isRGB = true;

        this->_cs_to_file_ch[0] = r_index;
        this->_cs_to_file_ch[1] = g_index;
        this->_cs_to_file_ch[2] = b_index;

        int file_i = 0;
        int cs_i   = 3;
        for (ChannelList::ConstIterator c = channels.begin ();
             c != channels.end ();
             ++c)
        {
            if (c.name () != "R" && c.name () != "G" && c.name () != "B")
            { this->_cs_to_file_ch[cs_i++] = file_i; }

            file_i++;
        }
    }
    else
    {
        this->_isRGB = false;

        for (int i = 0; i < this->_num_comps; i++)
        {
            this->_cs_to_file_ch[i] = i;
        }
    }

    this->_file_to_cs_ch.resize (this->_num_comps);

    for (int i = 0; i < this->_num_comps; i++)
    {
        this->_file_to_cs_ch[this->_cs_to_file_ch[i]] = i;
    }
}

HTCompressor::~HTCompressor ()
{
    this->_output.close ();
    this->_input.close ();
    delete[] this->_buffer;
}

int
HTCompressor::numScanLines () const
{
    return this->_numScanLines;
}

Compressor::Format
HTCompressor::format () const
{
    return Compressor::Format::NATIVE;
}

int
HTCompressor::compress (
    const char* inPtr, int inSize, int minY, const char*& outPtr)
{
    Box2i      dw     = this->header ().dataWindow ();
    ojph::ui32 height = std::min (dw.size ().y + 1 - minY, this->_numScanLines);
    ojph::ui32 width  = dw.size ().x + 1;

    this->_codestream.set_planar (false);

    ojph::param_siz siz = this->_codestream.access_siz ();

    siz.set_num_components (this->_num_comps);
    for (ojph::ui32 c = 0; c < this->_num_comps; c++)
        siz.set_component (c, ojph::point (1, 1), 16, true);

    /* TODO: extend to multiple tiles and non-coincident data and display windows */
    siz.set_image_offset (ojph::point (0, 0));
    siz.set_tile_offset (ojph::point (0, 0));
    siz.set_image_extent (ojph::point (width, height));
    siz.set_tile_size (ojph::size (width, height));

    ojph::param_cod cod = this->_codestream.access_cod ();

    cod.set_color_transform (this->_isRGB);
    cod.set_reversible (true);
    cod.set_block_dims (128, 32);
    cod.set_num_decomposition (5);

    this->_output.open ();

    this->_codestream.write_headers (&this->_output);

    assert (
        inSize == (this->_num_comps * pixelTypeSize (HALF) * height * width));

    const int16_t*  pixels      = (const int16_t*) inPtr;
    const int16_t*  line_pixels = pixels;
    ojph::ui32      next_comp   = 0;
    ojph::line_buf* cur_line    = this->_codestream.exchange (NULL, next_comp);

    for (ojph::ui32 i = 0; i < height; ++i)
    {
        const char* in_buf;

        for (ojph::ui32 c = 0; c < this->_num_comps; c++)
        {
            assert (next_comp == c);

            const int16_t* channel_pixels =
                line_pixels + width * this->_cs_to_file_ch[c];

            for (uint32_t p = 0; p < width; p++)
            {
                ojph::si32 c = static_cast<const ojph::si32> (*channel_pixels++);

                cur_line->i32[p] = c < 0 ? -32769 - c : c;
            }

            cur_line = this->_codestream.exchange (cur_line, next_comp);
        }

        line_pixels += width * this->_num_comps;
    }

    this->_codestream.flush ();

    assert (this->_output.tell () >= 0);

    outPtr = reinterpret_cast<const char*> (this->_output.get_data ());

    return static_cast<int> (this->_output.tell ());
}

int
HTCompressor::uncompress (
    const char* inPtr, int inSize, int minY, const char*& outPtr)
{
    this->_input.open (reinterpret_cast<const ojph::ui8*> (inPtr), inSize);
    this->_codestream.read_headers (&this->_input);

    ojph::param_siz siz = this->_codestream.access_siz ();
    ojph::ui32 width    = siz.get_image_extent ().x - siz.get_image_offset ().x;
    ojph::ui32 height   = siz.get_image_extent ().y - siz.get_image_offset ().y;

    assert (this->_num_comps == siz.get_num_components ());

    this->_codestream.set_planar (false);

    this->_codestream.create ();

    assert (sizeof (int16_t) == pixelTypeSize (HALF));

    this->_buffer = new int16_t[this->_num_comps * width * height];

    int16_t* line_pixels = this->_buffer;

    for (uint32_t i = 0; i < height; ++i)
    {

        for (uint32_t c = 0; c < this->_num_comps; c++)
        {
            ojph::ui32      next_comp = 0;
            ojph::line_buf* cur_line  = this->_codestream.pull (next_comp);

            assert (next_comp == c);

            int16_t* channel_pixels = line_pixels + width * this->_cs_to_file_ch[c];

            for (uint32_t p = 0; p < width; p++)
            {
                ojph::si32 c = cur_line->i32[p];

                *channel_pixels++ = (int16_t) (c < 0 ? -32769 - c : c);
            }
        }

        line_pixels += width * this->_num_comps;
    }

    outPtr = (const char*) this->_buffer;

    return this->_num_comps * pixelTypeSize (HALF) * width * height;
}

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_EXIT
