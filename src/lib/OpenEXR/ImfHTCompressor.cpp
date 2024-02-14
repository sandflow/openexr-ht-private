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

HTCompressor::HTCompressor (const Header& hdr)
    : Compressor (hdr), _num_comps (0), _buffer (NULL)
{
    const ChannelList& channels = header ().channels ();

    for (ChannelList::ConstIterator c = channels.begin (); c != channels.end ();
         ++c)
    {
        assert (c.channel ().type == HALF);
        assert (c.channel ().xSampling == 1);
        assert (c.channel ().ySampling == 1);
        this->_num_comps++;
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
    return 16000;
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
    ojph::ui32 height = dw.size ().y + 1;
    ojph::ui32 width  = dw.size ().x + 1;

    this->_codestream.set_planar (false);

    ojph::param_siz siz = this->_codestream.access_siz ();

    siz.set_num_components (this->_num_comps);
    for (ojph::ui32 c = 0; c < this->_num_comps; c++)
        siz.set_component (c, ojph::point (1, 1), 16, false);

    /* TODO: extend to multiple tiles and non-coincident data and display windows */
    siz.set_image_offset (ojph::point (0, 0));
    siz.set_tile_offset (ojph::point (0, 0));
    siz.set_image_extent (ojph::point (width, height));
    siz.set_tile_size (ojph::size (width, height));

    ojph::param_cod cod = this->_codestream.access_cod ();

    cod.set_color_transform (this->_num_comps == 3 || this->_num_comps == 4);
    cod.set_reversible (true);

    this->_output.open ();

    this->_codestream.write_headers (&this->_output);

    const char*      line       = inPtr;
    const ojph::ui32 pixel_size = this->_num_comps * pixelTypeSize (HALF);
    ojph::ui32       next_comp  = 0;
    ojph::line_buf*  cur_line   = this->_codestream.exchange (NULL, next_comp);

    assert (inSize == (pixel_size * height * width));

    for (ojph::ui32 i = 0; i < height; ++i)
    {
        const char* in_buf;

        for (ojph::ui32 c = 0; c < this->_num_comps; c++)
        {
            assert (next_comp == c);

            in_buf = line + pixelTypeSize (HALF) * c;

            for (uint32_t p = 0; p < width; p++)
            {
                cur_line->i32[p] =
                    *reinterpret_cast<const ojph::si32*> (in_buf);
                in_buf += pixel_size;
            }

            cur_line = this->_codestream.exchange (cur_line, next_comp);
        }

        line = in_buf;
    }

    this->_codestream.flush ();

    assert (this->_output.tell () >= 0);

    outPtr = reinterpret_cast<const char*> (this->_output.get_data ());

    return static_cast<int> (this->_output.tell ());
}

int
HTCompressor::compressTile (
    const char*            inPtr,
    int                    inSize,
    IMATH_NAMESPACE::Box2i range,
    const char*&           outPtr)
{
    assert (0);
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

    const ojph::ui32 pixel_size = this->_num_comps * pixelTypeSize (HALF);

    this->_buffer = new char[pixel_size * width * height];

    char* line_buf = this->_buffer;

    for (uint32_t i = 0; i < height; ++i)
    {
        char* pixel_buf;

        for (uint32_t c = 0; c < this->_num_comps; c++)
        {
            ojph::ui32      next_comp = 0;
            ojph::line_buf* cur_line  = this->_codestream.pull (next_comp);
            assert (next_comp == c);

            char*       pixel_buf = line_buf + c * pixel_size;
            ojph::si32* in        = cur_line->i32;

            for (uint32_t p = 0; p < width; p++)
            {
                *reinterpret_cast<ojph::si32*> (pixel_buf) = in[p];
                pixel_buf += pixel_size;
            }
        }

        line_buf = pixel_buf;
    }

    outPtr = this->_buffer;

    return pixel_size * width * height;
}

int
HTCompressor::uncompressTile (
    const char*            inPtr,
    int                    inSize,
    IMATH_NAMESPACE::Box2i range,
    const char*&           outPtr)
{
    assert (0);
}

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_EXIT
