//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

#ifndef INCLUDED_IMF_HT_COMPRESSOR_H
#define INCLUDED_IMF_HT_COMPRESSOR_H

//-----------------------------------------------------------------------------
//
//	class HTCompressor -- uses High-Throughput JPEG 2000.
//
//-----------------------------------------------------------------------------

#include <vector>

#include "ImfNamespace.h"

#include "ImfCompressor.h"

#include <ojph_codestream.h>
#include <ojph_file.h>

OPENEXR_IMF_INTERNAL_NAMESPACE_HEADER_ENTER

class HTCompressor : public Compressor
{
public:
    HTCompressor (const Header& hdr, int numScanLines = 0);

    virtual ~HTCompressor ();

    HTCompressor (const HTCompressor& other) = delete;
    HTCompressor& operator= (const HTCompressor& other) = delete;
    HTCompressor (HTCompressor&& other)                 = delete;
    HTCompressor& operator= (HTCompressor&& other) = delete;

    virtual int numScanLines () const;

    virtual Format format () const;

    virtual int
    compress (const char* inPtr, int inSize, int minY, const char*& outPtr);

    virtual int
    uncompress (const char* inPtr, int inSize, int minY, const char*& outPtr);

private:

    ojph::codestream           _codestream;
    ojph::mem_outfile          _output;
    ojph::mem_infile           _input;
    int                        _num_comps;
    int16_t*                   _buffer;
    int                        _numScanLines;
    std::vector<int>           _cs_to_file_ch;    /* maps from codestream channel to file channel */
    std::vector<int>           _file_to_cs_ch;    /* maps from file channel to codestream channel */
    bool                       _isRGB;
};

OPENEXR_IMF_INTERNAL_NAMESPACE_HEADER_EXIT

#endif
