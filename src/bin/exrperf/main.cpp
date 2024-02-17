#include <string>
#include <map>
#include <chrono>

#include "ImfArray.h"
#include "ImfCompression.h"
#include "ImfHeader.h"
#include "ImfRgbaFile.h"
#include "ImfFrameBuffer.h"
#include <ImfNamespace.h>
#include <OpenEXRConfig.h>

#include "cxxopts.hpp"

namespace IMF = OPENEXR_IMF_NAMESPACE;
using namespace OPENEXR_IMF_NAMESPACE;
using namespace IMATH_NAMESPACE;

static std::map<std::string, Compression> comp_table = {
    {{"NO_COMPRESSION", NO_COMPRESSION},
     {"RLE_COMPRESSION", RLE_COMPRESSION},
     {"ZIPS_COMPRESSION", ZIPS_COMPRESSION},
     {"ZIP_COMPRESSION", ZIP_COMPRESSION},
     {"PIZ_COMPRESSION", PIZ_COMPRESSION},
     {"PXR24_COMPRESSION", PXR24_COMPRESSION},
     {"B44_COMPRESSION", B44_COMPRESSION},
     {"B44A_COMPRESSION", B44A_COMPRESSION},
     {"DWAA_COMPRESSION", DWAA_COMPRESSION},
     {"DWAB_COMPRESSION", DWAA_COMPRESSION},
     {"HT_COMPRESSION", HT_COMPRESSION}}};

class OMemStream : public OStream
{
public:
    OMemStream (std::stringstream* ss) : OStream ("<omemfile>"), _buffer (ss)
    {
        this->_buffer->seekp (0);
    }

    virtual ~OMemStream () {}

    virtual void write (const char c[/*n*/], int n)
    {
        this->_buffer->write (c, n);
    }

    virtual uint64_t tellp () { return this->_buffer->tellp (); }

    virtual void seekp (uint64_t pos) { this->_buffer->seekp (pos); }

private:
    std::stringstream* _buffer;
};

class IMemStream : public IStream
{
public:
    IMemStream (std::stringstream* ss) : IStream ("<imemfile>"), _buffer (ss)
    {
        this->_buffer->exceptions (
            std::stringstream::failbit | std::stringstream::eofbit);
        this->_buffer->seekp (std::ios::end);
        this->_size = this->_buffer->tellp ();
        this->_buffer->seekg (0);
    }

    virtual ~IMemStream () {}

    virtual bool read (char c[/*n*/], int n)
    {
        this->_buffer->read (c, n);

        return this->_buffer->tellg () != this->_size;
    }

    virtual uint64_t tellg () { return this->_buffer->tellg (); }

    virtual void seekg (uint64_t pos) { this->_buffer->seekg (pos); }

    virtual void clear () { this->_buffer->clear (); }

private:
    std::stringstream*          _buffer;
    std::stringstream::pos_type _size;
};

int
main (int argc, char* argv[])
{
    cxxopts::Options options (
        "exrperf", "OpenEXR compress/uncompress benchmarks");

    options.add_options () (
        "r,repetitions",
        "Repetition count",
        cxxopts::value<int> ()->default_value ("5")) (
        "file", "Input image", cxxopts::value<std::string> ()) (
        "compression", "Compression", cxxopts::value<std::string> ());

    options.parse_positional ({"file", "compression"});

    options.show_positional_help ();

    auto args = options.parse (argc, argv);

    if (args.count ("compression") != 1 || args.count ("file") != 1)
    {
        std::cout << options.help () << std::endl;
        exit (-1);
    }

    Compression c = comp_table[args["compression"].as<std::string> ()];

    auto& src_fn = args["file"].as<std::string> ();

    /* load src image */
    RgbaInputFile src_file (src_fn.c_str ());

    Box2i dw     = src_file.dataWindow ();
    int   width  = dw.max.x - dw.min.x + 1;
    int   height = dw.max.y - dw.min.y + 1;

    Array2D<Rgba> pixels (height, width);

    src_file.setFrameBuffer (&pixels[-dw.min.x][-dw.min.y], 1, width);
    src_file.readPixels (dw.min.y, dw.max.y);

    Header src_header         = src_file.header ();
    src_header.compression () = c;

    /* mem buffer */

    std::stringstream mem_file;

    /* encode performance */

    OMemStream o_memfile (&mem_file);

    RgbaOutputFile o_file (o_memfile, src_header, src_file.channels ());
    o_file.setFrameBuffer (&pixels[-dw.min.x][-dw.min.y], 1, width);

    auto start = std::chrono::high_resolution_clock::now ();
    o_file.writePixels (height);
    auto dur = std::chrono::high_resolution_clock::now () - start;

    std::cout << "Encode time: " << std::chrono::duration<double> (dur).count ()
              << std::endl;
    std::cout << "Encoded size: " << mem_file.tellp () << std::endl;

    /* decode performance */

    IMemStream i_memfile (&mem_file);

    RgbaInputFile i_file (i_memfile);
    i_file.setFrameBuffer (&pixels[-dw.min.x][-dw.min.y], 1, width);

    start = std::chrono::high_resolution_clock::now ();
    i_file.readPixels (dw.min.y, dw.max.y);
    dur = std::chrono::high_resolution_clock::now () - start;

    std::cout << "Decode time: " << std::chrono::duration<double> (dur).count ()
              << std::endl;

    return 0;
}