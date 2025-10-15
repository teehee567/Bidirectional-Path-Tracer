#pragma once
#include <png.h>
#include <vector>
#include <cstdio>
#include <stdexcept>
#include <filesystem>

namespace fs = std::filesystem;

inline double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

inline void colors_to_rgb8(const std::vector<color>& fb, int W, int H,
                           int samples_per_pixel, std::vector<uint8_t>& out_rgb)
{
    out_rgb.resize(size_t(W) * size_t(H) * 3);
    const double scale = 1.0 / std::max(1, samples_per_pixel);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            const color c = fb[j * W + i] * scale;

            // gamma 2.0 via sqrt, clamp to [0, 0.999] before scaling
            const double r = std::sqrt(clampd(c.x(), 0.0, 0.999));
            const double g = std::sqrt(clampd(c.y(), 0.0, 0.999));
            const double b = std::sqrt(clampd(c.z(), 0.0, 0.999));

            const int idx = (j * W + i) * 3;
            out_rgb[idx + 0] = static_cast<uint8_t>(256.0 * r);
            out_rgb[idx + 1] = static_cast<uint8_t>(256.0 * g);
            out_rgb[idx + 2] = static_cast<uint8_t>(256.0 * b);
        }
    }
}

// ---------- PNG writer (libpng) ----------
inline void write_png(const char* filename,
                      const std::vector<uint8_t>& rgb,
                      int W, int H)
{
    if (rgb.size() != static_cast<size_t>(W) * static_cast<size_t>(H) * 3)
        throw std::runtime_error("write_png: RGB buffer size mismatch");

    fs::path out_dir = "output";
    fs::create_directories(out_dir);

    // Combine directory + filename
    fs::path out_path = out_dir / filename;

    FILE* fp = nullptr;
#if defined(_MSC_VER)
    if (fopen_s(&fp, out_path.string().c_str(), "wb") != 0 || !fp)
        throw std::runtime_error("write_png: cannot open output file");
#else
    fp = std::fopen(out_path.string().c_str(), "wb");
    if (!fp) throw std::runtime_error("write_png: cannot open output file");
#endif

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) { std::fclose(fp); throw std::runtime_error("png_create_write_struct failed"); }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) { png_destroy_write_struct(&png_ptr, nullptr); std::fclose(fp); throw std::runtime_error("png_create_info_struct failed"); }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        std::fclose(fp);
        throw std::runtime_error("libpng write error");
    }

    png_init_io(png_ptr, fp);
    png_set_IHDR(png_ptr, info_ptr, W, H, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
    // Optional: declare sRGB intent
    // png_set_sRGB_gAMA_and_cHRM(png_ptr, info_ptr, PNG_sRGB_INTENT_PERCEPTUAL);

    png_write_info(png_ptr, info_ptr);

    std::vector<png_bytep> rows(H);
    for (int j = 0; j < H; ++j)
        rows[j] = (png_bytep)&rgb[j * W * 3];

    png_write_image(png_ptr, rows.data());
    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    std::fclose(fp);
}