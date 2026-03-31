#ifndef TEXTURE_H
#define TEXTURE_H

#include "rtweekend.h" // Access to vec3, color, shared_ptr
#include <iostream>

// =========================================================================
// STB Image Setup
// =========================================================================
// Only define STB_IMAGE_IMPLEMENTATION in ONE .cpp file (e.g., main.cpp).
// If you implement it here, ensure this header is included only once or
// move the #define to your main.cpp
#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif

// =========================================================================
// Abstract Texture Class
// =========================================================================
class texture {
public:
    virtual ~texture() = default;

    // Returns the color value at coordinates (u,v) or point p
    virtual color value(double u, double v, const point3& p) const = 0;
};

// =========================================================================
// Solid Color (Wrapper for existing single colors)
// =========================================================================
class solid_color : public texture {
public:
    solid_color(const color& c) : color_value(c) {}

    virtual color value(double u, double v, const point3& p) const override {
        (void)u; (void)v; (void)p;
        return color_value;
    }

private:
    color color_value;
};

// =========================================================================
// Checker Texture (Procedural)
// =========================================================================
class checker_texture : public texture {
public:
    // Checkers can be two colors, or even two other textures nested!
    checker_texture(double _scale, std::shared_ptr<texture> _even, std::shared_ptr<texture> _odd)
        : inv_scale(1.0 / _scale), even(_even), odd(_odd) {
    }

    checker_texture(double _scale, color c1, color c2)
        : inv_scale(1.0 / _scale),
        even(std::make_shared<solid_color>(c1)),
        odd(std::make_shared<solid_color>(c2)) {
    }

    virtual color value(double u, double v, const point3& p) const override {
        // Use spatial coordinates for 3D checker pattern
        int xInteger = static_cast<int>(std::floor(inv_scale * p.x()));
        int yInteger = static_cast<int>(std::floor(inv_scale * p.y()));
        int zInteger = static_cast<int>(std::floor(inv_scale * p.z()));

        bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

        return isEven ? even->value(u, v, p) : odd->value(u, v, p);
    }

private:
    double inv_scale;
    std::shared_ptr<texture> even;
    std::shared_ptr<texture> odd;
};

// =========================================================================
// Image Texture (Texture Mapping)
// =========================================================================
class image_texture : public texture {
public:
    const static int bytes_per_pixel = 3;

    image_texture(const char* filename) {
        auto components_per_pixel = bytes_per_pixel;

        // Load image with stb_image
        data = stbi_load(filename, &width, &height, &components_per_pixel, components_per_pixel);

        if (!data) {
            std::cerr << "ERROR: Could not load texture image file '" << filename << "'.\n";
            width = height = 0;
        }

        bytes_per_scanline = bytes_per_pixel * width;
    }

    ~image_texture() {
        STBI_FREE(data);
    }

    virtual color value(double u, double v, const point3& p) const override {
        (void)p; // Image textures strictly use UV coordinates

        // If we have no texture data, return a debug color (Magenta)
        if (data == nullptr) return color(1, 0, 1);

        // Clamp input texture coordinates to [0,1] x [1,0]
        u = clampd(u, 0.0, 1.0);
        v = 1.0 - clampd(v, 0.0, 1.0); // Flip V to image coordinates

        auto i = static_cast<int>(u * width);
        auto j = static_cast<int>(v * height);

        // Clamp integer mapping
        if (i >= width)  i = width - 1;
        if (j >= height) j = height - 1;

        const double color_scale = 1.0 / 255.0;
        auto pixel = data + j * bytes_per_scanline + i * bytes_per_pixel;

        // Return gamma corrected color (assuming input image is sRGB, we want Linear)
        double r = pixel[0] * color_scale;
        double g = pixel[1] * color_scale;
        double b = pixel[2] * color_scale;

        // Approximate sRGB to Linear conversion (Gamma 2.2)
        return color(std::pow(r, 2.2), std::pow(g, 2.2), std::pow(b, 2.2));
    }

private:
    unsigned char* data;
    int width, height;
    int bytes_per_scanline;
};

#endif