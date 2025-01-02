#define _CRT_SECURE_NO_WARNINGS

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include <stdio.h> 
#include <time.h>

#include "geometry.h"
#include "model.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION 
#include "stb-master/stb_image_write.h"

#define BACKGROUND_COLOR Vec3f(0.2, 0.7, 0.8)
#define DEPTH 4

struct Light {
    
    Vec3f position;
    float intensity;

    Light(const Vec3f& p, const float& i) : position(p), intensity(i) {}
};


struct Material {
    
    float specular_exponent, refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
    Material(const float& r, const Vec4f& a, const Vec3f& color, const float& spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
};


struct ObjModel : Model {
    
    Material material;
    
    ObjModel(const char* filename, const Vec3f&  shift, const Material& m) : Model(filename, shift), material(m) {}
};


void rotation(Vec3f& v, const Vec3f& angles) {

    float alpha = angles[0] * M_PI / 180.; // перевод градусов в радианы
    float beta = angles[1] * M_PI / 180.;
    float gamma = angles[2] * M_PI / 180.;

    float rotated_x = v.x * (cos(beta) * cos(gamma)) + v.y * (-sin(gamma) * cos(beta)) + v.z * sin(beta);
    float rotated_y = v.x * (sin(alpha) * sin(beta) * cos(gamma) + sin(gamma) * cos(alpha)) + v.y * (-sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma)) +
        v.z * (-sin(alpha) * cos(beta));
    float rotated_z = v.x * (sin(alpha) * sin(gamma) - sin(beta) * cos(alpha) * cos(gamma)) + v.y * (sin(alpha) * cos(gamma) + sin(beta) * sin(gamma) * cos(alpha)) 
        + v.z * (cos(alpha) * cos(beta));

    v.x = rotated_x;
    v.y = rotated_y;
    v.z = rotated_z;
}


Vec3f reflect(const Vec3f& I, const Vec3f& N) {
    return I - N * 2.f * (I * N);
}


Vec3f refract(const Vec3f& I, const Vec3f& N, const float& refractive_index) { // По закону Снеллиуса

    float cosi = -std::max(-1.f, std::min(1.f, I * N));
    float etai = 1, etat = refractive_index;
    Vec3f n = N;

    if (cosi < 0) {
        cosi = -cosi;
        std::swap(etai, etat); n = -N;
    }

    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrtf(k));
}



bool scene_intersect(const Vec3f& orig, const Vec3f& dir, const ObjModel& model, Vec3f& hit, Vec3f& N, Material& material) {
        
    float object_dist = std::numeric_limits<float>::max();
    float dist_i;
    for (size_t i = 0; i < model.nfaces(); i++) {

        if (model.ray_triangle_intersect(i, orig, dir, dist_i) && dist_i <= object_dist) {

            object_dist = dist_i;
            hit = orig + dir * object_dist;
            material = model.material;

            Vec3f edge1 = model.point(model.vert(i, 1)) - model.point(model.vert(i, 0));
            Vec3f edge2 = model.point(model.vert(i, 2)) - model.point(model.vert(i, 0));
            N = (cross(edge1, edge2)).normalize();
        }
    }

    if (fabs(dir.y) > 1e-3) {

        float d = -(orig.y + 4) / dir.y; // orig.y + t*dir.y = -4 => t = (-orig.y - 4) / dir.y
        Vec3f pt = orig + dir * d;

        if (d > 0 && fabs(pt.x) < 10 && pt.z<-10 && pt.z>-30 && d < object_dist) {

            object_dist = d;
            hit = pt;
            N = Vec3f(0, 1, 0);
            material.diffuse_color = Vec3f(1, .7, .3) * .3;
        }
    }

    return object_dist < 10000;
}



Vec3f cast_ray(const Vec3f& orig, const Vec3f& dir, const ObjModel& model, const std::vector<Light>& lights, size_t depth = 0) {

    Vec3f point, N;
    Material material;

    if (depth > DEPTH || !scene_intersect(orig, dir, model, point, N, material)) {
        return BACKGROUND_COLOR;
    }

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, model, lights, depth + 1);

    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, model, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i = 0; i < lights.size(); i++) {

        Vec3f light_dir = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();
        Vec3f shadow_orig = light_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;

        if (scene_intersect(shadow_orig, light_dir, model, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt - shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N) * dir), material.specular_exponent) * lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1] 
        + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}



void render(const ObjModel& model, const std::vector<Light>& lights, const Vec3f& pos, const Vec3f& rot, const char* filename) {

    const int width = 1024;
    const int height = 768;
    const int fov = M_PI / 2.;
    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++) {
        for (size_t i = 0; i < width; i++) {
            float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height;
            float y = -(2 * (j + 0.5) / (float)height - 1) * tan(fov / 2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            rotation(dir, rot);
            framebuffer[i + j * width] = cast_ray(pos, dir, model, lights);
            std::cerr << (i + j * width) << " / " << (height * width) << "  Pixel Completed\n";
        }
    }

    std::vector<unsigned char> pixmap(width * height * 3);
    for (size_t i = 0; i < height * width; ++i) {
        Vec3f& color = framebuffer[i];
        float max = std::max(color[0], std::max(color[1], color[2]));
        if (max > 1) color = color * (1. / max);
        for (size_t j = 0; j < 3; j++) {
            pixmap[i * 3 + j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    stbi_write_jpg(filename, width, height, 3, pixmap.data(), 100);
}



int main() {

    Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
    ObjModel model("duck.obj", Vec3f(-5., .13437, -10.), red_rubber);

    std::vector<Light> lights;
    lights.push_back(Light(Vec3f(30, 50, -25), 1.5));
    lights.push_back(Light(Vec3f(30, 51, -25), 1.8));
    lights.push_back(Light(Vec3f(31, 50, -25), 1.7));

    clock_t start = clock();

    render(model, lights, Vec3f(0., 0., -8.), Vec3f(0., 4., 0.), "Duck/Result_0_0_0.png");
    render(model, lights, Vec3f(10., 0., -10.), Vec3f(0., 45., 0.), "Duck/Result_0_45_0.png");
    render(model, lights, Vec3f(10., 0., -20.), Vec3f(0., 90., 0.), "Duck/Result_0_90_0.png");
    render(model, lights, Vec3f(10., 0., -30.), Vec3f(0., 115., 0.), "Duck/Result_0_115_0.png");

    render(model, lights, Vec3f(0., 0., -34.), Vec3f(0., 180., 0.), "Duck/Result_0_180_0.png");
    render(model, lights, Vec3f(-12., 0., -32.), Vec3f(0., -135., 0.), "Duck/Result_0_225_0.png");
    render(model, lights, Vec3f(-12., 0., -20.), Vec3f(0., -75., 0.), "Duck/Result_0_285_0.png");
    render(model, lights, Vec3f(-10., 0., -10.), Vec3f(0., -45., 0.), "Duck/Result_0_315_0.png");

    clock_t end = clock();
    double seconds = (double)(end - start) / CLOCKS_PER_SEC;
    printf("\nThe time: %f seconds\n", seconds);

    return 0;
}
