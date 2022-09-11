#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
using namespace std;
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
Model *model = NULL;
const int width = 800;
const int height = 800;
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    bool jud = true;
    if (abs(x0 - x1) < abs(y0 - y1)) {
        jud = false;
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1) {
        swap(x0, x1);
        swap(y0, y1);
    }
    float dx = x1 - x0;
    float dy = y1 - y0;
    int derror2 = std::abs(dy) * 2;
    int error2 = 0;
    int y = y0;
    for (int x = x0; x <= x1; x++) {
        if (jud) {
            image.set(x, y, color);
        } else {
            image.set(y, x, color);
        }
        error2 += derror2;
        if (error2 > dx) {
            y += (y1 > y0 ? 1 : -1);
            error2 -= dx * 2;
        }
    }
}
Vec3f tran(int x0, int y0, int x1, int y1, int x2, int y2, int x, int y)
{
    Vec3f ans = Vec3f(x2 - x0, x1 - x0, x0 - x) ^ Vec3f(y2 - y0, y1 - y0, y0 - y);
    if (abs(ans.z) < 1)
        return Vec3f(-1, 1, 1);
    return Vec3f(1.f - (ans.x + ans.y) / ans.z, ans.y / ans.z, ans.x / ans.z);
}
void pixel_triangle(int x0, int y0, int x1, int y1, int x2, int y2, TGAImage &image, TGAColor color)
{
    Vec2i clamp(image.get_width() - 1, image.get_height() - 1);
    int x_min = max(0, min(x0, min(x1, x2)));
    int y_min = max(0, min(y0, min(y1, y2)));
    int y_max = min(clamp.y, max(y0, max(y1, y2)));
    int x_max = min(clamp.x, max(x0, max(x1, x2)));

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            Vec3f tmp = tran(x0, y0, x1, y1, x2, y2, x, y);
            if (tmp.x < 0 || tmp.y < 0 || tmp.z < 0)
                continue;
            image.set(x, y, color);
        }
    }
}
void fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, TGAImage &image, TGAColor color)
{
    //这里进行一个排序
    if (y0 > y1) {
        swap(x0, x1);
        swap(y0, y1);
    }
    if (y0 > y2) {
        swap(x0, x2);
        swap(y0, y2);
    }
    if (y1 > y2) {
        swap(x1, x2);
        swap(y1, y2);
    }
    //分两段拉横线
    int xl, xr;
    int dxl = abs(x1 - x0);
    int dxl2 = abs(x1 - x2);
    int dxr = abs(x2 - x0);
    float dy1 = y1 - y0 + 1;
    float dy2 = y2 - y1 + 1;
    float dy = y2 - y0;
    for (int y = y0; y < y1; y++) {
        float tmp1 = (y - y0) / dy1;
        float tmp2 = (y - y0) / dy;
        xl = (1 - tmp1) * x0 + tmp1 * x1;
        xr = (1 - tmp2) * x0 + tmp2 * x2;
        if (xl > xr)
            swap(xl, xr);
        for (int x = xl; x <= xr; x++)
            image.set(x, y, color);
    }
    for (int y = y1; y <= y2; y++) {
        float tmp1 = (y - y1) / dy2;
        float tmp2 = (y - y0) / dy;
        xl = (1 - tmp1) * x1 + tmp1 * x2;
        xr = (1 - tmp2) * x0 + tmp2 * x2;
        // line(xl, y, xr, y, image, color);
        if (xl > xr)
            swap(xl, xr);
        for (int x = xl; x <= xr; x++)
            image.set(x, y, color);
    }
}

int main(int argc, char **argv)
{
    if (2 == argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    Vec3f light_dir(0, 0, -1);
    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f v0 = model->vert(face[0]);
        Vec3f v1 = model->vert(face[1]);
        Vec3f v2 = model->vert(face[2]);
        int x0 = (v0.x + 1.) * width / 2.;
        int y0 = (v0.y + 1.) * height / 2.;
        int x1 = (v1.x + 1.) * width / 2.;
        int y1 = (v1.y + 1.) * height / 2.;
        int x2 = (v2.x + 1.) * width / 2.;
        int y2 = (v2.y + 1.) * height / 2.;
        // line(x0, y0, x1, y1, image, white);
        // line(x0, y0, x2, y2, image, white);
        // line(x2, y2, x1, y1, image, white);

        Vec3f n = (v2 - v0) ^ (v1 - v0);
        n = n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0)
            pixel_triangle(x0, y0, x1, y1, x2, y2, image,
                           TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
    }

    image.flip_vertically();  // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
