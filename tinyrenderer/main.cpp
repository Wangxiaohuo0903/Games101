#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include <iostream>
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
    return Vec3f(1.f - (ans.x + ans.y) / ans.z, float(ans.y )/ ans.z, float(ans.x) / ans.z);
}
void pixel_triangle(Vec3f *pt, Vec2f *px, TGAImage &image, float *z_buffer, TGAImage &texture,float intensity)
{
    Vec2i clamp(image.get_width() - 1, image.get_height() - 1);
    int x_min = min(pt[0].x, min(pt[1].x, pt[2].x));
    int y_min = min(pt[0].y, min(pt[1].y, pt[2].y));
    int y_max = max(pt[0].y, max(pt[1].y, pt[2].y));
    int x_max = max(pt[0].x, max(pt[1].x, pt[2].x));

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            Vec3f tmp = tran(pt[0].x, pt[0].y, pt[1].x, pt[1].y, pt[2].x, pt[2].y, x, y);
            int z = 0;

            if (tmp.x < 0 || tmp.y < 0 || tmp.z < 0)
                continue;
            z = tmp.x * pt[0].z + tmp.y * pt[1].z + tmp.z * pt[2].z;
            if (z > z_buffer[int(x + y * width)]) {
                z_buffer[int(x + y * width)] = z;
                int tx = tmp.x * px[0].x + tmp.y * px[1].x + tmp.z * px[2].x;
                int ty = tmp.x * px[0].y + tmp.y * px[1].y + tmp.z * px[2].y;
                TGAColor color= texture.get(tx,ty);
                image.set(x, y, TGAColor(color.r*intensity, color.g*intensity, color.b*intensity,255));
                //image.set(x, y, TGAColor(255*intensity, 255*intensity, 255*intensity,255));
               // cout<<px[0].x<<" "<<px[0].y<<endl;
            }
        }
    }
}
void fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, TGAImage &image, float intensity)
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
            image.set(x, y, TGAColor(255*intensity,255*intensity,255*intensity,255));
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
            image.set(x, y, TGAColor(255*intensity,255*intensity,255*intensity,255));
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
    TGAImage texture;
    texture.read_tga_file("obj/african_head_diffuse.tga");
    texture.flip_vertically();
    float *z_buffer = new float[width * height];
    for (int i = width * height; i--; z_buffer[i] = -std::numeric_limits<float>::max())
        ;
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        std::vector<int> text = model->texture(i);
        Vec3f v0 = model->vert(face[0]);
        Vec3f v1 = model->vert(face[1]);
        Vec3f v2 = model->vert(face[2]);
        Vec3f t0 = model->tx_vert(text[0]);
        Vec3f t1 = model->tx_vert(text[1]);
        Vec3f t2 = model->tx_vert(text[2]);
        
        int tx0 = t0.x * (texture.get_width());
        int ty0 = t0.y * (texture.get_height());
        int tx1 = t1.x * (texture.get_width());
        int ty1 = t1.y * (texture.get_height());
        int tx2 = t2.x * (texture.get_width());
        int ty2 = t2.y * (texture.get_height());
       //cout<<tx0<<" "<<tx1<<" "<<ty2<<" "<<endl;
        int x0 = (v0.x + 1.) * width / 2.;
        int y0 = (v0.y + 1.) * height / 2.;
        int x1 = (v1.x + 1.) * width / 2.;
        int y1 = (v1.y + 1.) * height / 2.;
        int x2 = (v2.x + 1.) * width / 2.;
        int y2 = (v2.y + 1.) * height / 2.;
        // line(x0, y0, x1, y1, image, white);
        // line(x0, y0, x2, y2, image, white);
        // line(x2, y2, x1, y1, image, white);
        int z0 = v0.z;
        int z1 = v1.z;
        int z2 = v2.z;

        Vec3f Pt[3] = {Vec3f(x0, y0, z0), Vec3f(x1, y1, z1), Vec3f(x2, y2, z2)};
        Vec2f Px[3] = {Vec2f(tx0, ty0), Vec2f(tx1, ty1), Vec2f(tx2, ty2)};
        Vec3f n = (v2 - v0) ^ (v1 - v0);
        n = n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0)
            //fill_triangle(x0,y0,x1,y1,x2,y2,image,intensity);
            pixel_triangle(Pt, Px, image, z_buffer, texture,intensity);
    }

    image.flip_vertically();  // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
