#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), tx_verts_(), faces_(), textures_()
{
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail())
        return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());  // iss是每行字符串
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i = 0; i < 3; i++)
                iss >> v.raw[i];
            verts_.push_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<int> f;
            std::vector<int> t;
            int itrash, idx, itx;
            iss >> trash;
            while (iss >> idx >> trash >> itx >> trash >> itrash) {
                idx--;  // in wavefront obj all indices start at 1, not zero
                f.push_back(idx);
                t.push_back(--itx);
            }

            faces_.push_back(f);
            textures_.push_back(t);
        } else if (!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vec3f v;
            for (int i = 0; i < 3; i++)
                iss >> v.raw[i];
            tx_verts_.push_back(v);
        }
    }

    std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << std::endl;
}

Model::~Model()
{
}

int Model::nverts()
{
    return (int)verts_.size();
}
int Model::ntxverts()
{
    return (int)tx_verts_.size();
}

int Model::nfaces()
{
    return (int)faces_.size();
}
int Model::ntextures()
{
    return (int)textures_.size();
}

std::vector<int> Model::face(int idx)
{
    return faces_[idx];
}
std::vector<int> Model::texture(int idx)
{
    return textures_[idx];
}

Vec3f Model::vert(int i)
{
    return verts_[i];
}
Vec3f Model::tx_vert(int i)
{
    return tx_verts_[i];
}
