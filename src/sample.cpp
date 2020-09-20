#include "include/TMesh2D.h"
#include "include/TMeshRender2D.hpp"
#include "include/frep2D.h"

using namespace std::placeholders;

int main(int argc, char *argv[])
{
    int resX{512}, resY{512};
    glm::vec2 center(resX/2.0f, resY/2.0f);
    frep::FRepObj2D frep0(resX, resY);
    auto clef = std::bind(&frep::FRepObj2D::trebleClef, &frep0, _1, _2, _3);
    std::vector<float> clefFrep = frep0.getFRep2D(center, 1.0, clef);

    tmesh2D::TMeshBoundingBox bbox(glm::vec2(0, 0), glm::vec2(resX, resY));
    tmesh2D::TMesh tmesh(bbox, resX, resY, 40, 10, 3, 5);
    tmesh.generateTMesh(&clefFrep, glm::vec2(0, 0.001));
    std::vector<tmesh2D::TMesh::tNode*> leaves = tmesh.getLeaves();

    tmesh2D::TMeshRender<tmesh2D::TMesh, tmesh2D::TMesh::tNode> render;
    render.createWindow(resX, resY, "tmesh2D");
    render.displayTree(leaves, "tmesh.png");

    return 0;
}
