#ifndef H_FREP_CLASS
#define H_FREP_CLASS

#include <glm/glm.hpp>
#include <cmath>
#include <vector>
#include <functional>
#include <algorithm>

namespace frep {

class FRepObj2D
{
public:
    FRepObj2D(int resX, int resY);

    //2D primitves
    float triangle(glm::vec2 pos, glm::vec2 cent, float a, float b, float c, const float scale = 1.0f);
    float triangle2(glm::vec2 pos, glm::vec2 cent, float a, float b, const float scale = 1.0f);

    float ellipticCylZ2D(glm::vec2 pos, glm::vec2 cent, float R, float a, float b, const float scale = 1.0f);
    float ellipsoid2D(glm::vec2 pos, glm::vec2 cent, float R, float a, float b, const float scale = 1.0f);
    float torusY2D(glm::vec2 pos, glm::vec2 cent, float R , float rev, const float scale = 1.0f);
    float torusZ2D(glm::vec2 pos, glm::vec2 cent, float R, float rev, const float scale = 1.0f);
    float rectangle(glm::vec2 pos, glm::vec2 cent, float w, float h, const float scale = 1.0f);

    float circle(glm::vec2 pos, glm::vec2 cent, float R, const float scale = 1.0f);
    float blobby2D(glm::vec2 pos, glm::vec2 cent, float R, const float scale = 1.0f);

    float heart2D(glm::vec2 pos, glm::vec2 cent, const float scale = 1.0f);
    float decocube2D(glm::vec2 pos, glm::vec2 cent,const float scale = 1.0f);
    float suriken(glm::vec2 pos, glm::vec2 cent, const float scale = 1.0f);
    float elf(glm::vec2 pos, glm::vec2 center, const float scale = 1.0f);
    float bat(glm::vec2 pos, glm::vec2 center, const float scale = 1.0f);
    float trebleClef(glm::vec2 pos, glm::vec2 center, const float scale = 1.0f);

    std::vector<float> getFRep2D(std::function<float(glm::vec2)> fun);
    std::vector<float> getFRep2D(glm::vec2 cent, std::function<float(glm::vec2, glm::vec2)> fun);
    std::vector<float> getFRep2D(glm::vec2 cent, float R, std::function<float(glm::vec2, glm::vec2, float)> fun);
    std::vector<float> getFRep2D(glm::vec2 cent, float p1, float p2,
                                 std::function<float(glm::vec2, glm::vec2, float, float)> fun);
    std::vector<float> getFRep2D(glm::vec2 cent, float p1, float p2, float p3,
                                 std::function<float(glm::vec2, glm::vec2, float, float, float)> fun);
    std::vector<float> getFRep2D(std::vector<float> f1, std::vector<float> f2, float alpha, float m,
                                  std::function<float(float, float, float, float)> fun);
    std::vector<float> getFRep2D(std::vector<float> f1, std::vector<float> f2, float n,
                                  std::function<float(float, float, float)> fun);

    std::vector<float> getRotatedFrep2D(glm::vec2 cent, float w, float h,
                                         float angle, std::function<float (glm::vec2, glm::vec2, float, float)>);
    std::vector<float> getRotatedFrep2D(glm::vec2 cent, float a, float b, float c,
                                         float angle, std::function<float(glm::vec2, glm::vec2, float, float, float, float)>);
    ~FRepObj2D() = default;

    //operations over 2D primitives
public:
    float bounded_blending(float f1, float f2, float a0, float a1, float a2, float a3 , float time, float alpha, float m);
    float constOffset(float f, float offset);
    float constRadiusOffset(float f, float fOffset, float R, float x0, float y0);
    float union_function_R0(float f1, float f2, float n);

    glm::vec2 findZeroLevelSetInterval(std::vector<float> field , int numElemToAverage = 15);

    std::vector<float> scaleFunction(const std::vector<float> field, const float factor);

    float intersect_function(float f1, float f2, float alpha = 0, float m = 0);
    float union_function(float f1, float f2, float alpha = 0, float m = 0);
    float subtract_function(float f1, float f2, float alpha = 0, float m = 0);

    inline void setNewRes(int resX, int resY)  { m_resX = resX; m_resY = resY; }
    inline float convertToUV(float val) { return val / m_resX; }
    inline glm::vec2 convertToUV(glm::vec2 val) { return glm::vec2(val.x/m_resX, val.y/m_resY); }
    inline float scaleToNewRange(float v, float newMin=0, float oldMin = 0, float oldMax = 512)
                                { return ((v - oldMin)*(m_resX-newMin)/(oldMax - oldMin))+newMin; }
private:
    glm::vec2 getRotatedCoords(glm::vec2 inCoords , const float angle);

    int m_resX, m_resY;
    std::vector<float> m_frep;
};

} //namespace frep
#endif
