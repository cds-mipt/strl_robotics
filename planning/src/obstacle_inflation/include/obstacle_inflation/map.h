#pragma once
#include <vector>

class  Map
{
private:
    unsigned int _width, _height;
    bool **_grid{nullptr};
    double **_distances{nullptr};
    void dt(float *image);
    float *dt1(float *f, int n);
    bool b = true;
public:
    Map(/* args */);
    Map(unsigned int width, unsigned int height);
    Map (const Map&);
    ~ Map();
    unsigned int getWidth() const;
    unsigned int getHeight() const;
    bool cellIsObstacle(unsigned int x, unsigned int y) const;
    void setWidth(unsigned int newWidth);
    void setHeight(unsigned int newHeight);
    void setCell(unsigned int x, unsigned int y, bool value);
    void setDistance(unsigned int x, unsigned int y, double value);
    void computeDistances();
    double getDistance(int xi, int yi) const;
};
