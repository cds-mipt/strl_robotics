#include "map.h"
#include <math.h>
#include <iostream>
#include <fstream>

#define INF 1E20

Map::Map()
{
    _width = 0;
    _height = 0;
}

Map::~ Map()
{
}

Map::Map(unsigned int width, unsigned int height)
{
    this->_width = width;
    this->_height = height;
    _grid = new bool*[_width];
    _distances = new double*[_width];
    for (unsigned int i = 0; i < _height; ++i)
    {
        _grid[i] = new bool[_height];
        _distances[i] = new double[_height];
    }
}
Map::Map (const Map& map)
{
	this->_height = map.getHeight();
	this->_width = map.getWidth();
}

void Map::setWidth(unsigned int width)
{
    _width = width;
	if(_grid == nullptr)
		_grid = new bool*[_width];
	
	if(_distances == nullptr)
		_distances = new double*[_width];
}

void Map::setHeight(unsigned int height)
{
    _height = height;
	for (unsigned int i = 0; i < _width; ++i)
  {
      _grid[i] = new bool[_height];
      _distances[i] = new double[_height];
  }
}

void Map::setCell(unsigned int x, unsigned int y, bool value)
{
    _grid[x][y] = value;
}

void Map::setDistance(unsigned int x, unsigned int y, double value)
{
	_distances[x][y] = value;
}

bool Map::cellIsObstacle(unsigned int x, unsigned int y) const
{
    return _grid[x][y];
}

unsigned int Map::getWidth() const
{
   return _width;
}

unsigned int Map::getHeight() const
{
    return _height;
}


/* dt of 1d function using euclidean distance */
float * Map::dt1(float *f, int n)
{
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;
  for (int q = 1; q <= n-1; q++)
  {
    float s  = ((f[q] + q * q)-(f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    while (s <= z[k]) {
      k--;
      s  = ((f[q] + q * q)-(f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    d[q] = (q-v[k]) * (q-v[k]) + f[v[k]];
  }

  delete [] v;
  delete [] z;
  return d;
}

/* dt of 2d function using euclidean distance */
void Map::dt(float *image)
{
  
//  float *f = new float[std::max(_width, _height)];
  float *f = new float[_height];
  // transform along columns
  for (int x = 0; x < _width; x++)
  {
    for (int y = 0; y < _height; y++)
	{
      f[y] = image[x + y * _width];
    }

    float *d = dt1(f, _height);
    
	for (int y = 0; y < _height; y++) {
      image[x + y * _width] = d[y];
    }
    delete [] d;
  }

  delete f;
  f = new float[_width];
  // transform along rows


  for (int y = 0; y < _height; y++)
  {
    for (int x = 0; x < _width; x++)
	{
      f[x] = image[x + y * _width];
    }
    float *d = dt1(f, _width);
    for (int x = 0; x < _width; x++)
	{
      image[x + y * _width] = d[x];
    }
    delete [] d;
  }

  delete f;
}

void Map::computeDistances()
{
	/* dt of binary image using squared distance */
	float *image = new float[_width * _height];
	for (int y = 0; y < _height; y++)
	{
		for (int x = 0; x < _width; x++)
		{
			if (_grid[x][y])
				image[x + y * _width] = 0;
			else
				image[x + y * _width] = INF;
		}
	}

	dt(image);

	//std::ofstream output;
	//output.open("/home/vlad/lab_ws/src/obstacle_inflation/output/test.txt");

	for (int y = 0 ; y < _height; ++y)
	{
		for (int x = 0 ; x < _width; ++x)
		{
			//_distances[x][_height - y - 1] = sqrt(image[x + y * _width]);
            _distances[x][y] = sqrt(image[x + y * _width]);

			//output << sqrt(_distances[x][y]) << " ";
			//std::cout << sqrt(_distances[i][j]) << " ";
		}
		//std::cout << "\n";
		//output << "\n";
	}

	//output.close();
	delete image;
}

double Map::getDistance(int x, int y) const
{
	return _distances[x][y];
}
