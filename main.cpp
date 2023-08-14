#include <string>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <SDL2/SDL.h>

//#include <Eigen/Core>
#include <Eigen/Dense>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.hpp"

#define die(err) {fprintf(stderr, "%s:%d\t[FATAL] %s\n", __FILE__, __LINE__, (err)); exit(-1);}

FILE* logfile;

#define log(s) {fprintf(logfile, "%s:%d\t[LOG] %s\n", __FILE__, __LINE__, (s));}
#define ABS(x) (((x) > 0)?(x):(-(x)))

#define WIDTH 1366
#define HEIGHT 768

SDL_Window* win = NULL;
SDL_Surface* s = NULL;
float zbuffer[WIDTH*HEIGHT];
struct color_t{
  uint8_t a;
  uint8_t r;
  uint8_t g;
  uint8_t b;

  color_t operator*(float f) const {
    return color_t{a, (uint8_t)(this->r*f), (uint8_t)(this->g*f), (uint8_t)(this->b*f)};
  }


  color_t operator+(color_t&& right) {
    return color_t{a,(uint8_t)( r + right.r ),(uint8_t)( g + right.g ), (uint8_t)( b + right.b )};
  }
};

typedef struct color_t color;

typedef struct model_t {
  float* vertices;
  int* faces;
} model;

typedef struct point_t {
  int x;
  int y;
}point;

void set_pixel(int x, int y, color c) {
  Uint32 *target_pixel = (Uint32*)( (Uint8 *) s->pixels + y * s->pitch +
    x * sizeof *target_pixel );
  *target_pixel = c.b | (c.g << 8) | (c.r << 16) | (c.a << 24);
}


void line(int x0, int y0, int x1, int y1, color c) {
  int sweepx = 1; 
  if(ABS(x1 - x0) < ABS(y1 - y0)) sweepx = 0;
  if(sweepx) {
    if(x1 < x0) {
      int temp = x0;
      x0 = x1;
      x1 = temp;
      temp = y0;
      y0 = y1;
      y1 = temp;
    }
    int y = y0;
    float slope = ABS(y1 - y0)/(float)ABS(x1 -  x0);
    float dy = 0;
    for(int x = x0; x <= x1; x++) {
      set_pixel(x, y, c);
      dy += slope;
      if(dy >= 1) {
	dy -= 1;
	y += (y1 > y0)?1:-1;
      }
    }
  }
  else {
   if(y1 < y0) {
      int temp = x0;
      x0 = x1;
      x1 = temp;
      temp = y0;
      y0 = y1;
      y1 = temp;
   }
   int x = x0;
   float inverseslope = ABS(x1 - x0)/(float)ABS(y1 - y0);
   float dx = 0;
   for(int y = y0; y < y1; y++) {
     set_pixel(x, y, c);
     dx += inverseslope;
     if(dx >= 1) {
       dx -= 1;
       x += (x1 > x0)?1:-1;
     }
   }
  }
}

bool compare_y_in_point(const point& p1, const point& p2) {
  return p1.y < p2.y;
}


struct lin_eq_coeff{
  float a;
  float b;
  float c;
};
inline float calc_lin_eq(lin_eq_coeff c, float x, float y) {
  return c.a*x + c.b*y + c.c;
}

inline float solveforx_linear(lin_eq_coeff c, float y) {
  return (c.a != 0)?-(c.b*y + c.c)/c.a : -1;
}

void drawTri2(std::array<point, 3> triangle, color c) {
  std::sort(triangle.begin(), triangle.end(), compare_y_in_point);
  lin_eq_coeff c01;
  lin_eq_coeff c12;
  lin_eq_coeff c02;
  if(triangle[1].x == triangle[0].x)
    c01 = {1, 0, -(float)triangle[1].x};
  else
    c01 = {-(triangle[1].y - triangle[0].y)/(float)(triangle[1].x - triangle[0].x),  1, -triangle[0].y + triangle[0].x*(triangle[1].y - triangle[0].y)/(float)(triangle[1].x - triangle[0].x)};

  if(triangle[1].x == triangle[2].x)
    c12 = {1, 0, -(float)triangle[1].x};
  else
  c12 = {.a = -(triangle[2].y - triangle[1].y)/(float)(triangle[2].x - triangle[1].x), .b = 1, .c = -triangle[1].y + triangle[1].x*(triangle[2].y - triangle[1].y)/(float)(triangle[2].x - triangle[1].x)};

  if(triangle[0].x == triangle[2].x)
    c02 = {1, 0, -(float)triangle[2].x};
  else
  c02 = {.a = -(triangle[2].y - triangle[0].y)/(float)(triangle[2].x - triangle[0].x), .b = 1, .c = -triangle[0].y + triangle[0].x*(triangle[2].y - triangle[0].y)/(float)(triangle[2].x - triangle[0].x)};
  int y = 0;
  for(y = triangle[0].y + 1; y <= triangle[1].y; y++) {
    float x[2] = {solveforx_linear(c02, y), solveforx_linear(c01, y)};
    std::sort(x, x + 2, std::greater<float>());
    if(x[0] < 0)
      break;
    x[0] = std::floor(x[0]);
    x[1] = std::ceil(x[1]);
    line((int)x[0], y, (int)x[1], y, c);
  }
  for(; y < triangle[2].y; y++) {
    float x[2] = {solveforx_linear(c02, y), solveforx_linear(c12, y)};
    std::sort(x, x + 2, std::greater<float>());
    x[0] = std::floor(x[0]);
    x[1] = std::ceil(x[1]);
    line((int)x[0], y, (int)x[1], y, c);
  }
}

void drawTri(std::vector<point> points, color c) {
  std::sort(points.begin(), points.end(), compare_y_in_point);
  log("Points given to draw in ascending order by y");
  char msg[1024];
  sprintf(msg, "P1: %d, %d\nP2: %d, %d\nP3: %d, %d", points[0].x, points[0].y,points[1].x, points[1].y,points[2].x, points[2].y);
  log(msg);
  lin_eq_coeff c01 = {-(points[1].y - points[0].y)/(float)(points[1].x - points[0].x),  1, -points[0].y + points[0].x*(points[1].y - points[0].y)/(float)(points[1].x - points[0].x)};
  lin_eq_coeff c12 = {.a = -(points[2].y - points[1].y)/(float)(points[2].x - points[1].x), .b = 1, .c = -points[1].y + points[1].x*(points[2].y - points[1].y)/(float)(points[2].x - points[1].x)};
  std::vector<point> starts;
  bool sweepy = true;
  if(ABS(points[2].y - points[0].y) < ABS(points[2].x - points[0].x)) sweepy = false;
  if(sweepy) {
    float islope = ABS(points[2].x - points[0].x)/(float)ABS(points[2].y - points[0].y);
    int x = points[0].x + (int)islope;
    float dx = islope;
    for(int y = points[0].y + 1; y < points[2].y; y++) {
      starts.push_back({x, y});
      dx += islope;
      if(dx >= 1) {
	dx -= 1;
	x += (points[2].x > points[0].x)?1:-1;
      }
    }
  }
  else {
    float slope = ABS(points[2].y - points[0].y)/(float)ABS(points[2].x - points[0].x);
    float dy = slope;
    int dx = (points[2].x - points[0].x)/ABS(points[2].x - points[0].x);
    int y = points[0].y + (int)slope*dx;
    for(int x = points[0].x + dx; x!= points[2].x; x += dx) {
      starts.push_back({x, y});
      dy += slope;
      if(dy >= 1) {
	dy -= 1;
	y += (points[2].y > points[1].y)?1:-1;
      }
    }
  }
  int i = 0;
  float s = calc_lin_eq(c01, starts[0].x, starts[0].y);
  s /= ABS(s);
  bool rasterize_right = true;
  int x_on02 = points[0].x + (int)( (points[1].y - points[0].y)/(float)(points[2].y - points[0].y) * (points[2].x - points[0].x) );
  if(points[1].x - x_on02 < 0) rasterize_right = false;
  for(; starts[i].y != points[1].y; i++) {
    int y = starts[i].y;
    int x = starts[i].x;
    float s_ = calc_lin_eq(c01, starts[i].x, starts[i].y);
    while((s > 0 && s_ > 0) || (s < 0 && s_ < 0)) {
      set_pixel(x, y, c);
      if(rasterize_right) x++;
      else x--;
      if(x > WIDTH || x < 0) break;
      s_ = calc_lin_eq(c01, x, y);
    }
  }
  s = calc_lin_eq(c12, starts[i].x, starts[i].y);
  s /= ABS(s);
  for(; i < starts.size(); i++) {
    int y = starts[i].y;
    int x = starts[i].x;
    float s_ = calc_lin_eq(c12, starts[i].x, starts[i].y);
    while(s == s_/ABS(s_)) {
      set_pixel(x, y, c);
      if(rasterize_right) x++;
      else x--;
      if(x > WIDTH || x < 0) break;
      s_ = calc_lin_eq(c12, x, y);
    }
  }
  set_pixel(points[0].x, points[0].y, c);
  set_pixel(points[1].x, points[1].y, c);
  set_pixel(points[2].x, points[2].y, c);
}

Eigen::Vector3f barycentric(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3, const Eigen::Vector3f& P) {
  Eigen::Vector3f p21 = ( p2 - p1 ).cast<float>();
  Eigen::Vector3f p31 = ( p3 - p1 ).cast<float>();
  Eigen::Vector3f p1P = ( p1 - P ).cast<float>();
  Eigen::Vector3f Vx{p21[0], p31[0], p1P[0]};
  Eigen::Vector3f Vy{p21[1], p31[1], p1P[1]};
  Eigen::Vector3f temp = Vx.cross(Vy);
  if(std::abs(temp[2]) < 1) return Eigen::Vector3f{-1.f, -1.f, -1.f};
  Eigen::Vector3f ret{1- ( temp[0] + temp[1] )/temp[2], temp[0]/temp[2], temp[1]/temp[2]};
  return ret;
}

void drawTriBarycentric(const std::array<Eigen::Vector3f, 3>& tri, const std::array<color, 3> c, float* zbuffer) {
  Eigen::Vector2i bbox_ul = {WIDTH - 1, HEIGHT - 1};
  Eigen::Vector2i bbox_lr = {0, 0};
  for(int i = 0; i < 3; i++) {
    if(tri[i][0] < bbox_ul[0]) bbox_ul[0] = tri[i][0];
    if(tri[i][1] < bbox_ul[1]) bbox_ul[1] = tri[i][1];
    if(tri[i][0] > bbox_lr[0]) bbox_lr[0] = tri[i][0];
    if(tri[i][1] > bbox_lr[1]) bbox_lr[1] = tri[i][1];
  }
  for(int j = bbox_ul[1]; j <= bbox_lr[1]; j++) {
    for(int i = bbox_ul[0]; i <= bbox_lr[0]; i++) {
      Eigen::Vector3f b = barycentric(tri[0], tri[1], tri[2], Eigen::Vector3f{(float)i, (float)j, 0});
      if(b[0] < 0 || b[1] < 0 || b[2] < 0) continue;
      float k = b[0]*tri[0][2] + b[1]*tri[1][2] + b[2]*tri[2][2];
      if(zbuffer[j*WIDTH + i] < k) {
	zbuffer[j*WIDTH + i] = k;
	set_pixel(i, j, c[0]*b[0] + c[1]*b[1] + c[2]*b[2]);
      }
    }
  }
}


void drawObj(std::string filename) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string w;
  std::string e;
  if(!tinyobj::LoadObj(&attrib, &shapes, &materials, &w, &e, filename.c_str())) {
    log("Unable to load obj");
    die(e.c_str());
  }
  for(int i = 0; i < WIDTH*HEIGHT; i++) {
    zbuffer[i] = -std::numeric_limits<float>::max();
  }
  for(size_t s = 0; s < shapes.size(); s++) {
    for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      tinyobj::index_t idx1 = shapes[s].mesh.indices[f*3];
      tinyobj::index_t idx2 = shapes[s].mesh.indices[f*3 + 1];
      tinyobj::index_t idx3 = shapes[s].mesh.indices[f*3 + 2];
      int x1 = ( attrib.vertices[3*idx1.vertex_index] + 1. )*WIDTH/2;
      if(x1 >= WIDTH) x1 = WIDTH - 1;
      int y1 = ( 1. - attrib.vertices[3*idx1.vertex_index + 1] )*HEIGHT/2;
      if(y1 >= HEIGHT) y1 = HEIGHT - 1;
      int x2 = ( attrib.vertices[3*idx2.vertex_index] + 1. )*WIDTH/2;
      if(x2 >= WIDTH) x2 = WIDTH - 1;
      int y2 = ( 1. - attrib.vertices[3*idx2.vertex_index + 1] )*HEIGHT/2;
      if(y2 >= HEIGHT) y2 = HEIGHT - 1;
      int x3 = ( attrib.vertices[3*idx3.vertex_index] + 1. )*WIDTH/2;
      if(x3 >= WIDTH) x3 = WIDTH - 1;
      int y3 = ( 1. - attrib.vertices[3*idx3.vertex_index + 1] )*HEIGHT/2;
      if(y3 >= HEIGHT) y3 = HEIGHT - 1;
      color colors[] = {{0xff, 0xff, 0x00, 0x00}, {0xff, 0x00, 0xff, 0x00}, {0xff, 0x00, 0x00, 0xff}};
      std::array<point, 3> tri{point{x1, y1}, {x2, y2}, {x3, y3}};
      Eigen::Vector3f v1{attrib.vertices[3*idx1.vertex_index],attrib.vertices[3*idx1.vertex_index + 1] , attrib.vertices[3*idx1.vertex_index + 2]};
      Eigen::Vector3f v2{attrib.vertices[3*idx2.vertex_index],attrib.vertices[3*idx2.vertex_index + 1] , attrib.vertices[3*idx2.vertex_index + 2]};
      Eigen::Vector3f v3{attrib.vertices[3*idx3.vertex_index],attrib.vertices[3*idx3.vertex_index + 1] , attrib.vertices[3*idx3.vertex_index + 2]};
      Eigen::Vector3f vn = (v2 - v1).cross(v3 - v1);
      vn = vn.normalized();
      Eigen::Vector3f vlight{0.0f, 1.0f, 1.0f};
      vlight = vlight.normalized();
      float intensity = 0.5*vn.dot(vlight);
      char msg[100];
      sprintf(msg, "Intensity: %f", intensity);
      log(msg);
      sprintf(msg, "Z: %f %f %f", v1[2], v2[2], v3[2]);
      log(msg);
      if(intensity > 0.0){
        // drawTri2(tri, color{0xff,
        // (uint8_t)(0xff*intensity),(uint8_t)(0xff*intensity),(uint8_t)(0xff*intensity)});
	std::array<Eigen::Vector3f, 3> tri{ Eigen::Vector3f{(float)x1, (float)y1, v1[2]}, {(float)x2, (float)y2, v2[2]}, {(float)x3, (float)y3, v3[2]}};
	color c = {0xff, 0xff, 0xff, 0xff};
	c = c*intensity;
	drawTriBarycentric(tri, std::array<color, 3>{c, c, c}, zbuffer);
      }
    }
  }
}



int main(int argc, char** argv) {
  for(int i = 0; i < argc; i++) {
    if(!strcmp(argv[i], "-lf")) {
      if(argc > i + 1) 
	logfile = fopen(argv[i + 1], "a");
      else
	die("Log file not mentioned");
    }
  }
  if(!logfile) logfile = stderr;
  SDL_Init(SDL_INIT_VIDEO);
  log("SDL initialized");
  win = SDL_CreateWindow("WINDOW", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
  if(!win) die("Unable to create window");
  s = SDL_GetWindowSurface(win);
  if(!s) die("Unable to get surface");
  color red = {0xff, 0xff, 0, 0};
  color blue = {0xff, 0, 0, 0xff};
  color white = {0xff, 0xff, 0xff, 0xff};
  for(int i = 0; i < WIDTH*HEIGHT; i++) {
    zbuffer[i] = -std::numeric_limits<float>::max();
  }
  printf("MINIMUM : %f", *zbuffer);

  //  std::vector<point> tri1 = {{0, 0}, {10, 70}, {20, 40}};
  //  std::array<Eigen::Vector3f, 3> tri1{Eigen::Vector3f{50.f, 90.f, 0.f}, {100.f, 99.f, 0.f}, {150.f, 100.f, 0.f}};
  //  for(int i = 0; i <= 5*60; i++) {
  //    for(int i = 0; i < WIDTH*HEIGHT; i++) {
  //      zbuffer[i] = -std::numeric_limits<float>::max();
  //    }
  //
  //    SDL_FillRect(s, NULL, 0);
  //    float r = 2 * M_PI/(5*60) * i;
  //    tri1[2][0] = 100 + 50*cos(r);
  //    tri1[2][1] = 100 - 50*sin(r);
  //    drawTriBarycentric(tri1, {blue, blue, blue}, zbuffer);
  //    line(tri1[0][0], tri1[0][1], tri1[1][0], tri1[1][1], red);
  //    line(tri1[0][0], tri1[0][1], tri1[2][0], tri1[2][1], red);
  //    line(tri1[2][0], tri1[2][1], tri1[1][0], tri1[1][1], red);
  //    SDL_UpdateWindowSurface(win);
  //    SDL_Delay(16);
  //  }
  //  line(0, 0, 100, 100, red);
  //  line(0,0, 200, 300, red);
  //  line(100, 100, 200, 300, red);
  SDL_FillRect(s, NULL, 0);
  SDL_UpdateWindowSurface(win);
  drawObj("./test.obj");
  //drawObj("./bunny.obj");
  SDL_UpdateWindowSurface(win);
  sleep(4);
  
  SDL_DestroyWindow(win);
  return 0;
}
