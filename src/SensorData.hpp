#pragma once

class SensorData {
 public:
  double time;
  float opd;
  float shear_x1;
  float shear_x2;
  float shear_y1;
  float shear_y2;
  float point_x1;
  float point_x2;
  float point_y1;
  float point_y2;

  SensorData(double time, float opd, float shear_x1, float shear_x2, float shear_y1, float shear_y2, float point_x1,
             float point_x2, float point_y1, float point_y2) {
    this->time = time;
    this->opd = opd;
    this->shear_x1 = shear_x1;
    this->shear_x2 = shear_x2;
    this->shear_y1 = shear_y1;
    this->shear_y2 = shear_y2;
    this->point_x1 = point_x1;
    this->point_x2 = point_x2;
    this->point_y1 = point_y1;
    this->point_y2 = point_y2;
  }

  SensorData() {
    this->time = 0.0;
    this->opd = 0.0;
    this->shear_x1 = 0.0;
    this->shear_x2 = 0.0;
    this->shear_y1 = 0.0;
    this->shear_y2 = 0.0;
    this->point_x1 = 0.0;
    this->point_x2 = 0.0;
    this->point_y1 = 0.0;
    this->point_y2 = 0.0;
  }

  // acces by index
  float &operator[](int i) {
    switch (i) {
      case 0:
        return opd;
      case 1:
        return shear_x1;
      case 2:
        return shear_x2;
      case 3:
        return shear_y1;
      case 4:
        return shear_y2;
      case 5:
        return point_x1;
      case 6:
        return point_x2;
      case 7:
        return point_y1;
      case 8:
        return point_y2;
      default:
        return opd;
    }
  }
};