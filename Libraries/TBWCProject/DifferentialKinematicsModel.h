#ifndef DifferentialKinematicsModel_h
#define DifferentialKinematicsModel_h

#include "Arduino.h"

class DifferentialKinematicsModel {

  public:
    DifferentialKinematicsModel(float trackWidth, float *linearVel, float *angularVel);

    void calculate();

    float rWheelSpeed = 0.0F;
    float lWheelSpeed = 0.0F;
  private:
    float _trackWidth;
    float *_linearVel;
    float *_angularVel;


};

#endif