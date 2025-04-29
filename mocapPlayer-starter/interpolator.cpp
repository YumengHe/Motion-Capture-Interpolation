#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>
#include "performanceCounter.h"

PerformanceCounter pc;
double LEtime = 0.0;
double LQtime = 0.0;
double BEtime = 0.0;
double BQtime = 0.0;

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

// ------ math helper functions ------
double Qdotproduct(Quaternion<double> & q1, Quaternion<double> & q2) {
  return q1.Gets() * q2.Gets() + q1.Getx() * q2.Getx() + q1.Gety() * q2.Gety() + q1.Getz() * q2.Getz();
}

Quaternion<double> Qnegative(Quaternion<double> & q) {
  Quaternion<double> result;
  double s = q.Gets();
  double x = q.Getx();
  double y = q.Gety();
  double z = q.Getz();
  result.Set(-s, -x, -y, -z);
  return result;
}

// ------ converting ------
// rotation matrix -> Euler
// https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
void Interpolator::Rotation2Euler(double R[9], double angles[3]) {
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  // convert radians to degree
  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

// Euler -> rotation
void Interpolator::Euler2Rotation(double angles[3], double R[9]) { 
  // students should implement this

  // convert degree to radians
  double theta1 = angles[0] * (M_PI / 180.0);
  double theta2 = angles[1] * (M_PI / 180.0);
  double theta3 = angles[2] * (M_PI / 180.0);
  // row 1
  R[0] = cos(theta3) * cos(theta2);
  R[1] = -sin(theta3) * cos(theta1) + cos(theta3) * sin(theta2) * sin(theta1);
  R[2] = sin(theta3) * sin(theta1) + cos(theta3) * sin(theta2) * cos(theta1);
  // row 2
  R[3] = sin(theta3) * cos(theta2);
  R[4] = cos(theta3) * cos(theta1) + sin(theta3) * sin(theta2) * sin(theta1);
  R[5] = -cos(theta3) * sin(theta1) + sin(theta3) * sin(theta2) * cos(theta1);
  // row 3
  R[6] = -sin(theta2);
  R[7] = cos(theta2) * sin(theta1);
  R[8] = cos(theta2) * cos(theta1);
}

// Euler -> Quaternion
void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) {
  // students should implement this

  // rotation matrix
  double R[9];

  // Euler -> rotation matrix
  Euler2Rotation(angles, R);
  // rotation matrix -> Quaternion
  q = Quaternion<double>::Matrix2Quaternion(R);
  // Quaternion -> Unit Quaternion
  q.Normalize();
}

// Quaternion -> Euler
void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) {
  // students should implement this

  // rotation matrix
  double R[9];
  
  // Quaternion -> rotation matrix
  q.Quaternion2Matrix(R);
  // rotation matrix -> Euler
  Rotation2Euler(R, angles);
}

// ------ slerp functions ------
vector Interpolator::Lerp(vector q1, vector q2, double u) {
  vector result;
  result = q1 * (1 - u) + q2 * u;
  return result;
}

Quaternion<double> Interpolator::Slerp(Quaternion<double> & qStart, Quaternion<double> & qEnd, double u) {
  // students should implement this

  Quaternion<double> result;
  double theta;
  Quaternion<double> q1 = qStart;
  Quaternion<double> q2 = qEnd;
  double dotq1q2 = Qdotproduct(q1, q2);

  // determine the shortest arc 
  if(dotq1q2 <= 0.0) {
    q2 = Qnegative(qEnd);
  }

  // special case 1: acos(theta) error when dotq1q2 > 1
  if(dotq1q2 > 1) {
    theta = acos(1.0);
  } else {
    theta = acos(dotq1q2);
  }

  // special case 2: division by zero error when sin(theta) = 0
  if(sin(theta) == 0.0) {
    // do linear interpolation instead of slerp
    result = (1.0 - u) * q1 + u * q2;
    return result;
  }

  result = (sin((1.0 - u) * theta) / sin(theta)) * q1 + (sin(u * theta) / sin(theta)) * q2;
  return result;
}

// Slerp(q1, q2, 2.0)
Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

// ------ main functions ------
// pInputMotion: input motion
// pOutputMotion: output motion
// N: drop N consecutive frames
void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N) { 
  std::cout << "Using Linear Euler!!!!!!" << "\n";

  pc.StartCounter();

  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1); // u

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
  
  pc.StopCounter();
  LEtime = pc.GetElapsedTime();
  std::cout << "Linear Euler computation time: "<< LEtime << std::endl;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N) {
  // students should implement this
  std::cout << "Using Bezier Euler!!!!!!" << "\n";

  pc.StartCounter();

  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0; // qn
  while (startKeyframe + N + 1 < inputLength) {
    int endKeyframe = startKeyframe + N + 1; // qn+1
    int previousKeyframe = startKeyframe - (N + 1); //qn-1
    int nextKeyframe = endKeyframe + N + 1; //qn+2

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * previousPosture;
    Posture * nextPosture;
    
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++) {
      Posture interpolatedPosture;
      double u = 1.0 * frame / (N + 1);

      // interpolate root position
      if(previousKeyframe < 0) { // start
        nextPosture = pInputMotion->GetPosture(nextKeyframe);

        vector q1, q2, q3;
        q1 = startPosture->root_pos;
        q2 = endPosture->root_pos;
        q3 = nextPosture->root_pos;
        // a1 = Lerp(q1, Lerp(q3, q2, 2.0), 1/3)
        vector a1 = Lerp(q1, Lerp(q3, q2, 2.0), 1.0 / 3.0);
        // an+1_dash = Lerp(Lerp(qn, qn+1, 2.0), qn+2, 0.5)
        vector a2_dash = Lerp(Lerp(q1, q2, 2.0), q3, 0.5);
        // bn+1 = Lerp(qn+1, an+1_dash, -1/3)
        vector b2 = Lerp(q2, a2_dash, 1.0 / 3.0);
        
        interpolatedPosture.root_pos = DeCasteljauEuler(u, q1, a1, b2, q2);
        
      } else if(nextKeyframe > inputLength) { // end
        previousPosture = pInputMotion->GetPosture(previousKeyframe);

        vector q0, q1, q2;
        q0 = previousPosture->root_pos;
        q1 = startPosture->root_pos;
        q2 = endPosture->root_pos;
        // an_dash = Lerp(Lerp(qn-1,qn,2.0),qn+1,0.5)
        vector a1_dash = Lerp(Lerp(q0, q1, 2.0), q2, 0.5);
        // an = Lerp(qn, an_dash, 1/3)
        vector a1 = Lerp(q1, a1_dash, 1.0 / 3.0);
        // b2 = Lerp(q2, Lerp(q0, q1, 2.0), 1/3)
        vector b2 = Lerp(q2, Lerp(q0, q1, 2.0), 1.0 / 3.0);
        
        interpolatedPosture.root_pos = DeCasteljauEuler(u, q1, a1, b2, q2);

      } else { // middle
        previousPosture = pInputMotion->GetPosture(previousKeyframe);
        nextPosture = pInputMotion->GetPosture(nextKeyframe);

        vector q0, q1, q2, q3;
        q0 = previousPosture->root_pos;
        q1 = startPosture->root_pos;
        q2 = endPosture->root_pos;
        q3 = nextPosture->root_pos;
        // a1_dash = Lerp(Lerp(qn-1,qn,2.0),qn+1,0.5)
        vector a1_dash = Lerp(Lerp(q0, q1, 2.0), q2, 0.5);
        // a1 = Lerp(qn, an_dash, 1/3)
        vector a1 = Lerp(q1, a1_dash, 1.0 / 3.0);
        // a2_dash = Lerp(Lerp(qn, qn+1, 2.0), qn+2, 0.5)
        vector a2_dash = Lerp(Lerp(q1, q2, 2.0), q3, 0.5);
        // b2 = Lerp(qn+1, an+1_dash, -1/3)
        vector b2 = Lerp(q2, a2_dash, 1.0 / 3.0);
        
        interpolatedPosture.root_pos = DeCasteljauEuler(u, q1, a1, b2, q2);
      }

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        if(previousKeyframe < 0) { // start
          nextPosture = pInputMotion->GetPosture(nextKeyframe);

          vector q1, q2, q3;
          q1 = startPosture->bone_rotation[bone];
          q2 = endPosture->bone_rotation[bone];
          q3 = nextPosture->bone_rotation[bone];
          // a1 = Lerp(q1, Lerp(q3, q2, 2.0), 1/3)
          vector a1 = Lerp(q1, Lerp(q3, q2, 2.0), 1.0 / 3.0);
          // an+1_dash = Lerp(Lerp(qn, qn+1, 2.0), qn+2, 0.5)
          vector a2_dash = Lerp(Lerp(q1, q2, 2.0), q3, 0.5);
          // bn+1 = Lerp(qn+1, an+1_dash, -1/3)
          vector b2 = Lerp(q2, a2_dash, 1.0 / 3.0);
          
          interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(u, q1, a1, b2, q2);
        
        } else if(nextKeyframe > inputLength) { // end
          previousPosture = pInputMotion->GetPosture(previousKeyframe);

          vector q0, q1, q2;
          q0 = previousPosture->bone_rotation[bone];
          q1 = startPosture->bone_rotation[bone];
          q2 = endPosture->bone_rotation[bone];
          // an_dash = Lerp(Lerp(qn-1,qn,2.0),qn+1,0.5)
          vector a1_dash = Lerp(Lerp(q0, q1, 2.0), q2, 0.5);
          // an = Lerp(qn, an_dash, 1/3)
          vector a1 = Lerp(q1, a1_dash, 1.0 / 3.0);
          // b2 = Lerp(q2, Lerp(q0, q1, 2.0), 1/3)
          vector b2 = Lerp(q2, Lerp(q0, q1, 2.0), 1.0 / 3.0);
          
          interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(u, q1, a1, b2, q2);

        } else { // middle
          previousPosture = pInputMotion->GetPosture(previousKeyframe);
          nextPosture = pInputMotion->GetPosture(nextKeyframe);

          vector q0, q1, q2, q3;
          q0 = previousPosture->bone_rotation[bone];
          q1 = startPosture->bone_rotation[bone];
          q2 = endPosture->bone_rotation[bone];
          q3 = nextPosture->bone_rotation[bone];
          // a1_dash = Lerp(Lerp(qn-1,qn,2.0),qn+1,0.5)
          vector a1_dash = Lerp(Lerp(q0, q1, 2.0), q2, 0.5);
          // a1 = Lerp(qn, an_dash, 1/3)
          vector a1 = Lerp(q1, a1_dash, 1.0 / 3.0);
          // a2_dash = Lerp(Lerp(qn, qn+1, 2.0), qn+2, 0.5)
          vector a2_dash = Lerp(Lerp(q1, q2, 2.0), q3, 0.5);
          // b2 = Lerp(qn+1, an+1_dash, -1/3)
          vector b2 = Lerp(q2, a2_dash, - 1.0 / 3.0);
          
          interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(u, q1, a1, b2, q2);
        }
      }
      
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }
    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++) {
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
  }

  pc.StopCounter();
  BEtime = pc.GetElapsedTime();
  std::cout << "Bezier Euler computation time: "<< BEtime << std::endl;
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N) {
  // students should implement this
  std::cout << "Using Linear Quaternion!!!!!!" << "\n";

  pc.StartCounter();

  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength) {
    int endKeyframe = startKeyframe + N + 1;
    
    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame = 1; frame <= N; frame++) {
      Posture interpolatedPosture; //qn
      double u = 1.0 * frame / (N+1); 

      // interpolate root position using Linear Euler
      interpolatedPosture.root_pos = startPosture->root_pos * (1.0 - u) + endPosture->root_pos * u;

      // interpolate bone position
      for(int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        Quaternion<double> q1, q2, result;
        Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
        Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);
        result = Slerp(q1, q2, u);
        Quaternion2Euler(result, interpolatedPosture.bone_rotation[bone].p);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }
    startKeyframe = endKeyframe;
  }
  for(int frame=startKeyframe+1; frame<inputLength; frame++) {
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
  }

  pc.StopCounter();
  LQtime = pc.GetElapsedTime();
  std::cout << "Linear Quaternion computation time: "<< LQtime << std::endl;
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N) {
  // students should implement this
  std::cout << "Using Bezier Quaternion!!!!!!" << "\n";

  pc.StartCounter();

  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0; // qn
  while (startKeyframe + N + 1 < inputLength) {
    int endKeyframe = startKeyframe + N + 1; // qn+1
    int previousKeyframe = startKeyframe - (N + 1); // qn-1
    int nextKeyframe = endKeyframe + N + 1; // qn+2
    
    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * previousPosture;
    Posture * nextPosture;

    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for (int frame = 1; frame <= N; frame++) {
      Posture interpolatedPosture; //qn
      double u = 1.0 * frame / (N+1); 

      // interpolate root position using Bezier Euler
      if (previousKeyframe < 0) { // start
        nextPosture = pInputMotion->GetPosture(nextKeyframe);

        vector q1, q2, q3;
        q1 = startPosture->root_pos;
        q2 = endPosture->root_pos;
        q3 = nextPosture->root_pos;
        // a1 = Lerp(q1, Lerp(q3, q2, 2.0), 1/3)
        vector a1 = Lerp(q1, Lerp(q3, q2, 2.0), 1.0 / 3.0);
        // an+1_dash = Lerp(Lerp(qn, qn+1, 2.0), qn+2, 0.5)
        vector a2_dash = Lerp(Lerp(q1, q2, 2.0), q3, 0.5);
        // bn+1 = Lerp(qn+1, an+1_dash, -1/3)
        vector b2 = Lerp(q2, a2_dash, 1.0 / 3.0);
        
        interpolatedPosture.root_pos = DeCasteljauEuler(u, q1, a1, b2, q2);
        
      } else if (nextKeyframe > inputLength) { // end
        previousPosture = pInputMotion->GetPosture(previousKeyframe);

        vector q0, q1, q2;
        q0 = previousPosture->root_pos;
        q1 = startPosture->root_pos;
        q2 = endPosture->root_pos;
        // an_dash = Lerp(Lerp(qn-1,qn,2.0),qn+1,0.5)
        vector a1_dash = Lerp(Lerp(q0, q1, 2.0), q2, 0.5);
        // an = Lerp(qn, an_dash, 1/3)
        vector a1 = Lerp(q1, a1_dash, 1.0 / 3.0);
        // b2 = Lerp(q2, Lerp(q0, q1, 2.0), 1/3)
        vector b2 = Lerp(q2, Lerp(q0, q1, 2.0), 1.0 / 3.0);
        
        interpolatedPosture.root_pos = DeCasteljauEuler(u, q1, a1, b2, q2);

      } else { // middle
        previousPosture = pInputMotion->GetPosture(previousKeyframe);
        nextPosture = pInputMotion->GetPosture(nextKeyframe);

        vector q0, q1, q2, q3;
        q0 = previousPosture->root_pos;
        q1 = startPosture->root_pos;
        q2 = endPosture->root_pos;
        q3 = nextPosture->root_pos;
        // a1_dash = Lerp(Lerp(qn-1,qn,2.0),qn+1,0.5)
        vector a1_dash = Lerp(Lerp(q0, q1, 2.0), q2, 0.5);
        // a1 = Lerp(qn, an_dash, 1/3)
        vector a1 = Lerp(q1, a1_dash, 1.0 / 3.0);
        // a2_dash = Lerp(Lerp(qn, qn+1, 2.0), qn+2, 0.5)
        vector a2_dash = Lerp(Lerp(q1, q2, 2.0), q3, 0.5);
        // b2 = Lerp(qn+1, an+1_dash, -1/3)
        vector b2 = Lerp(q2, a2_dash, 1.0 / 3.0);
        
        interpolatedPosture.root_pos = DeCasteljauEuler(u, q1, a1, b2, q2);
      }
      
      // interpolate bone position
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        if (previousKeyframe < 0) { // start
          nextPosture = pInputMotion->GetPosture(nextKeyframe);

          Quaternion<double> q1, q2, q3, result;
          Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
          Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);

          // a1 = Slerp(q1, Slerp(q3, q2, 2.0), 1/3)
          Quaternion<double> a1_temp = Slerp(q3, q2, 2.0);
          Quaternion<double> a1 = Slerp(q1, a1_temp, 1.0 / 3.0);
          // a2_dash = Slerp(Slerp(q1, q2, 2.0), q3, 0.5)
          Quaternion<double> a2_dash_temp = Slerp(q1, q2, 2.0);
          Quaternion<double> a2_dash = Slerp(a2_dash_temp, q3, 0.5);
          // b2 = Slerp(q2, a2_dash, -1/3)
          Quaternion<double> b2 = Slerp(q2, a2_dash, - 1.0 / 3.0);
          
          result = DeCasteljauQuaternion(u, q1, a1, b2, q2);
          Quaternion2Euler(result, interpolatedPosture.bone_rotation[bone].p);

        } else if (nextKeyframe > inputLength) { // end
          previousPosture = pInputMotion->GetPosture(previousKeyframe);

          Quaternion<double> q0, q1, q2, result;
          Euler2Quaternion(previousPosture->bone_rotation[bone].p, q0);
          Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
          Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);

          // a1_dash = Slerp(Slerp(q0, q1, 2.0), q2, 0.5)
          Quaternion<double> a1_dash_temp = Slerp(q0, q1, 2.0);
          Quaternion<double> a1_dash = Slerp(a1_dash_temp, q2, 0.5);
          // a1 = Slerp(q1, a1_dash, 1/3)
          Quaternion<double> a1 = Slerp(q1, a1_dash, 1.0 / 3.0);
          // b2 = Slerp(q2, Slerp(q0, q1, 2.0), 1/3)
          Quaternion<double> b2_temp = Slerp(q0, q1, 2.0);
          Quaternion<double> b2 = Slerp(q2, b2_temp, 1.0 / 3.0);
          
          result = DeCasteljauQuaternion(u, q1, a1, b2, q2);
          Quaternion2Euler(result, interpolatedPosture.bone_rotation[bone].p);

        } else { // middle
          previousPosture = pInputMotion->GetPosture(previousKeyframe);
          nextPosture = pInputMotion->GetPosture(nextKeyframe);

          Quaternion<double> q0, q1, q2, q3, result;
          Euler2Quaternion(previousPosture->bone_rotation[bone].p, q0);
          Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
          Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);

          // a1_dash = Slerp(Slerp(q0, q1, 2.0), q2, 0.5)
          Quaternion<double> a1_dash_temp = Slerp(q0, q1, 2.0);
          Quaternion<double> a1_dash = Slerp(a1_dash_temp, q2, 0.5);
          // a1 = Slerp(q1, a1_dash, 1/3)
          Quaternion<double> a1 = Slerp(q1, a1_dash, 1.0 / 3.0);
          // a2_dash = Slerp(Slerp(q1, q2, 2.0), q3, 0.5)
          Quaternion<double> a2_dash_temp = Slerp(q1, q2, 2.0);
          Quaternion<double> a2_dash = Slerp(a2_dash_temp, q3, 0.5);
          // b2 = Slerp(q2, a2_dash, -1/3)
          Quaternion<double> b2 = Slerp(q2, a2_dash, - 1.0 / 3.0);
          
          result = DeCasteljauQuaternion(u, q1, a1, b2, q2);
          Quaternion2Euler(result, interpolatedPosture.bone_rotation[bone].p);
        }
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }
    startKeyframe = endKeyframe;
  }
  for(int frame=startKeyframe+1; frame<inputLength; frame++) {
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
  }

  pc.StopCounter();
  BQtime = pc.GetElapsedTime();
  std::cout << "Bezier Quaternion computation time: "<< BQtime << std::endl;
}

// ------ DeCasteljau ------
vector Interpolator::DeCasteljauEuler(double u, vector p0, vector p1, vector p2, vector p3) {
  // students should implement this

  vector result;
  vector Q0 = Lerp(p0, p1, u);
  vector Q1 = Lerp(p1, p2, u);
  vector Q2 = Lerp(p2, p3, u);
  vector R0 = Lerp(Q0, Q1, u);
  vector R1 = Lerp(Q1, Q2, u);
  result = Lerp(R0, R1, u); // P(t)

  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double u, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3) {
  // students should implement this

  Quaternion<double> result;
  Quaternion<double> Q0 = Slerp(p0, p1, u);
  Quaternion<double> Q1 = Slerp(p1, p2, u);
  Quaternion<double> Q2 = Slerp(p2, p3, u);
  Quaternion<double> R0 = Slerp(Q0, Q1, u);
  Quaternion<double> R1 = Slerp(Q1, Q2, u);
  result = Slerp(R0, R1, u); // P(t)

  return result;
}

