#pragma once

#include <iostream>
#include "../include/common/cppTypes.h"

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;
using namespace std;


/**
 * @file gaitgenerator.h
 * @brief Gait Generation for Bipedal Locomotion
 *
 * This file provides the definition for the Gait class, which is responsible for
 * generating and managing various gait patterns for a bipedal robot. 
 *
 */

/*
生成和管理双足机器人的步态模式
步态模式是指机器人行走时脚步的节奏和顺序。这个类包含了一些方法来计算和管理这些步态模式。
*/
class Gait
{
public:
  Gait(int nMPC_segments, Vec2<int> offsets, Vec2<int>  durations, const std::string& name="");
  ~Gait();
  Vec2<double> getContactSubPhase();
  Vec2<double> getSwingSubPhase();
  int* mpc_gait();
  void setIterations(int iterationsPerMPC, int currentIteration);
  //表示站立相的时间，即脚与地面接触的时间
  int _stance;
  //表示摆动相的时间，即脚在空中摆动的时间
  int _swing;


private:
  int _nMPC_segments;
  int* _mpc_table;
  Array2i _offsets;           // offset in mpc segments
  Array2i _durations;         // duration of step in mpc segments
  Array2d _offsetsPhase;      // offsets in phase (0 to 1)
  Array2d _durationsPhase;    // durations in phase (0 to 1)
  int _iteration;
  int _nIterations;
  int currentIteration;
  double _phase;

};