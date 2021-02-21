/**
 * @file model.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief model base class
 * @date 2021-01-13
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <iostream>

#include "neighbor.hpp"
#include "state.hpp"

using libMultiRobotPlanning::Neighbor;
using Action = int;

class Model {
 public:
  Model(){};
  ~Model(){};
  virtual std::vector<Neighbor<State, Action, double>> getNeighbors(
      const State &, Action) = 0;
  virtual bool isSolution(const State &, const State &,
                          std::vector<std::tuple<State, Action, double>> &) = 0;
  virtual double admissibleHeuristic(const State &, const State &) = 0;
  virtual bool collideObstacle(const State &s, const Location &obstacle) = 0;
  virtual double safetyRadius() = 0;
};
