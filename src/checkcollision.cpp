/**
 * @file checkcollision.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief check collision function singleton class
 * @version 0.1
 * @date 2021-01-17
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <typeindex>

#include "models/ackermann_model.hpp"
#include "models/holonomic_model.hpp"
#include "state.hpp"

class CheckCollision {
 public:
  static CheckCollision& getInstance() {
    static CheckCollision instance;  // Guaranteed to be destroyed.
    return instance;
  }

 private:
  CheckCollision() {}

 public:
  CheckCollision(CheckCollision const&) = delete;
  void operator=(CheckCollision const&) = delete;

  bool collideAgents(const State& s1, const State& s2,
                     const std::shared_ptr<Model>& model1,
                     const std::shared_ptr<Model>& model2) {
    if (pow(s1.x - s2.x, 2) + pow(s1.y - s2.y, 2) <
        pow(model1->safetyRadius() + model2->safetyRadius(), 2))
      return true;
    else
      return false;
  }

  bool collideObstacle(const State& s, const Location& obstacle,
                       const std::shared_ptr<Model>& m) {
    return m->collideObstacle(s, obstacle);
  }
};
