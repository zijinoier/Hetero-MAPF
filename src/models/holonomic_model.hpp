/**
 * @file holonomic_model.hpp
 * @author  Licheng Wen (wenlc@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2021-01-20
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include "model.hpp"
class HolonomicModel : public Model {
 private:
  /* data */
 public:
  HolonomicModel() : Model(){};
  HolonomicModel(float width, float v) : Model(), width(width) {
    stepLength *= v;
    // std::cout << "Successfully construct a Holonomic model with width
    // " << width << std::endl;
  };
  ~HolonomicModel(){};
  std::vector<Neighbor<State, Action, double>> getNeighbors(const State &s,
                                                            Action action) {
    std::vector<Neighbor<State, Action, double>> neighbors;
    double g = stepLength;
    std::vector<std::vector<int>> directions{{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (int act = 0; act < (int)directions.size(); act++) {
      State tempState(s.x + g * directions[act][0],
                      s.y + g * directions[act][1], s.yaw, s.time + 1);
      if (act != action)
        neighbors.emplace_back(Neighbor<State, Action, double>(
            tempState, act, g * Constants::penaltyTurning));
      else
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
    }
    // wait
    g = stepLength;
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    return neighbors;
  }
  bool isSolution(const State &state, const State &goal,
                  std::vector<std::tuple<State, Action, double>> &path) {
    double goal_distance =
        sqrt(pow(state.x - goal.x, 2) + pow(state.y - goal.y, 2));
    if (goal_distance > 3 * width) return false;
    std::vector<std::vector<int>> directions{{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    State tempState = state;
    path.clear();
    path.emplace_back(std::make_tuple<>(state, 0, 0));
    // X direction
    int act = (goal.x - state.x > 0) ? 0 : 2;
    for (int i = 0; i < (int)(abs(goal.x - state.x) / stepLength); i++) {
      tempState.x += stepLength * directions[act][0];
      tempState.time++;
      path.emplace_back(std::make_tuple<>(tempState, act, stepLength));
    }
    double ratio = abs(tempState.x - goal.x) / stepLength;
    tempState.x = goal.x;
    tempState.time++;
    path.emplace_back(std::make_tuple<>(tempState, act, ratio * stepLength));
    // Y direction
    act = (goal.y - state.y > 0) ? 1 : 3;
    for (int i = 0; i < (int)(abs(goal.y - state.y) / stepLength); i++) {
      tempState.y += stepLength * directions[act][1];
      tempState.time++;
      path.emplace_back(std::make_tuple<>(tempState, act, stepLength));
    }
    ratio = abs(tempState.y - goal.y) / stepLength;
    tempState.y = goal.y;
    tempState.time++;
    path.emplace_back(std::make_tuple<>(tempState, act, ratio * stepLength));

    // std::cout << state << " " << goal << std::endl;
    // for (auto p : path) {
    //   std::cout << std::get<0>(p) << " " << std::get<1>(p) << " "
    //             << std::get<2>(p) << std::endl;
    // }
    // std::cout << "------------\n";
    return true;
  }

  double admissibleHeuristic(const State &s, const State &goal) {
    return sqrt(pow(goal.x - s.x, 2) + pow(goal.y - s.y, 2));
  }

  bool collideObstacle(const State &s, const Location &obstacle) {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - s.x;
    obs(0, 1) = obstacle.y - s.y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, s.rot);
    if (rotated_obs(0, 0) > -width / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 0) < width / 2.0 + Constants::obsRadius &&
        rotated_obs(0, 1) > -width / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 1) < width / 2.0 + Constants::obsRadius)
      return true;
    return false;
  }

  double safetyRadius() {
    return sqrt(pow(width / 2.0, 2) + pow(width / 2.0, 2));
  }

 private:
  float width = Constants::holoCarWidth;
  float stepLength = 1 * Constants::holoStepLength;
};
