/**
 * @file ackermann_model.hpp
 * @author  Licheng Wen (wenlc@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2021-01-18
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <iostream>

#include "constants.hpp"
#include "model.hpp"

typedef ompl::base::SE2StateSpace::StateType OmplState;

// TODO: add agent size
class AckermannModel : public Model {
 public:
  AckermannModel() : Model(){};
  AckermannModel(float LB, float LF, float width, float r, float v)
      : Model(), LB(LB), LF(LF), width(width), r(r) {
    deltat *= v;
    dyaw = std::vector<double>({0, deltat, -deltat, 0, -deltat, deltat});
    dx = std::vector<double>({r * deltat, r * sin(deltat), r * sin(deltat),
                              -r * deltat, -r * sin(deltat), -r * sin(deltat)});
    dy =
        std::vector<double>({0, -r * (1 - cos(deltat)), r * (1 - cos(deltat)),
                             0, -r * (1 - cos(deltat)), r * (1 - cos(deltat))});
    // std::cout << "Successfully build a Ackermann Model with " << LB << " " <<
    // LF
    //           << " " << width << std::endl;
  }
  ~AckermannModel(){};

  bool isSolution(const State &state, const State &goal,
                  std::vector<std::tuple<State, Action, double>> &path) {
    double goal_distance =
        sqrt(pow(state.x - goal.x, 2) + pow(state.y - goal.y, 2));
    if (goal_distance > 3 * (LB + LF)) return false;
    // Analytical expansion
    ompl::base::ReedsSheppStateSpace reedsSheppSpace(r);
    OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
    OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
    rsStart->setXY(state.x, state.y);
    rsStart->setYaw(-state.yaw);
    rsEnd->setXY(goal.x, goal.y);
    rsEnd->setYaw(-goal.yaw);
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
        reedsSheppSpace.reedsShepp(rsStart, rsEnd);

    path.clear();
    path.emplace_back(std::make_tuple<>(state, 0, 0));
    for (auto pathidx = 0; pathidx < 5; pathidx++) {
      if (fabs(reedsShepppath.length_[pathidx]) < 1e-6) continue;
      double deltat = 0, dx = 0, act = 0, cost = 0;
      switch (reedsShepppath.type_[pathidx]) {
        case 0:  // RS_NOP
          continue;
          break;
        case 1:  // RS_LEFT
          deltat = -reedsShepppath.length_[pathidx];
          dx = r * sin(-deltat);
          // dy = r * (1 - cos(-deltat));
          act = 2;
          cost =
              reedsShepppath.length_[pathidx] * r * Constants::penaltyTurning;
          break;
        case 2:  // RS_STRAIGHT
          deltat = 0;
          dx = reedsShepppath.length_[pathidx] * r;
          // dy = 0;
          act = 0;
          cost = dx;
          break;
        case 3:  // RS_RIGHT
          deltat = reedsShepppath.length_[pathidx];
          dx = r * sin(deltat);
          // dy = -r * (1 - cos(deltat));
          act = 1;
          cost =
              reedsShepppath.length_[pathidx] * r * Constants::penaltyTurning;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown ReedsSheppPath type"
                    << "\033[0m\n";
          break;
      }
      if (cost < 0) {
        cost = -cost * Constants::penaltyReversing;
        act = act + 3;
      }
      generatePath(act, deltat, dx, path);
    }
    return true;
  };

  std::vector<Neighbor<State, Action, double>> getNeighbors(const State &s,
                                                            Action action) {
    std::vector<Neighbor<State, Action, double>> neighbors;
    double g = dx[0];
    double xSucc, ySucc, yawSucc;
    for (Action act = 0; act < 6; act++) {  // has 6 directions for Reeds-Shepp
      g = dx[0];
      xSucc = s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw);
      ySucc = s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw[act]);

      if (act % 3 != 0) {  // penalize turning
        g = g * Constants::penaltyTurning;
      }
      if ((act < 3 && action >= 3) || (action < 3 && act >= 3)) {
        // penalize change of direction
        g = g * Constants::penaltyCOD;
      }
      if (act >= 3) {  // backwards
        g = g * Constants::penaltyReversing;
      }
      State tempState(xSucc, ySucc, yawSucc, s.time + 1);
      neighbors.emplace_back(
          Neighbor<State, Action, double>(tempState, act, g));
    }
    // wait
    g = dx[0];
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    return neighbors;
  };

  double admissibleHeuristic(const State &s, const State &goal) {
    // std::cout << "come to Ackermann function\n";
    // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
    ompl::base::ReedsSheppStateSpace reedsSheppPath(r);
    OmplState *rsStart = (OmplState *)reedsSheppPath.allocState();
    OmplState *rsEnd = (OmplState *)reedsSheppPath.allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(goal.x, goal.y);
    rsEnd->setYaw(goal.yaw);
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
    // Euclidean distance
    double euclideanCost = sqrt(pow(goal.x - s.x, 2) + pow(goal.y - s.y, 2));
    // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
    // std::cout << "holonomic cost:" << twoDCost << std::endl;
    return std::max({reedsSheppCost, euclideanCost});
  };

 private:
  bool generatePath(int act, double deltaSteer, double deltaLength,
                    std::vector<std::tuple<State, Action, double>> &result) {
    double xSucc, ySucc, yawSucc, restx, resty, restyaw, ratio;
    // result.emplace_back(std::make_pair<>(startState, 0));
    if (act == 0 || act == 3) {
      for (size_t i = 0; i < (size_t)(deltaLength / dx[act]); i++) {
        State s = std::get<0>(result.back());
        xSucc = s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw);
        ySucc = s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw[act]);
        State nextState(xSucc, ySucc, yawSucc,
                        std::get<0>(result.back()).time + 1);
        result.emplace_back(std::make_tuple<>(nextState, act, dx[0]));
      }
      ratio = (deltaLength - (int)(deltaLength / dx[act]) * dx[act]) / dx[act];
      restyaw = 0;
      restx = ratio * dx[act];
      resty = 0;
    } else {
      for (size_t i = 0; i < (size_t)(deltaSteer / dyaw[act]); i++) {
        State s = std::get<0>(result.back());
        xSucc = s.x + dx[act] * cos(-s.yaw) - dy[act] * sin(-s.yaw);
        ySucc = s.y + dx[act] * sin(-s.yaw) + dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw[act]);
        State nextState(xSucc, ySucc, yawSucc,
                        std::get<0>(result.back()).time + 1);
        result.emplace_back(std::make_tuple<>(
            nextState, act, dx[0] * Constants::penaltyTurning));
      }
      ratio =
          (deltaSteer - (int)(deltaSteer / dyaw[act]) * dyaw[act]) / dyaw[act];
      restyaw = ratio * dyaw[act];
      restx = r * sin(restyaw);
      resty = -r * (1 - cos(restyaw));
      if (act == 2 || act == 5) {
        restx = -restx;
        resty = -resty;
      }
    }
    State s = std::get<0>(result.back());
    xSucc = s.x + restx * cos(-s.yaw) - resty * sin(-s.yaw);
    ySucc = s.y + restx * sin(-s.yaw) + resty * cos(-s.yaw);
    yawSucc = Constants::normalizeHeadingRad(s.yaw + restyaw);
    // std::cout << m_agentIdx << " ratio::" << ratio << std::endl;
    State nextState(xSucc, ySucc, yawSucc, std::get<0>(result.back()).time + 1);
    result.emplace_back(std::make_tuple<>(nextState, act, ratio * dx[0]));
    // std::cout << "Have generate " << result.size() << " path
    // segments:\n\t"; for (auto iter = result.begin(); iter != result.end();
    // iter++)
    //   std::cout << iter->first << ":" << iter->second << "->";
    // std::cout << std::endl;
    return true;
  }
  bool collideObstacle(const State &s, const Location &obstacle) {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - s.x;
    obs(0, 1) = obstacle.y - s.y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, s.rot);
    if (rotated_obs(0, 0) > -LB - Constants::obsRadius &&
        rotated_obs(0, 0) < LF + Constants::obsRadius &&
        rotated_obs(0, 1) > -width / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 1) < width / 2.0 + Constants::obsRadius)
      return true;
    return false;
  }
  double safetyRadius() { return sqrt(pow(width / 2.0, 2) + pow(LF, 2)); }

 private:
  float LB = Constants::ackerLB;
  float LF = Constants::ackerLF;
  float width = Constants::ackerWidth;
  float r = Constants::r;
  float deltat = Constants::deltat;
  std::vector<double> dyaw = {0, deltat, -deltat, 0, -deltat, deltat};
  std::vector<double> dx = {r * deltat, r *sin(deltat),  r *sin(deltat),
                            -r *deltat, -r *sin(deltat), -r *sin(deltat)};
  std::vector<double> dy = {0, -r *(1 - cos(deltat)), r *(1 - cos(deltat)),
                            0, -r *(1 - cos(deltat)), r *(1 - cos(deltat))};
};
