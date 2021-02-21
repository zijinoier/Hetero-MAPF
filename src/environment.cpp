/**
 * @file environment.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief Environment class header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "checkcollision.cpp"
#include "neighbor.hpp"
#include "planresult.hpp"
#include "state.hpp"

namespace libMultiRobotPlanning {

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

/**
 * @brief  Environment class
 *
 * @tparam Location
 * @tparam State
 * @tparam Action
 * @tparam Cost
 * @tparam Conflict
 * @tparam Constraint
 * @tparam Constraints
 */
template <typename Location, typename State, typename Model, typename Action,
          typename Cost, typename Conflict, typename Constraints>
class Environment {
 public:
  Environment(size_t maxx, size_t maxy, std::unordered_set<Location> obstacles,
              std::multimap<int, std::pair<State, std::shared_ptr<Model>>>
                  dynamic_obstacles,
              std::vector<State> goals,
              std::vector<std::shared_ptr<Model>> models)
      : m_obstacles(std::move(obstacles)),
        m_dynamic_obstacles(std::move(dynamic_obstacles)),
        m_models(std::move(models)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
    m_dimx = (int)maxx / Constants::mapResolution;
    m_dimy = (int)maxy / Constants::mapResolution;
    // std::cout << "env build " << m_dimx << " " << m_dimy << " "
    //           << m_obstacles.size() << std::endl;
    holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
        goals.size(), std::vector<std::vector<double>>(
                          m_dimx, std::vector<double>(m_dimy, 0)));
    m_goals.clear();
    for (const auto &g : goals) {
      if (g.x < 0 || g.x > maxx || g.y < 0 || g.y > maxy) {
        std::cout << "\033[1m\033[31m Goal out of boundary, Fail to build "
                     "environment \033[0m\n";
        return;
      }
      m_goals.emplace_back(
          State(g.x, g.y, Constants::normalizeHeadingRad(g.yaw)));
    }
    updateCostmap();
  }

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  /// High Level Environment functions
  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, double>> &solution,
      Conflict &result) {
    int max_t = 0;
    for (const auto &sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }
    for (int t = 0; t < max_t; ++t) {
      // check drive-drive collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (CheckCollision::getInstance().collideAgents(
                  state1, state2, m_models[i], m_models[j])) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.s1 = state1;
            result.s2 = state2;
            return true;
          }
        }
      }
    }
    return false;
  }

  void createConstraintsFromConflict(
      const Conflict &conflict, std::map<size_t, Constraints> &constraints) {
    Constraints c1;
    c1.addConflict(conflict.time, conflict.s2, conflict.agent2);
    constraints[conflict.agent1] = c1;
    Constraints c2;
    c2.addConflict(conflict.time, conflict.s1, conflict.agent1);
    constraints[conflict.agent2] = c2;
  }

  void onExpandHighLevelNode(int /*cost*/) {
    m_highLevelExpanded++;
    if (m_highLevelExpanded % 50 == 0)
      std::cout << "Now expand " << m_highLevelExpanded
                << " high level nodes.\n";
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  /// Low Level Environment functions
  void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto &c : constraints->constraints) {
      if (CheckCollision::getInstance().collideAgents(m_goals[m_agentIdx], c.s,
                                                      m_models[m_agentIdx],
                                                      m_models[c.agentid])) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, c.time);
      }
    }

    // std::cout << "Setting Lowlevel agent idx:" << agentIdx
    //           << " Constraints:" << constraints->constraints.size()
    //           << "  lastGoalConstraints:" << m_lastGoalConstraint <<
    //           std::endl;
  }

  double admissibleHeuristic(const State &s) {
    // holonomic-with-obstacles heuristic
    double twoDoffset =
        sqrt(pow((s.x - (int)s.x) -
                     (m_goals[m_agentIdx].x - (int)m_goals[m_agentIdx].x),
                 2) +
             pow((s.y - (int)s.y) -
                     (m_goals[m_agentIdx].y - (int)m_goals[m_agentIdx].y),
                 2));
    double twoDCost =
        holonomic_cost_maps[m_agentIdx][(int)s.x / Constants::mapResolution]
                           [(int)s.y / Constants::mapResolution] -
        twoDoffset;
    // std::cout << "holonomic cost:" << twoDCost << std::endl;

    double modelUniqueAdmissible =
        m_models[m_agentIdx]->admissibleHeuristic(s, m_goals[m_agentIdx]);

    return std::max({modelUniqueAdmissible, twoDCost});
  }

  bool isSolution(
      const State &state, double gscore,
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>> &_camefrom) {
    std::vector<std::tuple<State, Action, double>> path;
    if (m_models[m_agentIdx]->isSolution(state, m_goals[m_agentIdx], path) ==
        false)
      return false;

    if (std::get<0>(path.back()).time <= m_lastGoalConstraint) {
      return false;
    }
    for (auto iter = path.begin(); iter != path.end(); iter++)
      if (!stateValid(std::get<0>(*iter))) return false;

    for (auto iter = path.begin() + 1; iter != path.end(); iter++) {
      gscore += std::get<2>(*iter);
      _camefrom.insert(std::make_pair<>(
          std::get<0>(*iter),
          std::make_tuple<>(std::get<0>(*(iter - 1)), std::get<1>(*iter),
                            std::get<2>(*iter), gscore)));
    }

    m_goals[m_agentIdx] = std::get<0>(path.back());
    return true;
  }

  void getNeighbors(const State &s, Action action,
                    std::vector<Neighbor<State, Action, double>> &neighbors) {
    neighbors.clear();
    std::vector<Neighbor<State, Action, double>> modelNeighbours =
        m_models[m_agentIdx]->getNeighbors(s, action);
    for (auto n : modelNeighbours) {
      if (stateValid(n.state)) neighbors.emplace_back(n);
    }
  }

  State getGoal() { return m_goals[m_agentIdx]; }

  uint64_t calcIndex(const State &s) {
    return (uint64_t)s.time * (2 * M_PI / Constants::deltat) *
               (m_dimx / Constants::xyResolution) *
               (m_dimy / Constants::xyResolution) +
           (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                      Constants::yawResolution) *
               (m_dimx / Constants::xyResolution) *
               (m_dimy / Constants::xyResolution) +
           (uint64_t)(s.y / Constants::xyResolution) *
               (m_dimx / Constants::xyResolution) +
           (uint64_t)(s.x / Constants::xyResolution);
  }

  void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  bool startAndGoalValid(const std::vector<State> &m_starts, const size_t iter,
                         const int batchsize) {
    assert(m_goals.size() == m_starts.size());
    for (size_t i = 0; i < m_goals.size(); i++)
      for (size_t j = i + 1; j < m_goals.size(); j++) {
        if (CheckCollision::getInstance().collideAgents(
                m_goals[i], m_goals[j], m_models[i], m_models[j])) {
          std::cout << "ERROR: Goal point of " << i + iter * batchsize << " & "
                    << j + iter * batchsize << " collide!\n";
          return false;
        }
        if (CheckCollision::getInstance().collideAgents(
                m_starts[i], m_starts[j], m_models[i], m_models[j])) {
          std::cout << "ERROR: Start point of " << i + iter * batchsize << " & "
                    << j + iter * batchsize << " collide!\n";
          return false;
        }
      }
    return true;
  }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, double>> &solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State &s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
      return false;

    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      if (CheckCollision::getInstance().collideObstacle(s, *it,
                                                        m_models[m_agentIdx]))
        return false;
      // if (m_models[m_agentIdx]->collideObstacle(s, *it)) return false;
    }

    auto it = m_dynamic_obstacles.equal_range(s.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (CheckCollision::getInstance().collideAgents(
              s, itr->second.first, m_models[m_agentIdx], itr->second.second))
        return false;
    }
    auto itlow = m_dynamic_obstacles.lower_bound(-s.time);
    auto itup = m_dynamic_obstacles.upper_bound(-1);
    for (auto itr = itlow; itr != itup; ++itr)
      if (CheckCollision::getInstance().collideAgents(
              s, itr->second.first, m_models[m_agentIdx], itr->second.second))
        return false;

    for (auto it = m_constraints->constraints.begin();
         it != m_constraints->constraints.end(); it++) {
      if (s.time >= it->time &&
          s.time <= it->time + Constants::constraintWaitTime &&
          CheckCollision::getInstance().collideAgents(
              it->s, s, m_models[it->agentid], m_models[m_agentIdx])) {
        return false;
      }
    }

    return true;
  }

  struct compare_node {
    bool operator()(const std::pair<State, double> &n1,
                    const std::pair<State, double> &n2) const {
      return (n1.second > n2.second);
    }
  };
  void updateCostmap() {
    boost::heap::fibonacci_heap<std::pair<State, double>,
                                boost::heap::compare<compare_node>>
        heap;

    std::set<std::pair<int, int>> temp_obs_set;
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      temp_obs_set.insert(
          std::make_pair((int)it->x / Constants::mapResolution,
                         (int)it->y / Constants::mapResolution));
    }

    for (size_t idx = 0; idx < m_goals.size(); idx++) {
      heap.clear();
      int goal_x = (int)m_goals[idx].x / Constants::mapResolution;
      int goal_y = (int)m_goals[idx].y / Constants::mapResolution;
      heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

      while (!heap.empty()) {
        std::pair<State, double> node = heap.top();
        heap.pop();

        int x = node.first.x;
        int y = node.first.y;
        for (int dx = -1; dx <= 1; dx++)
          for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int new_x = x + dx;
            int new_y = y + dy;
            if (new_x == goal_x && new_y == goal_y) continue;
            if (new_x >= 0 && new_x < m_dimx && new_y >= 0 && new_y < m_dimy &&
                holonomic_cost_maps[idx][new_x][new_y] == 0 &&
                temp_obs_set.find(std::make_pair(new_x, new_y)) ==
                    temp_obs_set.end()) {
              holonomic_cost_maps[idx][new_x][new_y] =
                  holonomic_cost_maps[idx][x][y] +
                  sqrt(pow(dx * Constants::mapResolution, 2) +
                       pow(dy * Constants::mapResolution, 2));
              heap.push(std::make_pair(State(new_x, new_y, 0),
                                       holonomic_cost_maps[idx][new_x][new_y]));
            }
          }
      }
    }
    // for (size_t idx = 0; idx < m_goals.size(); idx++) {
    //   std::cout << "---------Cost Map -------Agent: " << idx
    //             << "------------\n";
    //   for (size_t i = 0; i < m_dimx; i++) {
    //     for (size_t j = 0; j < m_dimy; j++)
    //       std::cout << holonomic_cost_maps[idx][i][j] << "\t";
    //     std::cout << std::endl;
    //   }
    // }
  }

 private:
  int m_dimx;
  int m_dimy;
  std::vector<std::vector<std::vector<double>>> holonomic_cost_maps;
  std::unordered_set<Location> m_obstacles;
  std::multimap<int, std::pair<State, std::shared_ptr<Model>>>
      m_dynamic_obstacles;
  std::vector<State> m_goals;
  std::vector<std::shared_ptr<Model>> m_models;
  std::vector<double> m_vel_limit;
  size_t m_agentIdx;
  const Constraints *m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};  // namespace libMultiRobotPlanning
}  // namespace libMultiRobotPlanning