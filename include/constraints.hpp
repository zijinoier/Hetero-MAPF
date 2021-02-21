/**
 * @file constraints.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief base class of conflict & constraints
 * @version 0.1
 * @date 2021-01-17
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <iostream>

#include "model.hpp"
#include "state.hpp"

struct Conflict {
  int time;
  size_t agent1;
  size_t agent2;

  State s1;
  State s2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
       << c.s2 << " ]";
    return os;
  }
};

struct Constraint {
  Constraint(int time, State s, size_t agentid)
      : time(time), s(s), agentid(agentid) {}
  Constraint(int time, State s, size_t agentid, std::shared_ptr<Model> model)
      : time(time), s(s), agentid(agentid), model(model) {}
  Constraint() = default;
  int time;
  State s;
  size_t agentid;
  std::shared_ptr<Model> model;

  bool operator<(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) <
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  bool operator==(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) ==
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    return os << "Constraint[" << c.time << "," << c.s << "from " << c.agentid
              << "]";
  }
};

namespace std {
template <>
struct hash<Constraint> {
  size_t operator()(const Constraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.s.x);
    boost::hash_combine(seed, s.s.y);
    boost::hash_combine(seed, s.s.yaw);
    boost::hash_combine(seed, s.agentid);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<Constraint> constraints;

  void addConflict(int time, State s, size_t agentid) {
    constraints.emplace(Constraint(time, s, agentid));
  }

  void add(const Constraints& other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }

  bool overlap(const Constraints& other) {
    for (const auto& c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& cs) {
    for (const auto& c : cs.constraints) {
      os << c << std::endl;
    }
    return os;
  }
};