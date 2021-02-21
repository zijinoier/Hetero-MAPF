/**
 * @file state.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief State base class
 * @version 0.1
 * @date 2021-01-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "constants.hpp"

struct Location {
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct State {
  State(double x, double y, double yaw, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {
    rot.resize(2, 2);
    rot(0, 0) = cos(-this->yaw);
    rot(0, 1) = -sin(-this->yaw);
    rot(1, 0) = sin(-this->yaw);
    rot(1, 1) = cos(-this->yaw);
  }

  State() = default;

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;
  boost::numeric::ublas::matrix<double> rot;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std
