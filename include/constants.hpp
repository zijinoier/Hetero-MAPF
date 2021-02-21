/**
 * @file constants.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief Class for Constants
 * @version 0.1
 * @date 2021-01-17
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <vector>

namespace Constants {
// [m] --- The minimum turning radius of the vehicle
static float r = 3;
static float deltat = 6.75 * 6 / 180.0 * M_PI;
// [#] --- A movement cost penalty for turning (choosing non straight motion
// primitives)
static float penaltyTurning = 1.5;
// [#] --- A movement cost penalty for reversing (choosing motion primitives >
// 2)
static float penaltyReversing = 2.0;
// [#] --- A movement cost penalty for change of direction (changing from
// primitives < 3 to primitives > 2)
static float penaltyCOD = 2.0;
// map resolution
static float mapResolution = 2.0;
// change to set calcIndex resolution
static float xyResolution = r * deltat;
static float yawResolution = deltat;

// obstacle default radius
static float obsRadius = 1;
// least time to wait for constraint
static int constraintWaitTime = 2;

// Important!
// Holonomic Model
static float holoCarWidth = 2.0;
static float holoStepLength = r * deltat;
// Ackermann Model
// width of car
static float ackerWidth = 2.0;
// distance from rear to vehicle front end
static float ackerLF = 2.0;
// distance from rear to vehicle back end
static float ackerLB = 1.0;

// Test agent number for Random map
static int testAmount = -1;

// R = 3, 6.75 DEG
std::vector<double> dyaw = {0, deltat, -deltat, 0, -deltat, deltat};
std::vector<double> dx = {r * deltat, r* sin(deltat),  r* sin(deltat),
                          -r* deltat, -r* sin(deltat), -r* sin(deltat)};
std::vector<double> dy = {0, -r*(1 - cos(deltat)), r*(1 - cos(deltat)),
                          0, -r*(1 - cos(deltat)), r*(1 - cos(deltat))};

static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}
}  // namespace Constants
