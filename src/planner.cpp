/**
 * @file planner.cpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief The implement of planner
 * @date 2021-2-12
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

#include "cl_cbs.hpp"
#include "constraints.hpp"
#include "environment.cpp"
#include "models/ackermann_model.hpp"
#include "models/holonomic_model.hpp"
#include "state.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::CL_CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

using Action = int;

void readAgentConfig() {
  YAML::Node config;
  std::string test(__FILE__);
  boost::replace_all(test, "planner.cpp", "config.yaml");
  try {
    config = YAML::LoadFile(test.c_str());
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
              << test << "\033[0m , Using default params. \n";
  }
  // int car_r = car_config["r"].as<int>();
  Constants::r = config["r"].as<double>();
  Constants::deltat = config["deltat"].as<double>();
  Constants::penaltyTurning = config["penaltyTurning"].as<double>();
  Constants::penaltyReversing = config["penaltyReversing"].as<double>();
  Constants::penaltyCOD = config["penaltyCOD"].as<double>();
  // map resolution
  Constants::mapResolution = config["mapResolution"].as<double>();
  // change to set calcIndex resolution
  Constants::xyResolution = Constants::r * Constants::deltat;
  Constants::yawResolution = Constants::deltat;

  Constants::ackerWidth = config["ackerWidth"].as<double>();
  Constants::ackerLF = config["ackerLF"].as<double>();
  Constants::ackerLB = config["ackerLB"].as<double>();
  // obstacle default radius
  Constants::obsRadius = config["obsRadius"].as<double>();
  // least time to wait for constraint
  Constants::constraintWaitTime = config["constraintWaitTime"].as<double>();

  Constants::testAmount = config["testAmount"].as<int>();

  Constants::dx = {Constants::r * Constants::deltat,
                   Constants::r * sin(Constants::deltat),
                   Constants::r * sin(Constants::deltat),
                   -Constants::r * Constants::deltat,
                   -Constants::r * sin(Constants::deltat),
                   -Constants::r * sin(Constants::deltat)};
  Constants::dy = {0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat)),
                   0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat))};
  Constants::dyaw = {0, Constants::deltat,  -Constants::deltat,
                     0, -Constants::deltat, Constants::deltat};
}

void addModule(std::vector<std::shared_ptr<Model>>& models,
               const YAML::detail::iterator_value& node) {
  const auto& model = node["model"]["type"];
  if (model.as<std::string>() == "Ackermann") {
    double width = (node["model"]["width"])
                       ? node["model"]["width"].as<double>()
                       : Constants::ackerWidth;
    double lf = (node["model"]["lf"]) ? node["model"]["lf"].as<double>()
                                      : Constants::ackerLF;
    double lb = (node["model"]["lb"]) ? node["model"]["lb"].as<double>()
                                      : Constants::ackerLB;
    double r =
        (node["model"]["r"]) ? node["model"]["r"].as<double>() : Constants::r;
    double v = (node["model"]["v"]) ? node["model"]["v"].as<double>() : 1;
    models.emplace_back(std::make_shared<AckermannModel>(lb, lf, width, r, v));
  } else if (model.as<std::string>() == "Holonomic") {
    double width = (node["model"]["width"])
                       ? node["model"]["width"].as<double>()
                       : Constants::holoCarWidth;
    double v = (node["model"]["v"]) ? node["model"]["v"].as<double>() : 1;
    models.emplace_back(std::make_shared<HolonomicModel>(width, v));
  } else {
    std::cout << "\033[1m\033[33mWARNING: Encounter unrecognize model of "
              << node["name"].as<std::string>()
              << ". Using Ackermann Model by deafault.\033[0m \n";
    models.emplace_back(std::make_shared<AckermannModel>());
  }
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  int batchSize;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "batchsize,b", po::value<int>(&batchSize)->default_value(5),
      "batch size for iter");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  readAgentConfig();

  YAML::Node map_config;
  try {
    map_config = YAML::LoadFile(inputFile);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << inputFile
              << "\033[0m \n";
    return 0;
  }
  const auto& dim = map_config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  std::unordered_set<Location> obstacles;
  std::multimap<int, std::pair<State, std::shared_ptr<Model>>>
      dynamic_obstacles;
  std::vector<State> goals;
  std::vector<State> startStates;
  std::vector<std::shared_ptr<Model>> models;
  if (map_config["map"]["obs_radius"])
    Constants::obsRadius = map_config["map"]["obs_radius"].as<double>();
  for (const auto& node : map_config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>()));
  }

  for (const auto& node : map_config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(start[0].as<double>(), start[1].as<double>(),
                                   start[2].as<double>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(State(goal[0].as<double>(), goal[1].as<double>(),
                             goal[2].as<double>()));
    if (node["model"]) {
      addModule(models, node);
    } else {
      std::cout << "\033[1m\033[33mWARNING: Cannot determining model of "
                << node["name"].as<std::string>()
                << ". Using Ackermann Model by deafault.\033[0m \n";
      models.emplace_back(std::make_shared<AckermannModel>());
    }
  }

  std::cout << "Calculating Solution...\n";
  double timer = 0;
  bool success = false;
  std::vector<PlanResult<State, Action, double>> solution;
  for (size_t iter = 0; iter < (double)goals.size() / batchSize; iter++) {
    size_t first = iter * batchSize;
    size_t last = first + batchSize;
    if (last >= goals.size()) last = goals.size();
    std::vector<State> m_goals(goals.begin() + first, goals.begin() + last);
    std::vector<State> m_starts(startStates.begin() + first,
                                startStates.begin() + last);
    std::vector<std::shared_ptr<Model>> m_models(models.begin() + first,
                                                 models.begin() + last);

    Environment<Location, State, Model, Action, double, Conflict, Constraints>
        mapf(dimx, dimy, obstacles, dynamic_obstacles, m_goals, m_models);
    if (!mapf.startAndGoalValid(m_starts, iter, batchSize)) {
      success = false;
      break;
    }
    for (auto goal = goals.begin() + last; goal != goals.end(); goal++) {
      // std::cout << goal - goals.begin() - last << std::endl;
      dynamic_obstacles.insert(
          std::pair<int, std::pair<State, std::shared_ptr<Model>>>(
              -1, std::make_pair(State(goal->x, goal->y, goal->yaw),
                                 models[goal - goals.begin() - last])));
    }
    CL_CBS<State, Action, double, Conflict, Constraints,
           Environment<Location, State, Model, Action, double, Conflict,
                       Constraints>>
        cbsHybrid(mapf);
    std::vector<PlanResult<State, Action, double>> m_solution;
    Timer iterTimer;
    success = cbsHybrid.search(m_starts, m_solution);
    iterTimer.stop();

    if (!success) {
      std::cout << "\033[1m\033[31m No." << iter
                << "iter fail to find a solution \033[0m\n";
      break;
    } else {
      solution.insert(solution.end(), m_solution.begin(), m_solution.end());
      for (size_t a = 0; a < m_solution.size(); ++a) {
        for (const auto& state : m_solution[a].states)
          dynamic_obstacles.insert(
              std::pair<int, std::pair<State, std::shared_ptr<Model>>>(
                  state.first.time,
                  std::make_pair(
                      State(state.first.x, state.first.y, state.first.yaw),
                      m_models[a])));
        State lastState = m_solution[a].states.back().first;
        dynamic_obstacles.insert(
            std::pair<int, std::pair<State, std::shared_ptr<Model>>>(
                -lastState.time,
                std::make_pair(State(lastState.x, lastState.y, lastState.yaw),
                               m_models[a])));
      }
      timer += iterTimer.elapsedSeconds();
      std::cout << "Complete No. " << iter + 1
                << " batch. Runtime:" << iterTimer.elapsedSeconds()
                << " Expand high-level nodes:" << mapf.highLevelExpanded()
                << " Average Low-level-search time:"
                << iterTimer.elapsedSeconds() /
                       (mapf.highLevelExpanded() + m_goals.size())
                << std::endl;
    }
    dynamic_obstacles.erase(-1);
  }

  std::ofstream out;
  out = std::ofstream(outputFile);

  if (success) {
    std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";

    double makespan = 0, flowtime = 0, cost = 0, total_time = 0;
    for (const auto& s : solution) cost += s.cost;

    for (size_t a = 0; a < solution.size(); ++a) {
      // calculate makespan
      double current_makespan = 0;
      current_makespan = 0;
      for (size_t i = 1; i < solution[a].states.size(); i++) {
        current_makespan += sqrt(pow((solution[a].states[i].first.x -
                                      solution[a].states[i - 1].first.x),
                                     2) +
                                 pow((solution[a].states[i].first.y -
                                      solution[a].states[i - 1].first.y),
                                     2));
      }
      flowtime += current_makespan;
      if (current_makespan > makespan) makespan = current_makespan;
      if (solution[a].states.size() > total_time)
        total_time = solution[a].states.size();
    }
    std::cout << " Runtime: " << timer << std::endl
              << " Makespan: " << makespan << std::endl
              << " Flowtime: " << flowtime << std::endl
              << " TotalTime: " << total_time << std::endl
              << " cost: " << cost << std::endl;
    // output to file
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  flowtime: " << flowtime << std::endl;
    out << "  runtime: " << timer << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": "
      //             << solution[a].states[i].first << "->"
      //             << solution[a].actions[i].first
      //             << "(cost: " << solution[a].actions[i].second << ")"
      //             << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": "
      //           << solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      yaw: " << state.first.yaw << std::endl
            << "      t: " << state.first.time << std::endl;
      }
    }
  } else {
    std::cout << "\033[1m\033[31m Fail to find paths \033[0m\n";
  }
}
