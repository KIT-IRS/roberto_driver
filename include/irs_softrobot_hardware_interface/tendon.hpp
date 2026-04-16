#pragma once

#include <cmath>
#include <numbers>
#include <vector>

struct SoftrobotJointState {
  double kappa;
  double phi;
  double l;
};

std::vector<double>
length_from_softjoint_state(SoftrobotJointState joint_state);

SoftrobotJointState joint_state_from_length(std::vector<double>);
