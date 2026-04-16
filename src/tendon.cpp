#include "tendon.hpp"

std::vector<double>
length_from_softjoint_state(SoftrobotJointState joint_state) {
  const double d = 0.06416;
  std::vector<double> result;
  result.push_back(joint_state.l *
                   (1 - joint_state.kappa * d * std::cos(joint_state.phi)));
  result.push_back(
      joint_state.l *
      (1 - joint_state.kappa * d *
               std::cos(joint_state.phi - 2 * std::numbers::pi / 3.0)));
  result.push_back(
      joint_state.l *
      (1 - joint_state.kappa * d *
               std::cos(joint_state.phi - 4 * std::numbers::pi / 3.0)));
  return result;
};

SoftrobotJointState joint_state_from_length(std::vector<double> lengths) {
  const double d = 0.06416;
  double square_root =
      lengths.at(0) * lengths.at(0) + lengths.at(1) * lengths.at(1) +
      lengths.at(2) * lengths.at(2) - lengths.at(0) * lengths.at(1) -
      lengths.at(1) * lengths.at(2) - lengths.at(2) * lengths.at(0);
  double kappa = 2.0 * std::sqrt(square_root < 0.0 ? 0.0 : square_root) /
                 (d * (lengths.at(0) + lengths.at(1) + lengths.at(2)));
  double phi = std::atan2(std::sqrt(3.0) * (lengths.at(2) + lengths.at(1) -
                                            2 * lengths.at(0)),
                          3 * (lengths.at(1) - lengths.at(2))) -
               std::numbers::pi / 2;
  double l = (lengths.at(0) + lengths.at(1) + lengths.at(2)) / 3;
  return {kappa, phi, l};
};
