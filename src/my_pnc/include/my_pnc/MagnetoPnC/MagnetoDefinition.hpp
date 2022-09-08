#pragma once
#include <string>

namespace Magneto {
constexpr int n_bodynode = 69;
constexpr int n_leg = 9;

constexpr int n_leg_pdof = 3;
constexpr int n_leg_adof = 3;
constexpr int n_vdof = 6 + n_leg*n_leg_pdof;
constexpr int n_adof = n_leg*n_leg_adof;
constexpr int n_dof = n_vdof + n_adof;

constexpr int idx_vdof [n_vdof] = {0,1,2,3,4,5, 9,10,11, 15,16,17, 21,22,23, 27,28,29, 33,34,35, 39,40,41, 45,46,47, 51,52,53, 57,58,59};
constexpr int idx_adof [n_adof] = {6,7,8, 12,13,14, 18,19,20, 24,25,26, 30,31,32, 36,37,38, 42,43,44, 48,49,50, 54,55,56};
}  // namespace Magneto

namespace MagnetoFoot {
const std::string Names[9] = {
    "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "A9"  };

const std::string NamesLower[9] = {
    "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9"};

constexpr int LinkIdx[9] = {
    12, // A1_foot_link
    19, // A2_foot_link
    26, // A3_foot_link
    33, // A4_foot_link
    40, // A5_foot_link
    47, // A6_foot_link
    54, // A7_foot_link
    61, // A8_foot_link
    68 // A9_foot_link
};

constexpr int A1 = 0;
constexpr int A2 = 1;
constexpr int A3 = 2;
constexpr int A4 = 3;
constexpr int A5 = 4;
constexpr int A6 = 5;
constexpr int A7 = 6;
constexpr int A8 = 7;
constexpr int A9 = 8;

}// namespace MagnetoFoot

namespace MagnetoBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int base_link = 5;
constexpr int A1_coxa_link = 6;
constexpr int A1_femur_link = 7;
constexpr int A1_tibia_link = 8;
constexpr int A1_foot_link_1 = 9;
constexpr int A1_foot_link_2 = 10;
constexpr int A1_foot_link_3 = 11;
constexpr int A1_foot_link = 12;
constexpr int A2_coxa_link = 13;
constexpr int A2_femur_link = 14;
constexpr int A2_tibia_link = 15;
constexpr int A2_foot_link_1 = 16;
constexpr int A2_foot_link_2 = 17;
constexpr int A2_foot_link_3 = 18;
constexpr int A2_foot_link = 19;
constexpr int A3_coxa_link = 20;
constexpr int A3_femur_link = 21;
constexpr int A3_tibia_link = 22;
constexpr int A3_foot_link_1 = 23;
constexpr int A3_foot_link_2 = 24;
constexpr int A3_foot_link_3 = 25;
constexpr int A3_foot_link = 26;
constexpr int A4_coxa_link = 27;
constexpr int A4_femur_link = 28;
constexpr int A4_tibia_link = 29;
constexpr int A4_foot_link_1 = 30;
constexpr int A4_foot_link_2 = 31;
constexpr int A4_foot_link_3 = 32;
constexpr int A4_foot_link = 33;
constexpr int A5_coxa_link = 34;
constexpr int A5_femur_link = 35;
constexpr int A5_tibia_link = 36;
constexpr int A5_foot_link_1 = 37;
constexpr int A5_foot_link_2 = 38;
constexpr int A5_foot_link_3 = 39;
constexpr int A5_foot_link = 40;
constexpr int A6_coxa_link = 41;
constexpr int A6_femur_link = 42;
constexpr int A6_tibia_link = 43;
constexpr int A6_foot_link_1 = 44;
constexpr int A6_foot_link_2 = 45;
constexpr int A6_foot_link_3 = 46;
constexpr int A6_foot_link = 47;
constexpr int A7_coxa_link = 48;
constexpr int A7_femur_link = 49;
constexpr int A7_tibia_link = 50;
constexpr int A7_foot_link_1 = 51;
constexpr int A7_foot_link_2 = 52;
constexpr int A7_foot_link_3 = 53;
constexpr int A7_foot_link = 54;
constexpr int A8_coxa_link = 55;
constexpr int A8_femur_link = 56;
constexpr int A8_tibia_link = 57;
constexpr int A8_foot_link_1 = 58;
constexpr int A8_foot_link_2 = 59;
constexpr int A8_foot_link_3 = 60;
constexpr int A8_foot_link = 61;
constexpr int A9_coxa_link = 62;
constexpr int A9_femur_link = 63;
constexpr int A9_tibia_link = 64;
constexpr int A9_foot_link_1 = 65;
constexpr int A9_foot_link_2 = 66;
constexpr int A9_foot_link_3 = 67;
constexpr int A9_foot_link = 68;
}  // namespace MagnetoBodyNode

namespace MagnetoDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int _base_joint = 5;
constexpr int A1_coxa_joint = 6;
constexpr int A1_femur_joint = 7;
constexpr int A1_tibia_joint = 8;
constexpr int A1_foot_joint_1 = 9;
constexpr int A1_foot_joint_2 = 10;
constexpr int A1_foot_joint_3 = 11;
constexpr int A2_coxa_joint = 12;
constexpr int A2_femur_joint = 13;
constexpr int A2_tibia_joint = 14;
constexpr int A2_foot_joint_1 = 15;
constexpr int A2_foot_joint_2 = 16;
constexpr int A2_foot_joint_3 = 17;
constexpr int A3_coxa_joint = 18;
constexpr int A3_femur_joint = 19;
constexpr int A3_tibia_joint = 20;
constexpr int A3_foot_joint_1 = 21;
constexpr int A3_foot_joint_2 = 22;
constexpr int A3_foot_joint_3 = 23;
constexpr int A4_coxa_joint = 24;
constexpr int A4_femur_joint = 25;
constexpr int A4_tibia_joint = 26;
constexpr int A4_foot_joint_1 = 27;
constexpr int A4_foot_joint_2 = 28;
constexpr int A4_foot_joint_3 = 29;
constexpr int A5_coxa_joint = 30;
constexpr int A5_femur_joint = 31;
constexpr int A5_tibia_joint = 32;
constexpr int A5_foot_joint_1 = 33;
constexpr int A5_foot_joint_2 = 34;
constexpr int A5_foot_joint_3 = 35;
constexpr int A6_coxa_joint = 36;
constexpr int A6_femur_joint = 37;
constexpr int A6_tibia_joint = 38;
constexpr int A6_foot_joint_1 = 39;
constexpr int A6_foot_joint_2 = 40;
constexpr int A6_foot_joint_3 = 41;
constexpr int A7_coxa_joint = 42;
constexpr int A7_femur_joint = 43;
constexpr int A7_tibia_joint = 44;
constexpr int A7_foot_joint_1 = 45;
constexpr int A7_foot_joint_2 = 46;
constexpr int A7_foot_joint_3 = 47;
constexpr int A8_coxa_joint = 48;
constexpr int A8_femur_joint = 49;
constexpr int A8_tibia_joint = 50;
constexpr int A8_foot_joint_1 = 51;
constexpr int A8_foot_joint_2 = 52;
constexpr int A8_foot_joint_3 = 53;
constexpr int A9_coxa_joint = 54;
constexpr int A9_femur_joint = 55;
constexpr int A9_tibia_joint = 56;
constexpr int A9_foot_joint_1 = 57;
constexpr int A9_foot_joint_2 = 58;
constexpr int A9_foot_joint_3 = 59;
}  // namespace MagnetoDoF




namespace MagnetoAux {
constexpr double servo_rate = 0.001;
}
