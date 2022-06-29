#pragma once
#include <string>

namespace Magneto {
constexpr int n_bodynode = 81;
constexpr int n_leg = 4;
constexpr int n_leg_pdof = 3;
constexpr int n_leg_adof = 3;
constexpr int n_vdof = 6 + n_leg*n_leg_pdof;
constexpr int n_adof = n_leg*n_leg_adof;
constexpr int n_dof = n_vdof + n_adof;
constexpr int idx_vdof [n_vdof] = {0,1,2,3,4,5,9,10,11,15,16,17,21,22,23,27,28,29};
constexpr int idx_adof [n_adof] = {6,7,8,12,13,14,18,19,20,24,25,26};
}  // namespace Magneto

namespace MagnetoFoot {
const std::string Names[4] = {
    "AL", "AR", "BL", "BR" };

const std::string NamesLower[4] = {
    "al", "ar", "bl", "br" };

constexpr int LinkIdx[4] = {
    12, // AL_foot_link
    19, // AR_foot_link
    26, // BL_foot_link
    33 // BR_foot_link
};

constexpr int AL = 0;
constexpr int AR = 1;
constexpr int BL = 2;
constexpr int BR = 3;
}// namespace MagnetoFoot

namespace MagnetoBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int base_link = 5;
constexpr int AL_coxa_link = 6;
constexpr int AL_femur_link = 7;
constexpr int AL_tibia_link = 8;
constexpr int AL_foot_link_1 = 9;
constexpr int AL_foot_link_2 = 10;
constexpr int AL_foot_link_3 = 11;
constexpr int AL_foot_link = 12;
constexpr int AR_coxa_link = 13;
constexpr int AR_femur_link = 14;
constexpr int AR_tibia_link = 15;
constexpr int AR_foot_link_1 = 16;
constexpr int AR_foot_link_2 = 17;
constexpr int AR_foot_link_3 = 18;
constexpr int AR_foot_link = 19;
constexpr int BL_coxa_link = 20;
constexpr int BL_femur_link = 21;
constexpr int BL_tibia_link = 22;
constexpr int BL_foot_link_1 = 23;
constexpr int BL_foot_link_2 = 24;
constexpr int BL_foot_link_3 = 25;
constexpr int BL_foot_link = 26;
constexpr int BR_coxa_link = 27;
constexpr int BR_femur_link = 28;
constexpr int BR_tibia_link = 29;
constexpr int BR_foot_link_1 = 30;
constexpr int BR_foot_link_2 = 31;
constexpr int BR_foot_link_3 = 32;
constexpr int BR_foot_link = 33;
}  // namespace MagnetoBodyNode

namespace MagnetoDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int _base_joint = 5;
constexpr int AL_coxa_joint = 6;
constexpr int AL_femur_joint = 7;
constexpr int AL_tibia_joint = 8;
constexpr int AL_foot_joint_1 = 9;
constexpr int AL_foot_joint_2 = 10;
constexpr int AL_foot_joint_3 = 11;
constexpr int AR_coxa_joint = 12;
constexpr int AR_femur_joint = 13;
constexpr int AR_tibia_joint = 14;
constexpr int AR_foot_joint_1 = 15;
constexpr int AR_foot_joint_2 = 16;
constexpr int AR_foot_joint_3 = 17;
constexpr int BL_coxa_joint = 18;
constexpr int BL_femur_joint = 19;
constexpr int BL_tibia_joint = 20;
constexpr int BL_foot_joint_1 = 21;
constexpr int BL_foot_joint_2 = 22;
constexpr int BL_foot_joint_3 = 23;
constexpr int BR_coxa_joint = 24;
constexpr int BR_femur_joint = 25;
constexpr int BR_tibia_joint = 26;
constexpr int BR_foot_joint_1 = 27;
constexpr int BR_foot_joint_2 = 28;
constexpr int BR_foot_joint_3 = 29;
}  // namespace MagnetoDoF

namespace MagnetoAux {
constexpr double servo_rate = 0.01;
}
