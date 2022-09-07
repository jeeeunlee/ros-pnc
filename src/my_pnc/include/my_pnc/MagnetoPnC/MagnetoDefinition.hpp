#pragma once
#include <string>

namespace Magneto {
constexpr int n_bodynode = 48;
constexpr int n_leg = 6;
constexpr int n_leg_pdof = 3;
constexpr int n_leg_adof = 3;
constexpr int n_vdof = 6 + n_leg*n_leg_pdof;
constexpr int n_adof = n_leg*n_leg_adof;
constexpr int n_dof = n_vdof + n_adof;
constexpr int idx_vdof [n_vdof] = {0,1,2,3,4,5, 9,10,11, 15,16,17, 21,22,23, 27,28,29, 33,34,35, 39,40,41};
constexpr int idx_adof [n_adof] = {6,7,8, 12,13,14, 18,19,20, 24,25,26, 30,31,32, 36,37,38};
constexpr int idx_al_pdof [n_leg_pdof] = {9,10,11};
constexpr int idx_ar_pdof [n_leg_pdof] = {15,16,17};
constexpr int idx_bl_pdof [n_leg_pdof] = {21,22,23};
constexpr int idx_br_pdof [n_leg_pdof] = {27,28,29};
constexpr int idx_cl_pdof [n_leg_pdof] = {33,34,35};
constexpr int idx_cr_pdof [n_leg_pdof] = {39,40,41};
}  // namespace Magneto

namespace MagnetoFoot {
const std::string Names[6] = {
    "AL", "AR", "BL", "BR", "CL", "CR" };

const std::string NamesLower[6] = {
    "al", "ar", "bl", "br", "cl", "cr" };

constexpr int LinkIdx[6] = {
    12, // AL_foot_link
    19, // AR_foot_link
    26, // BL_foot_link
    33, // BR_foot_link
    40, // CL_foot_link
    47 // CR_foot_link
};

constexpr int AL = 0;
constexpr int AR = 1;
constexpr int BL = 2;
constexpr int BR = 3;
constexpr int CL = 4;
constexpr int CR = 5;

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
constexpr int CL_coxa_link = 34;
constexpr int CL_femur_link = 35;
constexpr int CL_tibia_link = 36;
constexpr int CL_foot_link_1 = 37;
constexpr int CL_foot_link_2 = 38;
constexpr int CL_foot_link_3 = 39;
constexpr int CL_foot_link = 40;
constexpr int CR_coxa_link = 41;
constexpr int CR_femur_link = 42;
constexpr int CR_tibia_link = 43;
constexpr int CR_foot_link_1 = 44;
constexpr int CR_foot_link_2 = 45;
constexpr int CR_foot_link_3 = 46;
constexpr int CR_foot_link = 47;
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
constexpr int CL_coxa_joint = 30;
constexpr int CL_femur_joint = 31;
constexpr int CL_tibia_joint = 32;
constexpr int CL_foot_joint_1 = 33;
constexpr int CL_foot_joint_2 = 34;
constexpr int CL_foot_joint_3 = 35;
constexpr int CR_coxa_joint = 36;
constexpr int CR_femur_joint = 37;
constexpr int CR_tibia_joint = 38;
constexpr int CR_foot_joint_1 = 39;
constexpr int CR_foot_joint_2 = 40;
constexpr int CR_foot_joint_3 = 41;
}  // namespace MagnetoDoF




namespace MagnetoAux {
constexpr double servo_rate = 0.001;
}
