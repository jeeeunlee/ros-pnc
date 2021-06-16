// #include <my_pnc/MagnetoPnC/ContactSet/ContactSet.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoContactTestPlanner.hpp>
#include <my_geometry/Polytope/Polytope.h>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>
#include <random>


MagnetoContactTestPlanner::MagnetoContactTestPlanner(RobotSystem* robot) {
    my_utils::pretty_constructor(1, "Magneto Contact Test Planner");

    robot_ = robot;
    // *robot_temp_ = *robot; todo later -> copy constructor
    robot_temp_ = new RobotSystem( 6+3*4, robot_->getFileName() );   // robottemp?

    gravity_ << 0.0, 0.0, -9.81;
    mass_ = robot_->getRobotMass();    

    // Fg_ = m*([pcom]xg; g)
    Eigen::Vector3d pcom = robot_->getCoMPosition();
    Fg_ = Eigen::VectorXd::Zero(6);
    Fg_.head(3) = mass_ * _skew(pcom) * gravity_;
    Fg_.tail(3) = mass_ * gravity_;
}

void MagnetoContactTestPlanner::setFullContact(const std::vector<ContactSpec*>& contact_list){
    full_contact_list_ = contact_list;
}

void MagnetoContactTestPlanner::setNewContact(const int& _foot_idx){
    new_contact_idx_ = _foot_idx;
    _setNextfoodIndices(_foot_idx);
}

void MagnetoContactTestPlanner::setContactInfo(const std::map<int, double>& f_pull_max,
                                                const std::map<int, double>& friction_coeff){
    // set pull max (fm)
    f_pull_max_ = f_pull_max;

    // set contact coeffecient
    for(auto &[idx, mu]: friction_coeff ){
        for(auto &contact : full_contact_list_){
            if(idx == ((BodyFramePointContactSpec*)contact)->getLinkIdx()){
                ((BodyFramePointContactSpec*)contact)->setFrictionCoeff(mu);
                ((BodyFramePointContactSpec*)contact)->updateContactSpec(); // update U matrix
            }
        }
    }
}

void MagnetoContactTestPlanner::findMinForcePullOnly(int next_foot_idx_i){
    // case1: friction coeff for new contact is known
    int next_foot_idx = next_step_idices_[next_foot_idx_i]; //MagnetoBodyNode::AL_foot_link;
    std::cout<<"new foot : " << new_contact_idx_ <<
        ", next foot = " << next_foot_idx <<  std::endl;
    std::vector<ContactSpec*> contact_list;
    bool b_new_contact = true;

    _setContactListForComputation(b_new_contact, next_foot_idx, contact_list); // include new contact
    // A = [A1 A2 A3], where Ai = [[pi]xRi; Ri]
    // U = diag(U1 U2 U3)        
    // V = Vrep(U), Uav_ = Hrep(AV)
    _buildAstanceMatrix(contact_list); // update Astance_
    my_utils::pretty_print(Astance_, std::cout, "Astance_");
    _buildUMatrix(contact_list); // update U_
    my_utils::pretty_print(U_, std::cout, "U_");
    _updateConvexHull(); // update Uav_(U_,Astance_), where Uav_=Hrep(AV) & V=vrep(U_)    
    my_utils::pretty_print(Uav_, std::cout, "Uav_");

    // u1,u2,u3 = U_av*A
    int nc = contact_list.size();    
    Eigen::MatrixXd UavA = Uav_*Astance_; //mx(3*nc)
    int rowsize = UavA.rows();
    Eigen::MatrixXd u_1 = UavA.block(0,0, rowsize, 3);
    Eigen::MatrixXd u_23 = UavA.block(0,3, rowsize, 3*(nc-1));
    Eigen::VectorXd fm_23 = Eigen::VectorXd::Zero(3*(nc-1));
    int tmp_idx;
    int pull_idx=2; // assume z direction
    for(int i(1);i<nc;++i){
        tmp_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
        if(f_pull_max_.find(tmp_idx)!=f_pull_max_.end())
            fm_23(3*(i-1)+pull_idx) = f_pull_max_[tmp_idx];
    }
    Eigen::VectorXd rhs_vec = -Uav_*Fg_ - u_23*fm_23;
    my_utils::pretty_print(u_23, std::cout, "u_23");
    my_utils::pretty_print(fm_23, std::cout, "fm_23");
    std::cout<<" findMinForcePullOnly : u1 fm1 >= rhs_vec " << std::endl;
    my_utils::pretty_print(u_1, std::cout, "u_1");
    my_utils::pretty_print(rhs_vec, std::cout, "rhs_vec");

    // check max
    double _eps = 1e-3;
    std::vector<double> fpull_min;
    std::vector<double> fpull_max;
    Eigen::VectorXd u_1_vec = u_1.col(2);
    for(int i(0); i<u_1_vec.size(); ++i){
        if(u_1_vec(i) > _eps){
            fpull_min.push_back( rhs_vec(i)/u_1_vec(i) );
        }else if(u_1_vec(i) < -_eps){
            fpull_max.push_back( rhs_vec(i)/u_1_vec(i) );
        }
    } // fpull_min < fpull < fpull_max
    // double min = *min_element(fpull_min.begin(), fpull_min.end());
    std::cout <<"fpull_max size: " << fpull_max.size() << ", fpull_min size: "<<fpull_min.size()<<std::endl;
    double min = *min_element(fpull_max.begin(), fpull_max.end());
    std::cout<<" min value: " << min << std::endl;
    

    // u1 fm1 <= Uav_*Fg_ - u23*fm23 = rhs_vec
}

void MagnetoContactTestPlanner::findMinForceAll(int next_foot_idx_i){
    // case2: friction coeff for new contact is unknown
    int next_foot_idx = next_step_idices_[next_foot_idx_i];
    std::cout<<"new foot : " << new_contact_idx_ <<
        ", next foot = " << next_foot_idx <<  std::endl;
    std::vector<ContactSpec*> contact_list;
    bool b_new_contact = true;
    _setContactListForComputation(b_new_contact, next_foot_idx, contact_list);
    
    double alpha = 0.01; // weight ratio b/w fx and fz, fz = alpha*fx
    double w1(1.0), w23(1.0); // assume new contact force is less minimized
    Eigen::VectorXd Winv1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Winv23 = Eigen::VectorXd::Zero(3);
    Winv1 << 1./w1, 1./w1, 1./w1/alpha;
    Winv23 << 1./w23, 1./w23, 1./w23/alpha;
    
    int link_idx, n_contact =  contact_list.size();
    Eigen::MatrixXd WinvAT = Eigen::MatrixXd::Zero(3*n_contact, 6);
    Eigen::MatrixXd AWinvAT = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd AWinvATinv = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd Ai;
    
    for(int i(0); i<n_contact; ++i) {
        link_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
        Ai = _getAiMatrix(link_idx);
        if(link_idx == new_contact_idx_) {
            WinvAT.block(3*i,0, 3, 6) = Winv1.asDiagonal() * Ai.transpose();
            AWinvAT = AWinvAT + Ai *  Winv1.asDiagonal() * Ai.transpose();

            // Eigen::MatrixXd winvd = Winv1.asDiagonal();
            // my_utils::pretty_print(winvd, std::cout, "Winv1");
            // my_utils::pretty_print(Ai, std::cout, "Ai");
        } else {
            WinvAT.block(3*i,0, 3, 6) = Winv23.asDiagonal() * Ai.transpose();
            AWinvAT = AWinvAT + Ai *  Winv23.asDiagonal() * Ai.transpose();

            // Eigen::MatrixXd winvd = Winv23.asDiagonal();
            // my_utils::pretty_print(winvd, std::cout, "Winv23");
            // my_utils::pretty_print(Ai, std::cout, "Ai");
        }
    }
    AWinvATinv = _pinv(AWinvAT);

    // my_utils::pretty_print(WinvAT, std::cout, "WinvAT");
    // my_utils::pretty_print(AWinvATinv, std::cout, "AWinvATinv");

    Eigen::VectorXd fc = - WinvAT * AWinvATinv * Fg_;
    my_utils::pretty_print(fc, std::cout, "fc");
}

// void MagnetoContactTestPlanner::findMinForceAll(int next_foot_idx_i){
//     // case2: friction coeff for new contact is unknown
//     int next_foot_idx = next_step_idices_[next_foot_idx_i];
//     std::cout<<"new foot : " << new_contact_idx_ <<
//         ", next foot = " << next_foot_idx <<  std::endl;
//     std::vector<ContactSpec*> contact_list;
//     bool b_new_contact = false;

//     _setContactListForComputation(b_new_contact, next_foot_idx, contact_list); 
//     Eigen::MatrixXd A1 = _getAiMatrix(new_contact_idx_);
//     // A = [A2 A3], where Ai = [[pi]xRi; Ri]
//     // U = diag(U1 U2 U3)        
//     // V = Vrep(U), Uav_ = Hrep(AV)
//     _buildAstanceMatrix(contact_list); // update Astance_
//     my_utils::pretty_print(Astance_, std::cout, "Astance_");
//     _buildUMatrix(contact_list); // update U_
//     my_utils::pretty_print(U_, std::cout, "U_");
//     _updateConvexHull(); // update Uav_(U_,Astance_), where Uav_=Hrep(AV) & V=vrep(U_)    
//     my_utils::pretty_print(Uav_, std::cout, "Uav_");

//     // u1,u2,u3 = U_av*A
//     int nc = contact_list.size();    
//     Eigen::MatrixXd UavA1 = Uav_*A1; //mx(3*nc)
//     Eigen::MatrixXd UavA = Uav_*Astance_; //mx(3*nc)
//     int rowsize = UavA.rows();
//     Eigen::VectorXd fm_23 = Eigen::VectorXd::Zero(3*nc);
//     int tmp_idx;
//     int pull_idx=2; // assume z direction
//     for(int i(0);i<nc;++i){
//         tmp_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
//         std::cout<<"tmp_idx = " << tmp_idx << std::endl;
//         if(f_pull_max_.find(tmp_idx)!=f_pull_max_.end())
//             fm_23(3*(i)+pull_idx) = f_pull_max_[tmp_idx];
//     }
//     Eigen::VectorXd rhs_vec = -Uav_*Fg_ - UavA*fm_23;
//     my_utils::pretty_print(UavA, std::cout, "UavA");
//     my_utils::pretty_print(fm_23, std::cout, "fm_23");
    
//     std::cout<<" findMinForceAll : UavA1 fm1 >= rhs_vec " << std::endl;
//     my_utils::pretty_print(UavA1, std::cout, "UavA1");
//     my_utils::pretty_print(rhs_vec, std::cout, "rhs_vec");

//     double _eps = 1e-3;
//     double coeffz;
//     std::vector<Eigen::VectorXd> abd_min;
//     std::vector<Eigen::VectorXd> abd_max;
//     Eigen::VectorXd abd_tmp = Eigen::VectorXd::Zero(3);
//     for(int i(0); i<UavA1.rows(); ++i){
//         coeffz = UavA1(i,2);
//         std::cout << "i= " << i << ", coeffz= " << coeffz << std::endl;
//         if(coeffz > _eps){
//             abd_tmp << UavA1(i,0)/coeffz, UavA1(i,1)/coeffz, rhs_vec(i)/coeffz;            
//             // std::cout<<"abd_min : " << std::endl;
//             // std::cout << abd_tmp.transpose();
//             abd_min.push_back(abd_tmp);
//         }else if(coeffz < -_eps){
//             abd_tmp << UavA1(i,0)/coeffz, UavA1(i,1)/coeffz, rhs_vec(i)/coeffz;            
//             // std::cout<<"abd_max : " << std::endl;
//             // std::cout << abd_tmp.transpose();
//             abd_max.push_back(abd_tmp);
//         }        
//     }

//     std::cout<<"abd_min : " << std::endl;
//     for(auto &abd : abd_min){
//         std::cout << abd(0) << ", " <<  abd(1) << ", " <<  abd(2) << std::endl ;
//     }
//     std::cout<<"abd_max : " << std::endl;
//     for(auto &abd : abd_max){
//         std::cout << abd(0) << ", " <<  abd(1) << ", " <<  abd(2) << std::endl ;
//     }
// }
        
// protected ---------------------------------------------------------------------------------------

void MagnetoContactTestPlanner::_setContactListForComputation(bool b_new_contact, 
                                                              const int& next_foot_idx,
                                                std::vector<ContactSpec*> & contact_list) {

    contact_list.clear();
    int temp_idx;
    // new contact comes first in contact_list
    if(b_new_contact){
        for(auto &contact : full_contact_list_){
        temp_idx = ((BodyFramePointContactSpec*)contact)->getLinkIdx();
        if( temp_idx == new_contact_idx_ )
            contact_list.push_back(contact);
        }
    }
    // rest of contacts, exclude both nextfoot, newfoot
    for(auto &contact : full_contact_list_){
        temp_idx = ((BodyFramePointContactSpec*)contact)->getLinkIdx();
        if( temp_idx != next_foot_idx && temp_idx!=new_contact_idx_ )
            contact_list.push_back(contact);
    }
}

Eigen::MatrixXd MagnetoContactTestPlanner::_getAiMatrix(int contact_link_idx) {
    Eigen::MatrixXd Mat_AdT = Eigen::MatrixXd::Zero(6,3);
    std::cout<<"_buildAstanceMatrix: link_idx : " <<contact_link_idx << std::endl;
    Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(contact_link_idx).linear();
    Eigen::VectorXd p_wb = robot_->getBodyNodeIsometry(contact_link_idx).translation();
    
    Eigen::MatrixXd p_skew = _skew(p_wb);
            
    Mat_AdT.block(0,0,3,3) = p_skew*R_wb;
    Mat_AdT.block(3,0,3,3) = R_wb; 

    return Mat_AdT;
}

void MagnetoContactTestPlanner::_buildAstanceMatrix(const std::vector<ContactSpec*> & contact_list) {
    int n_contact =  contact_list.size();
    Astance_ = Eigen::MatrixXd::Zero(6, 3*n_contact);
    int link_idx;
    for(int i(0); i<n_contact; ++i) {        
        link_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();    
        Astance_.block(0,3*i,6,3) = _getAiMatrix(link_idx);
    }    
}

void MagnetoContactTestPlanner::_buildUMatrix(const std::vector<ContactSpec*> & contact_list) {
    int dim_new_rf, dim_new_rf_cstr;
    Eigen::MatrixXd U_i;
    int dim_rows, dim_cols, dim_rows_i, dim_cols_i;
    int n_contact = contact_list.size();

    
    contact_list[0]->getRFConstraintMtx(U_);
    _ineqMat2Uf(U_);
    dim_rows = U_.rows();
    dim_cols = U_.cols();
    link_idx_.clear();
    int link_idx;
 
    for(int i(1); i<n_contact; ++i) {

        link_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
        link_idx_.push_back(link_idx);

        contact_list[i]->getRFConstraintMtx(U_i);
        _ineqMat2Uf(U_i);
        dim_rows_i = U_i.rows();
        dim_cols_i = U_i.cols();

        U_.conservativeResize(dim_rows + dim_rows_i, dim_cols + dim_cols_i);
        U_.block(0, dim_cols, dim_rows, dim_cols_i).setZero();
        U_.block(dim_rows, 0, dim_rows_i, dim_cols).setZero();
        U_.block(dim_rows, dim_cols, dim_rows_i, dim_cols_i) = U_i;

        dim_rows += dim_rows_i;
        dim_cols += dim_cols_i;
    }
}

void MagnetoContactTestPlanner::_updateConvexHull()
{
    // check empty
    if(U_.rows() == 0 || U_.cols() == 0)
        return;

    // double description convex hull
    Polyhedron poly_1, poly_2;
    // 1. face(Uf_)->span(Uf_S=Vf_)
    bool b_poly_U = poly_1.setHrep(U_, Eigen::VectorXd::Zero(U_.rows()));
    Eigen::MatrixXd V = (poly_1.vrep().first).transpose();
    my_utils::pretty_print(V, std::cout, "V");

    // 2. span(A_stance * Vf_)->face(U_st)
    // poly.vrep() return VrepXd:(first:Matrix V, second:Vector index(0:Ray, 1:vertex)) 
    Eigen::MatrixXd Vst = Astance_ * V;
    bool b_poly_Vst = poly_2.setVrep( Vst.transpose(), Eigen::VectorXd::Zero( Vst.cols()) );
    Uav_ = poly_2.hrep().first;
}

void MagnetoContactTestPlanner::_setNextfoodIndices(const int& next_foot_idx){
    next_step_idices_.clear();
    int temp_idx=0;
    for(auto &contact : full_contact_list_){
        temp_idx = ((BodyFramePointContactSpec*)contact)->getLinkIdx();
        if ( temp_idx != next_foot_idx )
            next_step_idices_.push_back(temp_idx);
    }
    //
} 

void MagnetoContactTestPlanner::_ineqMat2Uf(Eigen::MatrixXd &Uf) {
    Uf = -Uf.topLeftCorner(Uf.rows()-1, Uf.cols());
}
 

Eigen::MatrixXd MagnetoContactTestPlanner::_null(const Eigen::MatrixXd& input)
{
    Eigen::MatrixXd ret =  my_utils::getNullSpace(input, 0.0001);
    return ret;
}

Eigen::MatrixXd MagnetoContactTestPlanner::_pinv(const Eigen::MatrixXd& input)
{
    Eigen::MatrixXd ret;
    my_utils::pseudoInverse(input, 0.0001, ret);
    return ret;
}

Eigen::MatrixXd MagnetoContactTestPlanner::_skew(const Eigen::Vector3d& vec) {
    Eigen::MatrixXd skewoutput;
    skewoutput = Eigen::MatrixXd::Zero(3,3);

    // [[ 0, -3,  2],
    // [ 3,  0, -1],
    // [-2,  1,  0]]
    skewoutput <<  0.0,     -vec(2),    vec(1),
                vec(2),     0.0    ,    -vec(0),
                -vec(1),    vec(0) ,    0.0;
    return skewoutput;
}