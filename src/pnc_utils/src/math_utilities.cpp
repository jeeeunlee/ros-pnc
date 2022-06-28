#include <pnc_utils/math_utilities.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <stdio.h>
#include <cassert>
#include <cmath>


namespace pnc_utils {
void pseudoInverse(Eigen::MatrixXd const & matrix,
                    double sigmaThreshold,
                    Eigen::MatrixXd & invMatrix,
                    Eigen::VectorXd * opt_sigmaOut)    {

    if ((1 == matrix.rows()) && (1 == matrix.cols())) {
        // workaround for Eigen2
        invMatrix.resize(1, 1);
        if (matrix.coeff(0, 0) > sigmaThreshold) {
            invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
        }
        else {
            invMatrix.coeffRef(0, 0) = 0.0;
        }
        if (opt_sigmaOut) {
            opt_sigmaOut->resize(1);
            opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
        }
        return;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // not sure if we need to svd.sort()... probably not
    int const nrows(svd.singularValues().rows());

    Eigen::MatrixXd invS;
    invS = Eigen::MatrixXd::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii) {
        if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
        }
        else{
            // invS.coeffRef(ii, ii) = 0.0;
            invS.coeffRef(ii, ii) = svd.singularValues().coeff(ii) / sigmaThreshold / sigmaThreshold;
            // invS.coeffRef(ii, ii) = 1.0 / sigmaThreshold;
            // printf("sigular value is too small: %f\n", svd.singularValues().coeff(ii));
        }
    }
    invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();

    // Eigen::MatrixXd invS2;
    // invS2 = Eigen::MatrixXd::Zero(nrows, nrows);
    // for (int ii(0); ii < nrows; ++ii) {
    //     invS2.coeffRef(ii, ii) = svd.singularValues().coeff(ii) 
    //     / (svd.singularValues().coeff(ii)*svd.singularValues().coeff(ii) + sigmaThreshold*sigmaThreshold);

    //     printf("%f, ", svd.singularValues().coeff(ii));
    // }
    // printf("\n");

    // invMatrix = svd.matrixV() * invS2 * svd.matrixU().transpose();

    if (opt_sigmaOut) {
        *opt_sigmaOut = svd.singularValues();
    }
}

Eigen::MatrixXd getNullSpace(const Eigen::MatrixXd & J,
                                const double threshold) {

    Eigen::MatrixXd ret(J.cols(), J.cols());
    Eigen::MatrixXd J_pinv;
    pseudoInverse(J, threshold, J_pinv);
    ret = Eigen::MatrixXd::Identity(J.cols(), J.cols()) - J_pinv * J;
    return ret;
}

void weightedInverse(const Eigen::MatrixXd & J,
                        const Eigen::MatrixXd & Winv,
                        Eigen::MatrixXd & Jinv) {
        Eigen::MatrixXd lambda(J* Winv * J.transpose());
        Eigen::MatrixXd lambda_inv;
        pseudoInverse(lambda, 0.0001, lambda_inv);
        Jinv = Winv * J.transpose() * lambda_inv;
}

///////////////////////////////////////////////////

Eigen::MatrixXd skew(const Eigen::Vector3d& w){
    // [[ 0, -3,  2],
    // [ 3,  0, -1],
    // [-2,  1,  0]]
    Eigen::MatrixXd Wx = Eigen::MatrixXd::Zero(3,3);
    Wx <<   0.0,   -w(2),  w(1),
           w(2),     0.0, -w(0),
          -w(1),    w(0),  0.0;
    return Wx;
}

Eigen::MatrixXd hStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    if (a.rows()==0 || a.cols()==0)
        return b;
    if (b.rows()==0 || b.cols()==0)
        return a;
    if (a.rows() != b.rows()) {
        std::cout << "[hStack] Matrix Size is Wrong" << std::endl;
        exit(0);
    }

    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
    ab << a, b;
    return ab;
}

Eigen::MatrixXd hStack(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    if (a.size()==0)
        return b;
    if (b.size()==0)
        return a;
    if (a.size() != b.size()) {
        std::cout << "[hStack] Vector Size is Wrong" << std::endl;
        exit(0);
    }
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.size(), 2);
    ab << a, b;
    return ab;
}

Eigen::MatrixXd vStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    if (a.rows()==0 || a.cols()==0)
        return b;
    if (b.rows()==0 || b.cols()==0)
        return a;
    if (a.cols() != b.cols()) {
        std::cout << "[vStack] Matrix Size is Wrong" << std::endl;
        exit(0);
    }
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
    ab << a, b;
    return ab;
}

Eigen::VectorXd vStack(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    if(a.size()==0) return b;
    if(b.size()==0) return a;
    Eigen::VectorXd ab = Eigen::VectorXd::Zero(a.size()+b.size());    
    // ab.head( a.size() ) = a; 
    // ab.tail( b.size() ) = b;
    ab << a, b;
    return ab;
}

Eigen::MatrixXd dStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    // diagonally stack a,b -> [a 0; 0 b]
    if (a.rows()==0 || a.cols()==0)
        return b;
    if (b.rows()==0 || b.cols()==0)
        return a;
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols() + b.cols());
    ab << a, Eigen::MatrixXd::Zero(a.rows(), b.cols()), 
        Eigen::MatrixXd::Zero(b.rows(), a.cols()), b;
    return ab;
}

Eigen::MatrixXd deleteRow(const Eigen::MatrixXd& a_, int row_) {
    Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(a_.rows() - 1, a_.cols());
    ret.block(0, 0, row_, a_.cols()) = a_.block(0, 0, row_, a_.cols());
    ret.block(row_, 0, ret.rows() - row_, a_.cols()) =
        a_.block(row_ + 1, 0, ret.rows() - row_, a_.cols());
    return ret;
}

///////////////////////////////////////////////////

double smoothing(double ini, double fin, double rat) {
  double ret(0.);
  if (rat < 0) {
    return ini;
  } else if (rat > 1) {
    return fin;
  } else {
    return ini + (fin - ini) * rat;
  }
}

double smooth_changing(double ini, double end, double moving_duration,
                       double curr_time) {
  double ret;
  ret = ini + (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI));
  if (curr_time > moving_duration) {
    ret = end;
  }
  return ret;
}

double smooth_changing_vel(double ini, double end, double moving_duration,
                           double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        sin(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}
double smooth_changing_acc(double ini, double end, double moving_duration,
                           double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}

void getSinusoidTrajectory(double initTime_, const Eigen::VectorXd& midPoint_,
                           const Eigen::VectorXd& amp_,
                           const Eigen::VectorXd& freq_, double evalTime_,
                           Eigen::VectorXd& p_, Eigen::VectorXd& v_,
                           Eigen::VectorXd& a_) {
  int dim = midPoint_.size();
  p_ = Eigen::VectorXd::Zero(dim);
  v_ = Eigen::VectorXd::Zero(dim);
  a_ = Eigen::VectorXd::Zero(dim);
  for (int i = 0; i < dim; ++i) {
    p_[i] = amp_[i] * sin(2 * M_PI * freq_[i] * (evalTime_ - initTime_)) +
            midPoint_[i];
    v_[i] = amp_[i] * 2 * M_PI * freq_[i] *
            cos(2 * M_PI * freq_[i] * (evalTime_ - initTime_));
    a_[i] = -amp_[i] * 2 * M_PI * freq_[i] * 2 * M_PI * freq_[i] *
            sin(2 * M_PI * freq_[i] * (evalTime_ - initTime_));
  }
  double smoothing_dur(1.0);
  if (evalTime_ < (initTime_ + smoothing_dur)) {
    for (int i = 0; i < dim; ++i) {
      v_[i] = 0. + v_[i] * (evalTime_ - initTime_) / smoothing_dur;
      a_[i] = 0. + a_[i] * (evalTime_ - initTime_) / smoothing_dur;
    }
  }
}

double bind_half_pi(double ang) {
    if (ang > M_PI / 2) {
        return ang - M_PI;
    }
    if (ang < -M_PI / 2) {
        return ang + M_PI;
    }
    return ang;
}

bool isInBoundingBox(const Eigen::VectorXd& val, const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub) {
    int n = lb.size();
    bool ret(true);
    for (int i = 0; i < n; ++i) {
        if (lb[i] <= val[i] && val[i] <= ub[i]) {
        } else {
            pnc_utils::color_print(myColor::BoldMagneta, "Is not BoundingBox");
            std::cout << i << " th : lb = " << lb[i] << " val = " << val[i]
                      << " ub = " << ub[i] << std::endl;
            ret = false;
        }
    }
    return ret;
}

Eigen::VectorXd eulerIntegration(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& xdot, double dt) {
    Eigen::VectorXd ret = x;
    ret += xdot * dt;
    return ret;
}

Eigen::VectorXd doubleIntegration(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& alpha,
                                  const Eigen::VectorXd& alphad, double dt) {
    Eigen::VectorXd ret = q;
    ret += alpha * dt + alphad * dt * dt * 0.5;
    return ret;
}

double CropValue(double value, double min, double max, std::string source) {
    assert(min < max);
    if (value > max) {
        printf("%s: %f is cropped to %f.\n", source.c_str(), value, max);
        value = max;
    }
    if (value < min) {
        printf("%s: %f is cropped to %f.\n", source.c_str(), value, min);
        value = min;
    }
    return value;
}

Eigen::VectorXd CropVector(Eigen::VectorXd value, Eigen::VectorXd min,
                           Eigen::VectorXd max, std::string source) {
    assert(value.size() = min.size());
    assert(value.size() = max.size());
    int n_data = value.size();

    for (int i = 0; i < n_data; ++i) {
        if (value[i] > max[i]) {
            // printf("%s(%d): %f is cropped to %f\n", source.c_str(), i,
            // value[i], max[i]);
            value[i] = max[i];
        }
        if (value[i] < min[i]) {
            // printf("%s(%d): %f is cropped to %f\n", source.c_str(), i,
            // value[i], min[i]);
            value[i] = min[i];
        }
    }
    return value;
}

Eigen::MatrixXd CropMatrix(Eigen::MatrixXd value, Eigen::MatrixXd min,
                           Eigen::MatrixXd max, std::string source) {
    assert((value.cols() = min.cols()) && (value.cols() = max.cols()));
    assert((value.rows() = min.rows()) && (value.cols() = max.cols()));

    int n_row = value.rows();
    int n_cols = value.cols();

    for (int row_idx = 0; row_idx < n_row; ++row_idx) {
        for (int col_idx = 0; col_idx < n_cols; ++col_idx) {
            if (value(row_idx, col_idx) < min(row_idx, col_idx)) {
                // printf("%s(%d, %d): %f is cropped to %f\n", source.c_str(),
                // row_idx, col_idx, value(row_idx, col_idx), min(row_idx,
                // col_idx));
                value(row_idx, col_idx) = min(row_idx, col_idx);
            }
            if (value(row_idx, col_idx) > max(row_idx, col_idx)) {
                // printf("%s(%d, %d): %f is cropped to %f\n", source.c_str(),
                // row_idx, col_idx, value(row_idx, col_idx), max(row_idx,
                // col_idx));
                value(row_idx, col_idx) = max(row_idx, col_idx);
            }
        }
    }
    return value;
}

Eigen::MatrixXd GetRelativeMatrix(const Eigen::MatrixXd value,
                                  const Eigen::MatrixXd min,
                                  const Eigen::MatrixXd max) {
    assert((value.cols() = min.cols()) && (value.cols() = max.cols()));
    assert((value.rows() = min.rows()) && (value.cols() = max.cols()));

    Eigen::MatrixXd ret = value;
    for (int col_idx = 0; col_idx < value.cols(); ++col_idx) {
        for (int row_idx = 0; row_idx < value.rows(); ++row_idx) {
            double width = max(row_idx, col_idx) - min(row_idx, col_idx);
            double mid = (max(row_idx, col_idx) + min(row_idx, col_idx)) / 2.0;
            ret(row_idx, col_idx) =
                2.0 * (value(row_idx, col_idx) - mid) / width;
        }
    }
    return ret;
}

Eigen::VectorXd GetRelativeVector(const Eigen::VectorXd value,
                                  const Eigen::VectorXd min,
                                  const Eigen::VectorXd max) {
    assert((value.size() = min.size()) && (value.size() = max.size()));
    Eigen::VectorXd ret = value;

    for (int idx = 0; idx < value.size(); ++idx) {
        double width = max(idx) - min(idx);
        double mid = (max(idx) + min(idx)) / 2.0;
        ret(idx) = 2.0 * (value(idx) - mid) / width;
    }
    return ret;
}

}  // namespace pnc_utils
