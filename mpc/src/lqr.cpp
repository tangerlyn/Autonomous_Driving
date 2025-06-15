#include <vector>
#include <cmath>
#include <limits>
#include <tuple>
#include <Eigen/Dense>

#include "lqr.hpp"

using namespace Eigen;

static constexpr double dt = 0.1;
static constexpr double L = 0.5;
static constexpr double MAX_STEER = M_PI / 180.0 * 45.0;
static const Matrix4d Q = Matrix4d::Identity();
static const Matrix<double,1,1> R = Matrix<double,1,1>::Identity();

inline double pi2pi(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

Matrix4d solve_dare(const Matrix4d& A, const Matrix<double,4,1>& B) {
    Matrix4d X = Q;
    for (int i = 0; i < 150; ++i) {
        Matrix<double,1,1> tmp = R + B.transpose() * X * B;
        Matrix4d Xn = A.transpose() * X * A - A.transpose() * X * B * tmp.inverse() * B.transpose() * X * A + Q;
        if ((Xn - X).cwiseAbs().maxCoeff() < 1e-2) {
            return Xn;
        }
        X = Xn;
    }
    return X;
}

Matrix<double,1,4> dlqr(const Matrix4d& A, const Matrix<double,4,1>& B) {
    Matrix4d X = solve_dare(A, B);
    return (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
}

std::pair<int,double> calcNearest(const std::vector<double>& cx, const std::vector<double>& cy, const std::vector<double>& cyaw, double x, double y) {
    int ind = 0;
    double min_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < cx.size(); ++i) {
        double dx = cx[i] - x;
        double dy = cy[i] - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
            min_d2 = d2;
            ind = i;
        }
    }
    double e = std::sqrt(min_d2);
    double angle = pi2pi(cyaw[ind] - std::atan2(cy[ind] - y, cx[ind] - x));
    if (angle < 0) e = -e;
    return {ind, e};
}

std::tuple<double,int,double,double> lqrSteeringControl(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    double x, double y, double yaw, double v,
    double pe, double pth_e) {
    auto [ind, e] = calcNearest(cx, cy, cyaw, x, y);
    double k = ck[ind];
    double th_e = pi2pi(yaw - cyaw[ind]);

    Matrix4d A = Matrix4d::Zero();
    A(0,0) = 1.0; A(0,1) = dt;
    A(1,2) = v;
    A(2,2) = 1.0; A(2,3) = dt;
    A(3,3) = 1.0;

    Matrix<double,4,1> B = Matrix<double,4,1>::Zero();
    B(3,0) = v / L;

    auto Kmat = dlqr(A, B);

    Vector4d x_err;
    x_err << e,
             (e - pe) / dt,
             th_e,
             (th_e - pth_e) / dt;

    double ff = std::atan2(L * k, 1.0);
    double fb = pi2pi(- (Kmat * x_err)(0));
    double delta = std::max(-MAX_STEER, std::min(MAX_STEER, ff + fb));

    return {delta, ind, e, th_e};
}
