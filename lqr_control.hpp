#include <vector>
#include <tuple>

std::tuple<double, int, double, double> lqrSteeringControl(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    double x, double y, double yaw, double v,
    double pe, double pth_e
);
