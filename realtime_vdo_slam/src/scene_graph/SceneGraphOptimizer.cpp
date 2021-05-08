#include "scene_graph/SceneGraphOptimizer.hpp"



ExpCurveFittingFactor::ExpCurveFittingFactor(minisam::Key key, const Eigen::Vector2d& point,
                            const std::shared_ptr<minisam::LossFunction>& lossfunc)
    :   minisam::Factor(1, std::vector<minisam::Key>{key}, lossfunc), p_(point) {}

std::shared_ptr<minisam::Factor> ExpCurveFittingFactor::copy() const {
    return std::shared_ptr<Factor>(new ExpCurveFittingFactor(*this));
}

Eigen::VectorXd ExpCurveFittingFactor::error(const minisam::Variables& values) const {
    const Eigen::Vector2d& params = values.at<Eigen::Vector2d>(keys()[0]);
    return (Eigen::VectorXd(1) << p_(1) - std::exp(params(0) * p_(0) + params(1)))
        .finished();
}

std::vector<Eigen::MatrixXd> ExpCurveFittingFactor::jacobians(const minisam::Variables& values) const {
    const Eigen::Vector2d& params = values.at<Eigen::Vector2d>(keys()[0]);
    return std::vector<Eigen::MatrixXd>{
        (Eigen::MatrixXd(1, 2) <<
            -p_(0) * std::exp(params(0) * p_(0) + params(1)),
            -std::exp(params(0) * p_(0) + params(1)))
        .finished()};
}