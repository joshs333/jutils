#include <jutils/sparse_gpr.hpp>

#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <chrono>
#include <Eigen/LU>

Eigen::MatrixXd f(Eigen::MatrixXd x) {
    Eigen::MatrixXd res(x.rows(), 2);
    for(int i = 0; i < x.rows(); ++i) {
        res(i,0) = x(i,0) + x(i,1) + x(i,2);
        res(i,1) = x(i,0) * x(i,1) * x(i,2) * 1e-2;
    }
    return res;
}

double k(const Eigen::MatrixXd& x1, const Eigen::MatrixXd& x2) {
    if(x1.rows() == 1)
        return std::exp(-((x1 - x2) * (x1 - x2).transpose())(0,0) * 0.5);
    return std::exp(-((x1 - x2).transpose() * (x1 - x2))(0,0) * 0.5);
}

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;
    jutils::logging::get_consumer("stdout")->set_level_low(jutils::logging::level::NOTE);

    const std::size_t num_points = 2;
    Eigen::MatrixXd x_values(num_points, 3);

    for(std::size_t i = 0; i < num_points; ++i) {
        x_values(i,0) = 0.3 * static_cast<double>(i) - static_cast<double>(num_points)*0.5*0.3; 
        x_values(i,1) = 10. + 0.3 * static_cast<double>(i) - static_cast<double>(num_points)*0.5*0.2; 
        x_values(i,2) = 6. - 0.3 * static_cast<double>(i) - static_cast<double>(num_points)*0.5*0.1;
    }

    auto y_values = f(x_values);

    // Eigen::MatrixXd K(num_points, num_points);
    // for(std::size_t i = 0; i < num_points; ++i) {
    //     for(std::size_t j = 0; j < num_points; ++j) {
    //         K(i,j) = K(j,i) = k(x_values.row(i), x_values.row(j));
    //     }
    // }

    jutils::BaseGPR gpr(3,2);
    for(int i = 0; i < x_values.rows(); ++i) {
        gpr.insert(x_values.row(i), y_values.row(i));
    }

    Eigen::MatrixXd y_i(1,2);
    gpr.regress(x_values.row(0), y_i);
    std::cout << y_i << std::endl;
    std::cout << "vs" << std::endl << y_values.row(0) << std::endl;

    // std::cout << "Before :)" << std::endl;
    // auto start = std::chrono::high_resolution_clock::now();
    // auto R = K.inverse();
    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;
    // std::cout << R * K << std::endl;

    return 1;
}