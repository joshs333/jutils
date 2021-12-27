// #define JUTILS_SPARSE_GPR_DEBUG_LOGGING
#include <jutils/sparse_gpr.hpp>
#include <jutils/time.hpp>

#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <chrono>
#include <Eigen/LU>
#include <random>

#ifdef JUTILS_ENABLE_MATPLOTLIB
    #include <matplotlib.h>
    namespace plt = matplotlibcpp;
#endif


int main(int argc, char** argv) {
    (void) argc;
    (void) argv;
    jutils::logging::get_consumer("stdout")->set_level_low(jutils::logging::level::NOTE);


    auto f = [](double x) {
        return std::sin(x);
    };

    const std::size_t num_points = 100;
    Eigen::MatrixXd x_values(num_points, 1);
    Eigen::MatrixXd y_values(num_points, 1);

    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_real_distribution<double>  distr(0, 2*M_PI);

    jutils::BaseGPR gpr(1,1);
    for(std::size_t i = 0; i < num_points; ++i) {
        x_values(i, 0) = distr(generator);
        y_values(i, 0) = f(x_values(i,0));
        gpr.insert(x_values.row(i), y_values.row(i));
    }

    Eigen::MatrixXd y_i(1,1);
    gpr.regress(x_values.row(0), y_i);
    std::cout << y_i << std::endl;
    std::cout << "vs" << std::endl << y_values.row(0) << std::endl;


    const std::size_t plot_points = 1000;
    std::vector<double> x_points;
    std::vector<double> y_points;
    std::vector<double> x_pred_points;
    std::vector<double> y_pred_points;
    double error = 0.0;
    auto s = jutils::Clock::now();
    for(std::size_t i = 3; i < plot_points; ++i) {
        double cur_x = 2*M_PI * static_cast<double>(i) / static_cast<double>(plot_points);
        // double cur_y = std::sin(cur_x);
        double cur_y = f(cur_x);
        std::vector<double> y_val_pred;
        gpr.regress({cur_x}, y_val_pred);
        error += std::sqrt(std::pow(y_val_pred[0] - cur_y, 2));


        x_points.push_back(cur_x);
        y_points.push_back(cur_y);
        if(std::abs(y_val_pred[0]) < 100.) {
            x_pred_points.push_back(cur_x);
            y_pred_points.push_back(y_val_pred[0]);
        }
    }
    auto e = jutils::Clock::now();

    std::cout << "MSE: " << error / x_points.size() << std::endl;
    std::cout << "Mean Inference Time: " << static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count()) / x_points.size() << std::endl;

    #ifdef JUTILS_ENABLE_MATPLOTLIB
        plt::plot(x_points, y_points);
        plt::plot(x_pred_points, y_pred_points);
        plt::show();
    #endif

    // std::cout << "Before :)" << std::endl;
    // auto start = std::chrono::high_resolution_clock::now();
    // auto R = K.inverse();
    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;
    // std::cout << R * K << std::endl;

    return 1;
}