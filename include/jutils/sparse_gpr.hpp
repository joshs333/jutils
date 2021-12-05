#ifndef JUTILS_SPARSE_GPR_HPP_
#define JUTILS_SPARSE_GPR_HPP_

#define JUTILS_SPARSE_GPR_DEBUG_LOGGING
#include <jutils/logging.hpp>
#ifdef JUTILS_SPARSE_GPR_DEBUG_LOGGING
    #define GPR_DEBUG(...) JLOGN_NOTE("gpr", ##__VA_ARGS__)
#else
    #define GPR_DEBUG(...)
#endif

#include <jutils/kdtrie.hpp>
#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#include <iostream>

namespace jutils {

std::string string_format_matrix(const Eigen::MatrixXd& point) {
    std::string result = "(";
    for(uint32_t i = 0; i < point.cols(); ++i) {
        if(i != 0)
            result += ", ";
        result += string_format("%3.2f", point(0,i));
    }
    return result + ")";
}

struct SquaredExponentialKernel {
    static double k(const Eigen::MatrixXd& x1, const Eigen::MatrixXd& x2) {
        return std::exp(-((x1 - x2) * (x1 - x2).transpose())(0,0));
    }
};

class GPRDataPoint {
public:
    typedef uint64_t ID;
    typedef std::shared_ptr<GPRDataPoint> Ptr;

    /**
        * @brief templated function to construct a GPRDataPoint::Ptr
        **/
    template<typename ... Args>
    static Ptr newPtr(Args ... args) {
        return std::make_shared<GPRDataPoint>(args ...);
    }

    GPRDataPoint(
        ID point_id,
        const Eigen::MatrixXd& x_data_point,
        const Eigen::MatrixXd& y_data_point
    ):
        point_id_(point_id),
        x_data_point_(x_data_point),
        y_data_point_(y_data_point)
    {
        GPR_DEBUG("New Data Point(%lu, %s, %s)",
            point_id,
            string_format_matrix(x_data_point).c_str(),
            string_format_matrix(y_data_point).c_str()
        );
    }

    template<typename KernelFunc = SquaredExponentialKernel>
    void link(ID point_id, const Eigen::MatrixXd& point) {
        auto k = KernelFunc::k(point, x_data_point_);
        GPR_DEBUG("  GRPDataPoint(%lu).link(%lu, %s)",
            point_id_, point_id,
            string_format_matrix(point).c_str()
        );
        kernel_values_[point_id] = k;
    }

    bool link_value(const ID& point_id, double& value) {
        auto it = kernel_values_.find(point_id);
        if(it == kernel_values_.end()) {
            return false;
        }
        value = it->second;
        return true;
    }

    const ID& id() const {
        return point_id_;
    }

    const Eigen::MatrixXd& x_data() const {
        return x_data_point_;
    }

    const Eigen::MatrixXd& y_data() const {
        return y_data_point_;
    }

private:
    //! Each data point in a gaussian process has a unique ID
    ID point_id_;
    //! The actual data point (x values)
    Eigen::MatrixXd x_data_point_;
    //! The actual data point (y values)
    Eigen::MatrixXd y_data_point_;
    //! A mapping of kernel values for datapoints within some distance
    std::unordered_map<ID, double> kernel_values_;

}; /* class GPRDataPoint */

template<typename KernelFunc = SquaredExponentialKernel>
class GPR {
public:
    typedef KDTrie<GPRDataPoint::Ptr> Trie;

    GPR(
        uint8_t x_dim,
        uint8_t y_dim,
        double inference_distance = 10.0
    ):
        x_dim_(x_dim),
        y_dim_(y_dim),
        inference_distance_(inference_distance),
        current_id_(0),
        data_trie_(x_dim, 0.01)
    {}

    void insert(
        const std::vector<double>& x_data,
        const std::vector<double>& y_data
    ) {
        Eigen::MatrixXd eig_x_data(1, x_data.size());
        Eigen::MatrixXd eig_y_data(1, y_data.size());
        for(std::size_t i = 0; i < x_data.size(); ++i) eig_x_data(0,i) = x_data[i];
        for(std::size_t i = 0; i < y_data.size(); ++i) eig_y_data(0,i) = y_data[i];
        insert(x_data, y_data);
    }

    void insert(
        const Eigen::MatrixXd& x_data,
        const Eigen::MatrixXd& y_data
    ) {
        if(x_data.cols() != x_dim_ || x_data.rows() != 1) throw std::runtime_error("malformed x_data");
        if(y_data.cols() != y_dim_ || y_data.rows() != 1) throw std::runtime_error("malformed y_data");
        std::vector<double> x_data_vec(x_data.data(), x_data.data() + x_data.cols());

        auto new_point = GPRDataPoint::newPtr(current_id_, x_data, y_data);
        data_points_[current_id_] = new_point;

        Trie::NearAssociates near_data_points;
        data_trie_.nn(near_data_points, x_data_vec, inference_distance_ * 2.1);

        for(const auto& ndp : near_data_points) {
            ndp.associate->link<KernelFunc>(current_id_, x_data);
            new_point->link<KernelFunc>(ndp.associate->id(), ndp.associate->x_data());
        }
        data_trie_.insert(x_data_vec, new_point);
        ++current_id_;
    }

    bool regress(
        const std::vector<double>& x_data,
        std::vector<double>& y_data
    ) {
        Eigen::MatrixXd eig_x_data(1, x_data.size());
        Eigen::MatrixXd eig_y_data(1, y_dim_);
        for(std::size_t i = 0; i < x_data.size(); ++i) eig_x_data(0,i) = x_data[i];
        if(regress(eig_x_data, eig_y_data)) {
            y_data.resize(y_dim_);
            for(std::size_t i = 0; i < y_dim_; ++i) y_data[i] = eig_y_data(0,i);
            return true;
        }
        return false;
    }

    bool regress(
        const Eigen::MatrixXd& x_data,
        Eigen::MatrixXd& y_data
    ) {
        if(x_data.cols() != x_dim_ || x_data.rows() != 1) throw std::runtime_error("malformed x_data");
        if(y_data.cols() != y_dim_ || y_data.rows() != 1) throw std::runtime_error("malformed y_data");
        std::vector<double> x_data_vec(x_data.data(), x_data.data() + x_data.cols());

        GPR_DEBUG("regress(%s)", string_format_matrix(x_data).c_str());
        Trie::NearAssociates near_data_points;
        data_trie_.nn(near_data_points, x_data_vec, inference_distance_);
        GPR_DEBUG("  found %lu relevant data points", near_data_points.size());

        Eigen::MatrixXd K(near_data_points.size(), near_data_points.size());
        Eigen::MatrixXd K_star(1, near_data_points.size());
        Eigen::MatrixXd Y_data(near_data_points.size(), y_dim_);
        for(std::size_t i = 0; i < near_data_points.size(); ++i) {
            // std::cout << near_data_points[i].associate->x_data() << std::endl;
            K_star(0,i) = KernelFunc::k(x_data, near_data_points[i].associate->x_data());
            Y_data.row(i) = near_data_points[i].associate->y_data();
            K(i,i) = 1.00;
            for(std::size_t j = 0; j < i; ++j) {
                auto j_id = near_data_points[j].associate->id();
                if(!near_data_points[i].associate->link_value(j_id, K(i,j))) {
                    GPR_DEBUG("  link_value computation %lu -> %lu failed :(", j_id, near_data_points[i].associate->id());
                }
            }
        }

        std::cout << "K*" << std::endl << K_star << std::endl;
        std::cout << "K" << std::endl << K << std::endl;
        std::cout << "K-1" << std::endl << K.inverse() << std::endl;
        std::cout << "Y" << std::endl << Y_data << std::endl;

        GPR_DEBUG("  K matrices are generated.");
        y_data = K_star * K.inverse() * Y_data;
        GPR_DEBUG("  inference produced %s", string_format_matrix(y_data).c_str());
        return true;
    }

    const uint8_t& x_dim() const { return x_dim_; }
    const uint8_t& y_dim() const { return y_dim_; }

private:
    // The dimensionality of the x values of the function
    uint8_t x_dim_;
    //! The dimensionality of the y values of the function
    uint8_t y_dim_;
    //! Points within this distance will be used for inference / regression
    double inference_distance_;
    //! Latest ID (to track what new ones should be)
    GPRDataPoint::ID current_id_;
    //! A map of datapoints organized by ID
    std::unordered_map<GPRDataPoint::ID, GPRDataPoint::Ptr> data_points_;
    //! A KDTrie to track what points are releveant to regression
    Trie data_trie_;

}; /* class GPR */


typedef GPR<> BaseGPR;

} /* namespace jutils */

#endif /* JUTILS_SPARSE_GPR_H_ */
