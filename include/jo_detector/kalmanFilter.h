/*
    FILE: kalman_filter.h
    --------------------------------------
    header of kalman_filter velocity estimator
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

namespace joDetector{
    class kalman_filter
    {
        private:
        bool is_initialized;
        MatrixXd states;
        MatrixXd A; // state matrix
        MatrixXd B; // input matrix
        MatrixXd H; // observation matrix
        MatrixXd P; // uncertainty
        MatrixXd Q; // process noise
        MatrixXd R; // observation noise

        public:
        kalman_filter();

        void setup(const MatrixXd& states,
                   const MatrixXd& A,
                   const MatrixXd& B,
                   const MatrixXd& H,
                   const MatrixXd& P,
                   const MatrixXd& Q,
                   const MatrixXd& R);

        void setA(const MatrixXd& A);

        void predict(const MatrixXd& u);

        void estimate(const MatrixXd& z, const MatrixXd& u);

        double output(int state_index);
    };
}

#endif