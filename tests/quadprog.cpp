#include <Eigen/Core>
#include <quadprog/QuadProg++.h> 
#include <iostream>

int main(){
    Eigen::MatrixXd Ae(2,2);
    Ae << 2,0.5,0.5,1;
    Eigen::VectorXd be(2);
    be << 0,1;

    Eigen::MatrixXd CE(1,2);
    CE << 1,1;
    Eigen::VectorXd ce0(1);
    ce0 << -1;

    Eigen::MatrixXd CI(2,2);
    CI << 1,0,0,1;
    Eigen::VectorXd ci0(2);
    ci0 << 0,0;

    Eigen::VectorXd xsol(2);
    double cost;
    cost = QuadProgPP::solve_quadprog(Ae, be, CE, ce0, CI, ci0, xsol);

    std::cout << cost << std::endl;
    std::cout << xsol << std::endl;
    return 1;
}

