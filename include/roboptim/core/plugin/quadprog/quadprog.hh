// Copyright (c) 2013 ISIR CNRS
// Authors: Sovannara Hak
//

#ifndef ROBOPTIM_CORE_QUADPROG_HH
#define ROBOPTIM_CORE_QUADPROG_HH

#include <boost/mpl/vector.hpp>
#include <roboptim/core.hh>
#include <Eigen/Core>

namespace roboptim{
    class QuadprogSolver
        : public Solver< NumericQuadraticFunction, boost::mpl::vector<LinearFunction> > {
        public:
            typedef Solver<NumericQuadraticFunction, boost::mpl::vector<LinearFunction> > parent_t;

            explicit QuadprogSolver (const problem_t& problem) throw ();
            virtual ~QuadprogSolver () throw ();
            virtual void solve () throw ();

            void getConstraints();
            void getFunction();

        private:
            Eigen::MatrixXd G_;
            Eigen::VectorXd g0_;

            Eigen::MatrixXd CE_;
            Eigen::VectorXd ce0_;

            Eigen::MatrixXd CI_;
            Eigen::VectorXd ci0_;

            void appendMatrixTo(Function::matrix_t& A, Function::vector_t& b, Eigen::MatrixXd& Ae, Eigen::VectorXd& be);

    };
    
}

#endif
