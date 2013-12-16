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
            typedef parent_t solver_t;
            typedef parent_t::result_t result_t;
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

            Function::size_type n_;

            /// \brief Append the columns of a matrix_t at the end of an Eigen::MatrixXd (resp. vector_t, Eigen::VectorXd)
            /// 
            /// \param A The input matrix_t
            /// \param b the input vector_t
            /// \param Ae The output Eigen::MatrixXd
            /// \param be the output Eigen::VectorXd
            void appendMatrixTo(Function::matrix_t& A, Function::vector_t& b, Eigen::MatrixXd& Ae, Eigen::VectorXd& be);

    };
    
}

#endif
