// Copyright (c) 2013 ISIR CNRS
// Authors: Sovannara Hak
//

#include <typeinfo>

#include <roboptim/core.hh>
#include "roboptim/core/plugin/quadprog/quadprog.hh"
#include <quadprog/QuadProg++.h>

namespace roboptim{
    QuadprogSolver::QuadprogSolver (const problem_t& pb) throw ()
        : parent_t (pb){

            n_ = pb.function().inputSize();
            G_.resize(n_, n_);
            g0_.resize(n_);

            G_ = this->problem().function().A();
            g0_ = this->problem().function().b();
            //Function::size_type p = 0;

            //Checking if the added constraint is an equality or inequality
            //so that we can fill the correct matrices
            typedef parent_t::problem_t::intervals_t::const_iterator citer_t;
            typedef parent_t::problem_t::intervalsVect_t::const_iterator citerVect_t;
            typedef typename parent_t::problem_t::constraints_t::const_iterator c_citer_t;
            c_citer_t c_it = pb.constraints().begin();
            //for each constraints
            for (citerVect_t it = pb.boundsVector ().begin ();
                    it != pb.boundsVector ().end (); ++it){
                bool equality = true;
                //for each bounds of constraint i
                for (citer_t it2 = it->begin (); it2 != it->end (); ++it2){
                    parent_t::problem_t::value_type c_lbound = it2->first, c_ubound = it2->second;
                    if (c_lbound != c_ubound){
                        //inequality
                        equality = false;
                        break;
                    }
                }

                //Get A and b for each constraints
                boost::shared_ptr<GenericLinearFunction<roboptim::EigenMatrixDense> > constraint_function = 
                    boost::get<boost::shared_ptr<
                    GenericLinearFunction<roboptim::EigenMatrixDense> > > (*c_it);
                //If constraint is equality, append coefficient in CE_ and ce0_
                if( equality )
                    appendMatrixTo(boost::static_pointer_cast<NumericLinearFunction>(constraint_function)->A(),
                                   boost::static_pointer_cast<NumericLinearFunction>(constraint_function)->b(),
                                   CE_, ce0_);
                else
                //If constraint is inequality, append in CI_, ci0_
                    appendMatrixTo(boost::static_pointer_cast<NumericLinearFunction>(constraint_function)->A(),
                                   boost::static_pointer_cast<NumericLinearFunction>(constraint_function)->b(),
                                   CI_, ci0_);
                    
                if(c_it != pb.constraints().end ())
                    ++c_it;
            }

            // Check if CE or CI is empty
            if( CE_.size() == 0){
                CE_ = Eigen::MatrixXd::Zero(0, n_);
                ce0_ = Eigen::VectorXd::Zero(0);
            }

            if( CI_.size() == 0){
                CI_ = Eigen::MatrixXd::Zero(0, n_);
                ci0_ = Eigen::VectorXd::Zero(0);
            }

            std::cout << "Size G_ " << G_.size() << " " << g0_.size() << std::endl;
            std::cout << "Size CE_ " << CE_.size() << " " << ce0_.size() << std::endl;
            std::cout << "Size CI_ " << CI_.size() << " " << ci0_.size() << std::endl;
            // Matrices are now compatible with quadprog
    }

    QuadprogSolver::~QuadprogSolver () throw (){
    }
    
    void QuadprogSolver::appendMatrixTo( Function::matrix_t& A, Function::vector_t& b, Eigen::MatrixXd& Ae, Eigen::VectorXd& be ){
        Function::size_type n, pA, pAe;

        n = A.cols();
        pA = A.rows();
        pAe = Ae.rows();

        std::cout << "SizeA Ae " << A.cols() << " " << Ae.cols() << std::endl;
        
        Eigen::MatrixXd resultMatrix;
        Eigen::VectorXd resultVector;
        
        //Concatenate Ae and A
        if( Ae.rows() == 0 ){
            resultMatrix = A;
        }
        else{
            resultMatrix.resize(pA+pAe, n);
            resultMatrix << Ae, A;
        }
               
        //Concatenate b and be
        if( be.size() == 0 ){
            resultVector = b;

        }
        else{
            resultVector.resize(pAe+pA);
            resultVector << be,b;
        }

        Ae = resultMatrix;
        be = resultVector;
        return;
    }

    void QuadprogSolver::solve() throw (){
        // We can solve
        Eigen::VectorXd xsol(n_);
        double cost;
        cost = QuadProgPP::solve_quadprog(G_, g0_,
                CE_, ce0_, CI_, ci0_,
                xsol);

        if( cost == std::numeric_limits<double>::infinity() )
            result_ = SolverError( "Solution cannot be found." );    
        else{
            Result res( n_, 1 );
            res.x.resize( n_ );
            res.x = xsol;
            res.value(0) = cost;
            res.constraints.resize( ce0_.size() + ci0_.size() );
            result_ = res;
        }
    }
}

extern "C"
{
    using namespace roboptim;
    typedef QuadprogSolver::parent_t solver_t;

    ROBOPTIM_DLLEXPORT std::size_t getSizeOfProblem ();
    ROBOPTIM_DLLEXPORT const char* getTypeIdOfConstraintsList ();
    ROBOPTIM_DLLEXPORT solver_t* create (const QuadprogSolver::problem_t& pb);
    ROBOPTIM_DLLEXPORT void destroy (solver_t* p);

    ROBOPTIM_DLLEXPORT std::size_t getSizeOfProblem ()
    {
        return sizeof (solver_t::problem_t);
    }

    ROBOPTIM_DLLEXPORT const char* getTypeIdOfConstraintsList ()
    {
        return typeid (solver_t::problem_t::constraintsList_t).name ();
    }

    ROBOPTIM_DLLEXPORT solver_t* create (const QuadprogSolver::problem_t& pb)
    {
        return new QuadprogSolver (pb);
    }

    ROBOPTIM_DLLEXPORT void destroy (solver_t* p)
    {
        delete p;
    }
}
