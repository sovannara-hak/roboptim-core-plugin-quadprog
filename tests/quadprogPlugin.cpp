#include <roboptim/core.hh>
#include <stdio.h>
#include <roboptim/core/plugin/quadprog/quadprog.hh>
#include <boost/shared_ptr.hpp>

typedef roboptim::QuadprogSolver::solver_t solver_t;

int main()
{
    //Definition of the quadratic function
    //1/2 (2x1^2 + x1x2 + x2^2)
    roboptim::Function::matrix_t A( 2,2 );
    roboptim::Function::vector_t b( 2 );
    A << 2,0.5,0.5,1;
    b << 0,1;

    roboptim::NumericQuadraticFunction f( A, b );

    //Definition the constraint x1 + x2 - 1 = 0 
    roboptim::Function::matrix_t Ac1( 1,2 );
    Ac1 << 1,1;
    roboptim::Function::vector_t bc1( 1 );
    bc1 << -1;
    boost::shared_ptr< roboptim::NumericLinearFunction > c1(
            new roboptim::NumericLinearFunction( Ac1, bc1 ) );

    //Definition of another constraint 0 < x1 < +infinity
    roboptim::Function::matrix_t Ac2( 2,2 );
    roboptim::Function::vector_t bc2( 2 );
    Ac2 << 1,0,0,0;
    bc2 << 0,0;
    boost::shared_ptr< roboptim::NumericLinearFunction > c2(
            new roboptim::NumericLinearFunction(Ac2, bc2) );

    //Definition of another constraint: 
    roboptim::Function::matrix_t Ac3( 2,2 );
    roboptim::Function::vector_t bc3( 2 );
    Ac3 << 0,0,0,1;
    bc3 << 0,0;
    boost::shared_ptr< roboptim::NumericLinearFunction > c3(
            new roboptim::NumericLinearFunction(Ac3, bc3) );

    
    //Creation of the problem
    solver_t::problem_t pb ( f );
    std::cout << "problem input size : " << pb.function().inputSize() << std::endl;
    std::cout << "problem output size : " << pb.function().outputSize() << std::endl;

    //Set bounds for x in f(x)
    roboptim::Function::intervals_t function_arg_bounds;
    for( unsigned int i=0; i< pb.function().inputSize(); ++i ){
        roboptim::Function::interval_t x_bound = roboptim::Function::makeInterval( -4.0, roboptim::Function::infinity() );
        function_arg_bounds.push_back( x_bound );
    }

    pb.argumentBounds() = function_arg_bounds;

    //Set bounds for constraints: x1 + x2 -1 = 0
    roboptim::Function::intervals_t constraint_bounds;
    roboptim::Function::interval_t c1_0 = roboptim::Function::makeInterval( 0.0, 0.0 );
    constraint_bounds.push_back( c1_0 );

    //Set bounds for c2
    roboptim::Function::intervals_t c2_bounds;
    for( unsigned int i=0; i< pb.function().inputSize(); ++i ){
        roboptim::Function::interval_t c2_interval = roboptim::Function::makeLowerInterval( 0 );
        c2_bounds.push_back(c2_interval);
    }

    //Set bounds for c3
    roboptim::Function::intervals_t c3_bounds;
    for( unsigned int i=0; i< pb.function().inputSize(); ++i ){
        roboptim::Function::interval_t c3_interval = roboptim::Function::makeLowerInterval( 0 );
        c3_bounds.push_back(c3_interval);
    }
      
    //Set scales for the problem
    solver_t::problem_t::scales_t scales_c1(1, 1.0);
    solver_t::problem_t::scales_t scales_c2(pb.function().inputSize(), 1.0);
    
    //Add constraint
    
    pb.addConstraint(
            boost::static_pointer_cast<
      roboptim::GenericLinearFunction<roboptim::EigenMatrixDense>  >
            (c1), constraint_bounds, scales_c1 );
    

    pb.addConstraint(
            boost::static_pointer_cast<
      roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> >
            (c2), c2_bounds, scales_c2);

    pb.addConstraint(
            boost::static_pointer_cast<
      roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> >
            (c3), c3_bounds, scales_c2);

    //boost::shared_ptr<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > c3 =
    //    boost::get< boost::shared_ptr<roboptim::GenericLinearFunction<roboptim::EigenMatrixDense> > > 
    //(*pb.constraints().begin());  
    //std::cout << boost::static_pointer_cast<roboptim::NumericLinearFunction> (c3)->A() << std::endl;
      
    //Set initial guess
    roboptim::NumericQuadraticFunction::argument_t x_init( 2 );
    x_init << 0.4,0.0;

    pb.startingPoint() = x_init;

    //Set path of the roboptim-core-plugin-ipopt.so so that libltdl finds the lib.
    //The path is detected in the CMakeLists.txt which will substitute the variable PLUGIN_PATH during the compilation
    //Those two lines can be omitted if the environment variable LD_LIBRARY_PATH contains the path of the plugin
    lt_dlinit();
    lt_dlsetsearchpath (PLUGIN_PATH);

    roboptim::SolverFactory<solver_t> factory ("quadprog", pb);
    solver_t& solver = factory ();

    solver_t::result_t res = solver.minimum ();

    std::cout << solver << std::endl;

    // Check if the minimization has succeed.
    if (res.which () != solver_t::SOLVER_VALUE)
    {
        std::cout << "A solution should have been found. Failing..."
            << std::endl
            << boost::get<roboptim::SolverError> (res).what ()
            << std::endl;
        return 0;
    } 

    // Get the result.
    roboptim::Result& result = boost::get<roboptim::Result> (res);
    std::cout << "A solution has been found: " << std::endl
              << result << std::endl;
    return 1;
}
