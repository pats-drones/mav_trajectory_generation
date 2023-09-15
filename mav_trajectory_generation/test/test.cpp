#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <iostream>
#include "Eigen/src/Core/IO.h"
#include <fstream>
int main(){
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    
    // add constraints
    start.makeStartOrEnd(Eigen::Vector3d(0,-0.9,-0.9), derivative_to_optimize);
    vertices.push_back(start);

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0.7,-0.5,-0.9));
    vertices.push_back(middle);

    end.makeStartOrEnd(Eigen::Vector3d(-0.7,-0.1,2.5), derivative_to_optimize);
    vertices.push_back(end);

    // compute segment times
    std::vector<double> segment_times;
    const double v_max = 3.0;
    const double a_max = 10.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    // Solve optimization
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    // obtain segments
    mav_trajectory_generation::Segment::Vector segments;
    opt.getSegments(&segments);

    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);


    // Sample range:
    double t_start = 0.0;   
    double t_end = segment_times[0] + segment_times[1];
    double dt = 0.1;
    std::vector<Eigen::VectorXd> pos;
    std::vector<Eigen::VectorXd> vel;
    std::vector<Eigen::VectorXd> acc;
    std::vector<double> sampling_times; // Optional.
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;

    trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &pos, &sampling_times);
    trajectory.evaluateRange(t_start, t_end, dt, mav_trajectory_generation::derivative_order::VELOCITY, &vel, &sampling_times);
    trajectory.evaluateRange(t_start, t_end, dt, mav_trajectory_generation::derivative_order::ACCELERATION, &acc, &sampling_times);


    Eigen::IOFormat CleanFmt(4, 1, "," , ","  , "" ,  "");


    for (double t: segment_times){
        std::cout<<t<<std::endl;
    }

    std::ofstream myfile;
    myfile.open ("trajectory.csv");
    for (int i =0; i<pos.size(); i++){
        // std::cout << pos[i].format(CleanFmt) << "," << vel[i].format(CleanFmt) << std::endl;

        myfile << pos[i].format(CleanFmt) << "," << vel[i].format(CleanFmt) << "," << acc[i].format(CleanFmt) << "\n";
        
    }
    myfile.close();
    return 0;
}