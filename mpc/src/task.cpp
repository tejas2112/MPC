// Bicycle Model
#include <cppad/ipopt/solve.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include <iostream>
#include <fstream>
#include <string>

// obstacle position:
float obs_x;
float obs_y;

// current position
float x_curr;
float y_curr;
float theta_curr;
float vel_curr;
float steering_ang_curr;
float dist = 20; // this is ensure that the while is executed at least once.

int marker = 0;  // this marker is set to 1 after the mpc is executed at least once. This is done for initializing the steering angle and velocity.
int marker2 = 0; // this marker is for verifying whether the required number of input parameters is recieved or not.

namespace
{
    using CppAD::AD;
    class FG_eval
    {
        public:
            int N; // Horizon Length
            double T; // Sampling time period
            double target_x; // target x position
            double target_y; // target y position
            double target_theta; // target angle
            double L; // length of the bicycle

            typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
            void operator()(ADvector& fg, const ADvector X)
            {    
                assert(fg.size() == (2*N)+5); 
                // 1 for objective function, 
                // 2*N constraints for ensuring that the consecutive velocity and steering angle do not change by much,
                // 3 more constraints ensuring that the position and the angle remain in valid range.
                assert(X.size() == 2*N);
                // N velocity and N steering angle values to be found. (for the next N steps)
                // We need to find these 2*N values

                // Fortran style indexing               
                AD<double> vel[N]; // array storing the next N velocities.
                AD<double> steering_ang[N]; // array storing the next N steering angles.
                for (int i = 0; i < N; i++) // initialising the same
                {
                    vel[i] = X[2*i];
                    steering_ang[i] = X[2*i+1];
                }

                // local variable to store the position. Following for-loop will find the expected position after N steps
                AD<double> x_n = x_curr;
                AD<double> y_n = y_curr; 
                AD<double> theta_n = theta_curr;
                // these values are calculated using the vecoities and the steering angles that we need to optimize for. 
                for (int i = 0; i < N; i++)
                {
                    x_n += T*vel[i]*CppAD::cos(theta_n + steering_ang[i]);
                    y_n += T*vel[i]*CppAD::sin(theta_n + steering_ang[i]);
                    theta_n += T*CppAD::sin(steering_ang[i])*vel[i]/L;
                }
                
                // f(x)
                // Objective eqn: Distance between the target and the position after n steps.
                fg[0] = pow(x_n-target_x,2) + pow(y_n-target_y,2) + pow(theta_n-target_theta,2);

                // Constraints:
                // eqn for the distance from the obstacle is greater than 2 at each time step
                fg[1] = (x_n- obs_x)*(x_n- obs_x) + (y_n- obs_y)*(y_n- obs_y); // squared distance greater than 4
                for (int i = 0; i < N-1; i++)
                {
                    fg[2*i+2] = abs(vel[i+1] - vel[i]); // to ensure that the consecutive values don't differ by much
                    fg[2*i+3] = abs(steering_ang[i+1] - steering_ang[i]);
                }
                fg[2*N+2] = x_n; // constraint for position to be in valid range.
                fg[2*N+3] = y_n;
                fg[2*N+4] = theta_n;
                return;
            }
     };
}

void obs_position_callback(const turtlesim::Pose& msg)
{
    // called by subscriber of turtle1's pose.
    // to update the obstacle position
    obs_x = msg.x;
    obs_y = msg.y;
}
void curr_position_callback(const turtlesim::Pose& msg)
{
    // called by subscriber of turtle2's pose.
    // to update the current position and velocity.
    x_curr = msg.x;
    y_curr = msg.y;
    theta_curr = msg.theta;
    vel_curr = msg.linear_velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_task");
    ros::NodeHandle n;
    ros::Subscriber turt1_pose_sub = n.subscribe("/turtle1/pose", 10, obs_position_callback); // subscribes to the obstacle's pose
    ros::Subscriber turt2_pose_sub = n.subscribe("/turtle2/pose", 10, curr_position_callback); // subscribes to the controlling object's pose
    ros::Publisher turt2_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",10); // to publish the velocity
    
    geometry_msgs::Twist velocity;
    // object is controlled only by giving a linear velocity along the direction it faces and the angular velocity along the perpendicular to the plane.
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    
    // object that computes objective and constraints
    FG_eval fg_eval;

    // Taking input from the parameter file.
    std::ifstream param_file;
    param_file.open("catkin_ws/src/mpc/src/params/param.txt");
    // this path is relative from the home directory.
    // If it the file is not being opened by the code then check if the file exists in the params directory.
    // If yes then uncomment the following lines to see the current directory and then edit relative path.
    if(param_file.is_open())
    {
        std::string s;
        while (param_file.good())
        {
            int is_comment = 0;
            getline(param_file,s);
            std::vector<std::string> words;
            std::string word;
            std::stringstream ss(s);
            while (ss >> word)
            {
                if(word == "//") break; 
                words.push_back(word);
                is_comment = 1;
            }
            if(is_comment == 1)
            {
                assert(words.size() == 3);
                if(words[0] == "N")
                {
                    fg_eval.N = std::stoi(words[2]);
                    marker2 += 1;
                }
                else if((words[0] == "T"))
                {
                    fg_eval.T = std::stod(words[2]);
                    marker2 += 1;
                }
                else if((words[0] == "target_x"))
                {
                    fg_eval.target_x = std::stod(words[2]);
                    marker2 += 1;
                }
                else if((words[0] == "target_y"))
                {
                    fg_eval.target_y = std::stod(words[2]);
                    marker2 += 1;
                }
                else if((words[0] == "target_theta"))
                {
                    fg_eval.target_theta = std::stod(words[2]);
                    marker2 += 1;
                }
                else if((words[0] == "L"))
                {
                    fg_eval.L = std::stod(words[2]);
                    marker2 += 1;
                }
                else
                {
                    std::cout << "Check parameter file. Unidentified parameter entered\n";
                    ros::shutdown();
                }
            }
        }
        assert(marker2 == 6); // to ensure that the required number of parameters are given.
    }

    typedef CPPAD_TESTVECTOR( double ) Dvector;
    size_t nx = 2*(fg_eval.N);
    Dvector X_initial(nx);  // initial values of the velocities and the steering angle. This is required for the optimization algo used by CPPAD
    Dvector X_lower(nx);  // lower limits of the velocity and the steering angles.
    Dvector X_upper(nx);  // upper limits of the velocity and the steering angles.

    size_t ng = 2*fg_eval.N+4;
    Dvector gl(ng), gu(ng); // lower and upper limits of the constraints value.

    for (int i = 0; i < fg_eval.N; i++)
    {
        X_lower[2*i] = -1.5;
        X_lower[2*i+1] = -1.0;

        X_upper[2*i] = 1.5;
        X_upper[2*i+1] = 1.0;
    }

    gl[0] = 4.0; gu[0] = 1e19; // constraint for the distance between the turtlesim and the obstacle
    for (int i = 1; i < 2*fg_eval.N+1; i++)
    {
        // constraints to ensure that the cnsecutive values don't differ by much
        gl[i] = 0;
        gu[i] = 0.5;
    }
    // constraint for position to be in valid range.
    gl[2*fg_eval.N+1] = 0; gu[2*fg_eval.N+1] = 10;
    gl[2*fg_eval.N+2] = 0; gu[2*fg_eval.N+2] = 10;
    gl[2*fg_eval.N+3] = -3.14; gu[2*fg_eval.N+3] = 3.14; // constraint for angle to be in valid range.

    // print options for cppad
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // Disables printing IPOPT creator banner
    options += "String  sb          yes\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    // options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    while ((dist > 0.01) && (ros::ok())) // run till the distance between the target and the turtlesim is less than 0.01. Also to stop running if the rosnode is shutdown
    {
        // initializing the steering angle and velocity.
        if(marker == 0)
        {
            for (int i = 0; i < fg_eval.N; i++)
            {
                X_initial[2*i] = vel_curr;
                X_initial[2*i + 1] = 0;
            }
            marker = 1;
        }
        else
        {
            for (int i = 0; i < fg_eval.N; i++)
            {
                X_initial[2*i] = solution.x[2*i];
                X_initial[2*i + 1] = solution.x[2*i + 1];
            }
        }

        // calling cppad function to optimize for the best values of velocity and steering angles
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, X_initial, X_lower, X_upper, gl, gu, fg_eval, solution
        );
        ROS_INFO("solutions_fg_eval: %f", solution.obj_value); // printing out the objective function value
        // publish
        velocity.linear.x = solution.x[0];
        velocity.angular.z = sin(solution.x[1])*solution.x[0]/fg_eval.L; // angular velocity in terms of steering angle
        // std::cout << velocity << "\n";
        
        // to ensure that the same values of velocity and steering angle are published for the sampling time period after which the values will be calculated again
        double begin = ros::Time::now().toSec();
        while (ros::Time::now().toSec() < begin+fg_eval.T)
        {
            turt2_vel_pub.publish(velocity);
        }
        ros::spinOnce();

        // updating the distance between the obstacle and the turtlesim
        dist = pow(pow(x_curr-fg_eval.target_x,2) + pow(y_curr-fg_eval.target_y,2) + pow(theta_curr-fg_eval.target_theta,2),0.5);  
    }
    ros::spin();

  return 0;
}
