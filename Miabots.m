classdef Miabots < dcsl_robot
    %MIABOTS   Multi-agent Miabot ROS interface and simulator
    % This class provides an interface to control MiabotPro robots with the
    % DCSL ROS system. The user must provide initial poses for the robots,
    % a control law, a control method, and a run time duration. The class
    % can be configured to simulate the system.
    %
    % The connection to the ROS system requires installation of the
    % web-matlab-bridge available at
    % https://github.com/BrendanAndrade/web-matlab-bridge.
    %
    % SYNTAX
    %
    % h = Miabots(initial_poses, control_law, control_mode, run_time, options)
    %
    % INPUTS
    % initial_poses: n_robots X [x y z theta] matrix containing the initial
    % positions and headings of the robots.
    %
    % control_law:  Function handle to user provided control law.
    % Function should accept time and current states of the robots and
    % return input commands for the robots. Time should be a scalar in
    % seconds. Current states should be a n_robots X 7 matrix with the
    % second dimension in the format [x y z vx vz theta theta_dot].
    % Commands should be returned as a n_robots X 3 matrix with the
    % second dimension in the format [u_x u_theta u_z].
    % Example: @control_law if control takes two arguments (time, states)
    % Example: @(t,x) control_law(t,x,additional,arguments)
    %
    % control_mode: 'velocity' 'waypoint' or 'direct'
    %
    % run_time: Time in seconds to run the system or simulation. Provide
    % Inf to run system indefinitely.
    %
    % OPTIONS
    % 'sim': Logical. Default: false. true to simulate dynamics in MATLAB.
    % false to run ROS.
    %
    % 'sim_noise': Length 4 Vector. Default: [0 0 0 0]. Standard deviation
    % of the random gaussian noise applied to [x y z theta] estimates during
    % simulation.
    %
    % 'Ts': Number. Default: 0.04. Time step for measurement/control
    % update. Only affects simulation.
    %
    % 'URI': String. Default: 'ws://localhost:9090'. URI of rosbridge
    % server.
    %
    % PROPERTIES
    %   n_robots - Number of robots to use. Determined by size of
    %   initial_poses given at initialization of object.
    % 
    %   state_estimate - n_robotsX7 matrix. Current state estimates of the
    %   robots. Second dimension is in format [x y z vx vz theta
    %   theta_dot].
    % 
    %   state_estimate_history - n_robots x n_time_steps x 8 matrix. Format
    %   of third dimension is [time x y z vx vz theta theta_dot].
    %     
    %   command_history - n_robots x n_times_steps x 4 matrix. Format of
    %   third dimension is [time u_x u_theta u_z].
    %
    %   control_mode - 'velocity', 'waypoint', or 'direct' control of the
    %   robots. Set at initialization of object.
    %
    %   control_law - Function handle to user provided control law.
    %   Function should accept time and current states of the robots and
    %   return input commands for the robots. Time should be a scalar in
    %   seconds. Current states should be a n_robots X 7 matrix with the
    %   second dimension in the format [x y z vx vz theta theta_dot].
    %   Commands should be returned as a n_robots X 3 matrix with the
    %   second dimension in the format [u_x u_theta u_z].
    %
    %   run_time - Time in seconds to run system or to simulate. To run ROS
    %   system indefinitely set this to Inf. Required at initialization and
    %   can be changed with set_run_time.
    %
    %   sim - Logical type. Default: False. True if object should simulate
    %   system in MATLAB. False if system should command ROS. Set via
    %   set_sim or initialization with 'sim' option.
    %
    %   Ts - Default: 0.04 (25 Hz). Applies only to
    %   simulation. Time step to update control loop/receive updated state.
    %   Set via set.Ts or initialization with 'Ts' option.   
    %   
    %   URI - String. Default 'ws://localhost:9090'. URI of the rosbridge
    %   server. Set at initialization with 'uri' option.
    %
    % METHODS
    %
    %   connect - Setup connect to ROS without starting control.
    %
    %   start - Begin simulation or connect if necessary and start control
    %   of ROS system.
    %
    %   stop - Stop robots in ROS. Interrupt timed run or end
    %   indefinite run. Does not effect MATLAB simulation.
    %
    %   shutdown - Stop robots and close connection to ROS.
    %   
    %   command - Send command to ROS system if it is not beinging
    %   automatically controlled. Command should be in the format: n_robots
    %   X 4 matrix with the second dimension in the format [u_x u_theta
    %   u_z].
    %
    %   get_history(robot_ID, parameter) robot_ID is the index of the
    %   robot's initial_poses in that matrix. parameter options are:
    %   'state': returns n_time_stepsX7 array with second dimension as [x y
    %   z vx vz theta theta_dot]
    %   'state_times': returns n_time_steps vector with times of state
    %   updates
    %   'x', 'y', 'z', 'vx', 'vz', 'theta', 'theta_dot': returns
    %   n_time_steps vector with corresponding state history
    %   'command': returns n_time_stepsX3 array with second dimension as
    %   [ux utheta uz]
    %   'command_times': returns n_time_steps vector with times of command
    %   sends
    %   'ux' 'utheta' 'uz': returns n_time_steps vector with corresponding
    %   command history
    %
    % LICENSE
    %
    % This software is covered under the 2-clause BSD license.
    %   
    %   Copyright (c) 2013, Brendan Andrade
    %   All rights reserved.
    %   
    %   Redistribution and use in source and binary forms, with or without 
    %   modification, are permitted provided that the following conditions 
    %   are met:
    %
    %   Redistributions of source code must retain the above copyright 
    %   notice, this list of conditions and the following disclaimer.
    %   
    %   Redistributions in binary form must reproduce the above copyright 
    %   notice, this list of conditions and the following disclaimer in the
    %   documentation and/or other materials provided with the distribution.
    % 
    %   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
    %   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
    %   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
    %   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
    %   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
    %   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
    %   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
    %   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
    %   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
    %   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
    %   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    %   POSSIBILITY OF SUCH DAMAGE.
    
    properties
       
    end
    
    methods(Access = public)
        
        function obj = Miabots(initial_poses, control_law, control_mode, run_time, varargin )
            % Inherit from superclass 
            obj = obj@dcsl_robot(initial_poses, control_law, control_mode, run_time, varargin{:});
        end
        
    end
    
    methods (Access = public)
        
        % ROS related methods
               
        function ros_stop(obj)
            obj.control_on = false;
            % Wait for callbacks to complete
            drawnow();
            pause(0.1);
            % Send zero inputs to robots
            obj.vel_pub.publish(obj.commands_mat2struct(zeros(obj.n_robots, 3)));
        end
        
        function [direct_pub] = setup_direct_pub(obj, ros_websocket)
            %
            
            direct_pub = Publisher(ros_websocket, 'cmd_vel_array', 'dcsl_messages/TwistArray');
        end
        
        function [commands_struct] = commands_mat2dir_struct(obj, commands_mat)
            %
            
            commands_struct = obj.commands_mat2vel_struct(commands_mat);
        end
        
        % Simulation related methods
        
        function [states_out, measurements_out] = propagate(obj, states_in, commands_in, dt, noise)
            states_out = zeros(obj.n_robots, 7);
            measurements_out = zeros(obj.n_robots, 7);
            for i=1:obj.n_robots
                diffConversionFactor = 0.0667;
                motorScaleFactor = 501;
                max_motor_speed = 1000;
                
                eps = 0.001;
                
                x = states_in(i,1);
                y = states_in(i,2);
                z = states_in(i,3);
                v_x = states_in(i,4);
                v_z = states_in(i,5);
                theta = states_in(i,6);
                omega = states_in(i,7);
                
                
                switch obj.control_mode
                    case {'velocity', 'direct'}
                        u_x = commands_in(i,1);
                        u_omega = commands_in(i,2);
                        u_z = commands_in(i,3);
                    case 'waypoint'
                        [u_x, u_omega, u_z] = obj.wp_law(states_in, commands_in);
                end
                
                v_right = (u_x + u_omega*diffConversionFactor/(2))*motorScaleFactor;
                v_left = (u_x - u_omega*diffConversionFactor/(2))*motorScaleFactor;
                
                if v_right > max_motor_speed
                    v_right = max_motor_speed;
                elseif v_right < -max_motor_speed
                    v_right = -max_motor_speed;
                end
                
                if v_left > max_motor_speed
                    v_left = max_motor_speed;
                elseif v_left < -max_motor_speed
                    v_left = -max_motor_speed;
                end
                
                u_x = ((v_left + v_right)/2)/motorScaleFactor;
                u_omega = (v_right - v_left)/(diffConversionFactor*motorScaleFactor);
                
                if abs(u_omega) < eps
                    theta_out = theta;
                    x_out = x + u_x*dt*cos(theta);
                    y_out = y + u_x*dt*sin(theta);
                else
                    theta_out = theta + u_omega*dt;
                    radius = u_x/u_omega;
                    x_out = x + radius*(sin(theta_out) - sin(theta));
                    y_out = y + radius*(cos(theta) - cos(theta_out));
                    theta_out = wrapToPi(theta_out);
                end
                states_out(i,:) = [x_out y_out z u_x v_z theta_out u_omega];
                measurements_out(i, :) = [x_out+normrnd(0, noise(1)) y_out+normrnd(0, noise(2)) z u_x v_z wrapToPi(theta_out + normrnd(0, noise(4))) u_omega];
            end
        end
        
        function [ux, utheta, uz] = wp_law(obj, states, waypoints)
            %
            
        end
        
    end
    
end

