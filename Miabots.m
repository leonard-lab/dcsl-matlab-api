classdef Miabots < handle
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
    
    properties(SetAccess = private)
        n_robots    % Number of robots
        
        
        state_estimates % Current estimates states of the robots
        
        state_estimate_history % n_robotsX[t states]Xn_time_steps matrix of the state estimate history of the robots.
        command_history % n_robotsX[t commands]Xn_time_steps matrix of the command input history of the robots.
        
        control_mode % 'velocity' 'waypoint' or 'direct'
        control_law % Handle to the user provided control law.
        
        Ts          % Frequency of pose updates during simulation
        run_time    % Time in seconds to run ROS control or to simulate.
        sim         % Whether is is a simulation (True = sim, False = send to ROS)
        
        sim_noise   % Noise to apply to sensor output of simulation
       
        URI         % URI address for rosbridge server
    end
    
    properties(Access = private)
        pub         % ROS publisher for commands
        sub         % ROS subscriber for states
        ws          % ros_websocket object
        lh          % Listen handle for subscriber callback
        
        states      % Current states of the robots in simulation
        state_history % n_robotsX[t states]Xn_time_steps matrix of the state history of the robots.
        
        start_time
        
        control_on = true
        
        last_command
        first_callback = true
    end
    
    methods(Access = public)
        
        function obj = Miabots(initial_poses, control_law, control_mode, run_time, varargin )
            % initial_poses: Number of rows = n_robots, n_cols = 4 [x, y,
            % z, theta]
            
            % control_law should be function handle that takes poses nX7 in and
            % returns commands nX(2 miabots, 3beluga
            
            p = inputParser;
            
            defaultSim = false;
            defaultURI = 'ws://localhost:9090';
            defaultNoise = [0, 0, 0, 0];
            expected_control_modes = {'velocity', 'waypoint', 'direct'};
            defaultMeasureDT = 1/25;
            
            addRequired(p, 'initial_poses', @(x) ismatrix(x) && isnumeric(x) && (size(x, 2)==4));
            addRequired(p, 'control_law', @(x) isa(x,'function_handle'))
            addRequired(p, 'control_mode', @(x) any(validatestring(x, expected_control_modes)));
            addRequired(p, 'run_time', @isnumeric);
            addOptional(p, 'sim', defaultSim, @islogical);
            addOptional(p, 'uri', defaultURI, @ischar);
            addOptional(p, 'sim_noise', defaultNoise, @(x) ismatrix(x) && (length(x)==4));
            addOptional(p, 'Ts', defaultMeasureDT, @(x) isnumeric(x) && (x>0));
            
            parse(p, initial_poses, control_law, control_mode, run_time, varargin{:});
            
            poses = p.Results.initial_poses;
            obj.n_robots = size(poses, 1);
            obj.states = [poses(:,1:3) zeros(obj.n_robots,2) poses(:,4) zeros(obj.n_robots, 1)];
            obj.state_estimates = obj.states;
            obj.state_history(:, 1, :) = [zeros(obj.n_robots,1) obj.states];
            obj.state_estimate_history = obj.state_history;
            obj.last_command = zeros(obj.n_robots, 3);
            
            obj.sim = p.Results.sim;
            obj.sim_noise = p.Results.sim_noise;
            obj.URI = p.Results.uri;
            obj.control_mode = p.Results.control_mode;
            obj.Ts = p.Results.Ts;
            obj.run_time = p.Results.run_time;
            obj.control_law = p.Results.control_law;
        end
        
        function start(obj,varargin)
            if (length(varargin)>=1 && cell2mat(varargin(1)) == false)
                obj.control_on = false;
            elseif (length(varargin)>=1 && cell2mat(varargin(1)) == true)
                obj.control_on = true;
            end
            
            if obj.sim == false
                obj.setup_ros_connection();
            else
                obj.run_simulation();
            end
        end
        
        function stop(obj)
            if obj.sim == false
                obj.ros_stop();
            end
        end
        
        function states = get_states(obj)
            states = obj.states;
        end
        
        function shutdown(obj)
            if obj.sim == false
                obj.ros_shutdown();
            end
        end
        
        function command(obj, command_array)
            if obj.sim == false
                obj.ros_command(command_array)
            else
                obj.sim_command(command_array)
            end
        end
        
        function history = get_history(obj, robot_ID, option)
            %{
            p = inputParser;
            
            expected_options = {'states', 'state_times', 'x', 'y', 'z', 'vx', 'vz', 'theta', 'theta_dot', 'commands', 'command_times', 'ux', 'utheta', 'uz'};
            addRequired(p, 'robot_ID', @(x) (x > 0) && (x <= obj.n_robots));
            addRequired(p, 'option', @(x) any(validateattributes(x, expected_options)));
            
            parse(p, robot_ID, option);
            
            choice = p.Results.option;
            ID = p.Results.robot_ID;
            %}
            
            choice = option;
            ID = robot_ID;
            
            switch choice
                case 'states'
                    history = squeeze(obj.state_estimate_history(ID, :, 2:8));
                case 'state_times'
                    history = squeeze(obj.state_estimate_history(ID, :, 1));
                case 'x'
                    history = squeeze(obj.state_estimate_history(ID, :, 2));
                case 'y'
                    history = squeeze(obj.state_estimate_history(ID, :, 3));
                case 'z'
                    history = squeeze(obj.state_estimate_history(ID, :, 4));
                case 'vx'
                    history = squeeze(obj.state_estimate_history(ID, :, 5));
                case 'vz'
                    history = squeeze(obj.state_estimate_history(ID, :, 6));
                case 'theta'
                    history = squeeze(obj.state_estimate_history(ID, :, 7));
                case 'theta_dot'
                    history = squeeze(obj.state_estimate_history(ID, :, 8));
                case 'commands'
                    history = squeeze(obj.command_history(ID, :, 2:4));
                case 'command_times'
                    history = squeeze(obj.command_history(ID, :, 1));
                case 'ux'
                    history = squeeze(obj.command_history(ID, :, 2));
                case 'utheta'
                    history = squeeze(obj.command_history(ID, :, 3));
                case 'uz'
                    history = squeeze(obj.command_history(ID, :, 4));
            end
        end
        
    end
    
    methods (Access = private)
        
        % ROS related methods
        
        function setup_ros_connection(obj)
            obj.ws = ros_websocket(obj.URI);
            obj.pub = Publisher(obj.ws, 'velocity_input', 'dcsl_messages/TwistArray');
            obj.sub = Subscriber(obj.ws, 'state_estimate', 'dcsl_messages/StateArray');
            obj.lh = event.listener(obj.sub, 'OnMessageReceived', @(h,e) obj.callback(h, e));
        end
        
        function obj = callback(obj, ~, e)
            states_struct = e.data;
            wall_time = struct('secs', states_struct.header.stamp.secs, 'nsecs', states_struct.header.stamp.nsecs * 10^(-9));
            if obj.first_callback
                obj.start_time = wall_time;
                obj.first_callback = false;
            end
            time = (wall_time.secs - obj.start_time.secs) + (wall_time.nsecs + obj.start_time.nsecs);
            obj.state_estimates = obj.states_struct2mat(states_struct, obj.last_command);
            obj.state_estimate_history(:, end+1, :) = [ones(obj.n_robots, 1)*time obj.state_estimates];
            if obj.control_on
                commands = obj.control_law(time, obj.state_estimates);
                obj.last_command = commands;
                obj.command_history(:, end+1, :) = [ones(obj.n_robots, 1)*time commands];
                commands_struct = obj.commands_mat2struct(commands);
                obj.pub.publish(commands_struct);
            end
        end
        
        function ros_shutdown(obj)
            obj.ros_stop
            obj.sub.unsubscribe
            obj.pub.unadvertise
            delete(obj.lh)
            delete(obj.ws)
        end
        
        function ros_stop(obj)
            obj.control_on = false;
            obj.pub.publish(obj.commands_mat2struct(zeros(obj.n_robots, 3)));
        end
        
        function ros_command(obj, command_array)
            obj.pub.publish(obj.commands_mat2struct(command_array));
        end
        
        function states_matrix = states_struct2mat(obj, states_struct, commands)
            states_matrix = zeros(obj.n_robots, 7);
            for i=1:obj.n_robots
                states_matrix(i,:) = [states_struct.states(i).pose.position.x states_struct.states(i).pose.position.y states_struct.states(i).pose.position.z commands(i,1) commands(i,3) states_struct.states(i).pose.orientation.z commands(i,2)];
            end
        end
        
        function commands_struct = commands_mat2struct(obj, commands_mat)
            %twists(1:obj.n_robots,1)=struct();
            for i = 1:obj.n_robots
                linear = struct('x', commands_mat(i,1), 'y', 0, 'z', commands_mat(i, 3));
                angular = struct('x', 0, 'y', 0, 'z', commands_mat(i, 2));
                twists(i) = struct('linear', linear, 'angular', angular);
            end
            commands_struct = struct('twists', twists);
        end
        
        
        % Simulation related methods
        
        function run_simulation(obj)
             t_steps = ceil(obj.run_time/obj.Ts);
             
             for k = 0:t_steps
                 t = k*obj.Ts;
                 commands = obj.control_law(t, obj.state_estimates);
                 if k == 0
                     obj.command_history(:, 1, :) = [ones(obj.n_robots,1)*t commands];
                 else
                     obj.command_history(:, end+1, :) = [ones(obj.n_robots,1)*t commands];
                 end
                 [obj.states, obj.state_estimates] = obj.propagate(obj.states, commands, obj.Ts, obj.sim_noise);
                 obj.state_history(:, end+1, :) = [ones(obj.n_robots,1)*t obj.states];
                 obj.state_estimate_history(:, end+1, :) = [ones(obj.n_robots,1)*t obj.state_estimates];
             end
        end
        
        function [states_out, measurements_out] = propagate(obj, states_in, commands_in, dt, noise)
            states_out = zeros(obj.n_robots, 7);
            measurements_out = zeros(obj.n_robots, 7);
            for i=1:obj.n_robots
                eps = 0.001;
                
                x = states_in(i,1);
                y = states_in(i,2);
                z = states_in(i,3);
                v_x = states_in(i,4);
                v_z = states_in(i,5);
                theta = states_in(i,6);
                omega = states_in(i,7);
                
                u_x = commands_in(i,1);
                u_omega = commands_in(i,2);
                u_z = commands_in(i,3);
                
                v_right = (u_x + u_omega*0.1/(2));
                v_left = (u_x - u_omega*0.1/(2));
                
                if v_right > 1
                    v_right = 1;
                elseif v_right < -1
                    v_right = -1;
                end
                
                if v_left > 1
                    v_left = 1;
                elseif v_left < -1
                    v_left = -1;
                end
                
                u_x = (v_left + v_right)/2;
                u_omega = (v_right - v_left)/(0.1);
                
                if abs(omega) < eps
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
        
    end
    
end

