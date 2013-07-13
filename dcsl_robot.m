classdef (Abstract) dcsl_robot < handle
    %DCSL_ROBOT Abstract class for DCSL multi-agent ROS interface and simulator
    % This class provides an interface to control robots (assumed stable in
    % pitch and roll) with the DCSL ROS system. The user must provide
    % initial poses for the robots, a control law, a control method, and a
    % run time duration. The class can be configured to simulate the
    % system.
    %
    % Summary provided below. More help can be found by typing doc
    % dcsl_robots or help dcsl_robots.method_name.
    %
    % The connection to the ROS system requires installation of the
    % web-matlab-bridge available at
    % https://github.com/BrendanAndrade/web-matlab-bridge.
    %
    % SYNTAX
    %
    % h = dcsl_robot(initial_poses, control_law, control_mode, run_time, options)
    %
    % INPUTS
    % initial_poses: n_robots X [x y z theta] matrix containing the initial
    % positions and headings of the robots.
    %
    % control_law:  Function handle to user provided control law. Function
    % should accept time and current states of the robots and return input
    % commands for the robots. Time should be a scalar in seconds. Current
    % states should be a n_robots X 7 matrix with the second dimension in
    % the format [x y z vx vz theta theta_dot]. If in velocity control
    % mode, commands should be returned as a n_robots X 3 matrix with the
    % second dimension in the format [u_x u_theta u_z]. If in waypoint
    % mode, commands should be returned as a n_robots X 4 matrix with the
    % second dimension [x y z theta] of the goal pose. If direct control,
    % commands should be returned as a n_robots X M inputs matrix with
    % order consistant across system. Example: @control_law . If control
    % takes two arguments (time, states) Example: @(t,x)
    % control_law(t,x,additional,arguments)
    %
    % control_mode: 'velocity' 'waypoint' or 'direct'
    %
    % run_time: Time in seconds to run the system or simulation. Provide
    % Inf to run system indefinitely (run mode only).
    %
    % OPTIONS
    % 'sim': Logical. Default: false. true to simulate dynamics in MATLAB.
    % false to run ROS.
    %
    % 'sim_noise': Length 4 Vector. Default: [0 0 0 0]. Standard deviation
    % of the random gaussian noise applied to [x y z theta] measuremente
    % estimates during simulation.
    %
    % 'Ts': Number. Default: 0.0667. Time step for measurement/control
    % update. Only affects simulation. If your control law relies on the
    % time step, it is best to calculate it using the t input to the
    % control law.
    %
    % 'URI': String. Default: 'ws://localhost:9090'. URI of rosbridge
    % server.
    %
    % PROPERTIES
    %
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
    %   third dimension is [time u_x u_theta u_z] (velocity) or [x y z
    %   theta] (waypoint) or direct (as defined).
    %
    %   control_mode - 'velocity', 'waypoint', or 'direct' control of the
    %   robots. Set at initialization of object or via set method.
    %
    %   control_law - Function handle to user provided control law.
    %   Function should accept time and current states of the robots and
    %   return input commands for the robots. Time should be a scalar in
    %   seconds. Current states should be a n_robots X 7 matrix with the
    %   second dimension in the format [x y z vx vz theta theta_dot].
    %   Commands should be returned as a n_robots X 3 matrix with the
    %   second dimension in the format [u_x u_theta u_z].  If direct
    %   control, commands should be returned as a n_robots X M inputs
    %   matrix with order consistant across system.
    %
    %   run_time - Time in seconds to run system or to simulate. To run ROS
    %   system indefinitely set this to Inf. Required at initialization and
    %   can be changed with set_run_time.
    %
    %   sim - Logical type. Default: False. True if object should simulate
    %   system in MATLAB. False if system should command ROS. Set via
    %   set_sim or initialization with 'sim' option.
    %
    %   sim_noise -  Length 4 Vector. Default: [0 0 0 0]. Standard
    %   deviation of the random gaussian noise applied to [x y z theta]
    %   measuremente estimates during simulation.
    %
    %   Ts - Default: 0.0667 (15 Hz). Applies only to
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
    %   of ROS system. Robots will move to initial poses and then system
    %   will execute the control law for the run time.
    %
    %   stop - Stop robots in ROS. Interrupt timed run or end
    %   indefinite run. Does not effect MATLAB simulation. Sets inputs to
    %   zero.
    %
    %   shutdown - Stop robots and close connection to ROS.
    %   
    %   command - Send command to ROS system if it is not beinging
    %   automatically controlled. If in velocity or direct mode, command
    %   should be in the format: n_robots X 3 matrix with the second
    %   dimension in the format [u_x u_theta u_z]. If waypoint mode,
    %   n_robots X 4 with second dimension [x y z theta] as the goal
    %   waypoint.
    %
    %   enable_control - turns on control law manually and begins gathering
    %   state infomration
    %
    %   disable_control - turns off control law, system will still poll
    %   state information
    %
    %   go_to_poses - moves the robots to supplied poses. Works in sim and
    %   run modes.
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
    %   [ux utheta uz] or [x y z theta]
    %   'command_times': returns n_time_steps vector with times of command
    %   sends
    %   'u1' 'u2' 'u3', 'u4': returns n_time_steps vector with corresponding
    %   command history. If velocity or direct u1=ux u2=utheta u3=uz. If
    %   waypoint u1=x u2=y u3=z u4=theta of the waypoints.
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
    
    properties(Access = public)
        control_mode % 'velocity' 'waypoint' or 'direct'
    end
    
    properties(SetAccess = private)
        n_robots % Number of robots
        
        state_estimates % Current estimate of the states of the robots
        
        state_estimate_history % n_robotsX[t states]Xn_time_steps matrix of the state estimate history of the robots.
        command_history % n_robotsX[t commands]Xn_time_steps matrix of the command input history of the robots.
        
        control_law % Handle to the user provided control law.
        
        Ts          % Frequency of pose updates during simulation
        run_time    % Time in seconds to run ROS control or to simulate.
        sim         % Whether is is a simulation (True = sim, False = send to ROS)
        
        sim_noise   % Noise to apply to sensor output of simulation
       
        connected = false  % Logical, True = connected to ROS, false otherwise
        URI         % URI address for rosbridge server
    end
    
    properties(Access = protected)
        vel_pub     % ROS publisher for velocity commands
        wp_pub      % ROS publisher for waypoint commands
        direct_pub  % ROS published for direct commands to robot
        sub         % ROS subscriber for state estimates
        ws          % ros_websocket object
        lh          % Listen handle for subscriber callback
        
        states      % Current state of the robots, used for simulation
        
        start_time  % ROS walltime at the start of the run
        initial_poses % Positions and headings to start the robots at.
        
        control_on = false % Indicator whether to actively apply the control law
        
        last_command % To retain in memory the previously applied command
        first_callback = true % Indicator for initialization during callback
    end
    
    methods
        function set.control_mode(obj, value)
            % SET_CONTROL_MODE Ensures valid control mode is supplied
            %
            % SYNOPSIS set.control_mode(obj, value)
            %
            % INPUTS obj: the object
            % value: a string, the control mode: velocity direct or waypoint
            
            if any(strcmpi(value, {'velocity', 'direct', 'waypoint'}))
                obj.control_mode = value;
            else
                error('control_mode must be "velocity" "direct" or "waypoint"')
            end
        end
    end
    
    methods(Access = public)
        
        function obj = dcsl_robot(initial_poses, control_law, control_mode, run_time, varargin)
            % DCSL_ROBOT Initializes the robot object
            %
            % SYNOPSIS obj = dcsl_robot(control_law, control_node,
            % run_time, opt)
            %
            % INPUTS 
            %
            % initial_poses: n_robots X [x y z theta] matrix containing the
            % initial positions and headings of the robots.
            % 
            % control_law: a function handle to the control law, it
            % should have look like: [ commands ] = control_law(t, x) where
            % t is the current time, a scalar in seconds, x is the
            % current states of the robots, n_robots X 7 matrix with the
            % second dimension in the format [x y z vx vz theta theta_dot],
            % and commands is a matrix of the commands to be sent to the
            % robot. If under velocity control, commands should be n_robots
            % by 3 matrix with the second dimension in the format [u_x_dot
            % u_theta_dot u_z_dot]. If waypoint control, commands should be
            % a n_robots by 4 matrix with the second dimension in the
            % format [x y z theta] (the goal pose). If under direct
            % control, it should be a n_robots by M matrix where M is the
            % number of inputs for that robot.
            %
            % control_mode: a string, 'velocity' 'waypoint' or 'direct'
            %
            % run_time: a scalar number, the length of time in seconds to
            % simulate or run the ROS system. Inf allowed when not
            % simulating. System will run until stop or shutdown method is
            % called.
            %
            % opt: optional arguments enter name of optional arg as string
            % followed by the value. Example: ..., 'sim', true);
            % 'sim': a logical, default: false. false = run ROS on start,
            % true = run simulation in MATLAB on start
            % 'sim_noise': a vector, default: [0 0 0 0]. Standard deviation
            % of the noise to apply to the measurements [x y z theta]
            % during simulation.
            % 'Ts' = a scalar number, default: 1/15. The measurement time
            % step using during simulation in seconds.
            % 'uri' = a string, default: 'ws://localhost:9090'. Address of
            % the ROS webbridge server
                
            % Use inputParser to check validity of arguments
            p = inputParser;
            
            % Default values for optional args
            defaultSim = false;
            defaultURI = 'ws://localhost:9090';
            defaultNoise = [0, 0, 0, 0];
            expected_control_modes = {'velocity', 'waypoint', 'direct'};
            defaultMeasureDT = 1/15;
            
            % Set restrictions on values and setup optional args
            addRequired(p, 'initial_poses', @(x) ismatrix(x) && isnumeric(x) && (size(x, 2)==4));
            addRequired(p, 'control_law', @(x) isa(x,'function_handle'))
            addRequired(p, 'control_mode', @(x) any(validatestring(x, expected_control_modes)));
            addRequired(p, 'run_time', @(x) isnumeric(x) && x >= 0);
            addOptional(p, 'sim', defaultSim, @islogical);
            addOptional(p, 'uri', defaultURI, @ischar);
            addOptional(p, 'sim_noise', defaultNoise, @(x) ismatrix(x) && (length(x)==4));
            addOptional(p, 'Ts', defaultMeasureDT, @(x) isnumeric(x) && (x>0));
            
            % Parse args
            parse(p, initial_poses, control_law, control_mode, run_time, varargin{:});
            
            % Assign args to properties
            poses = p.Results.initial_poses;
            obj.n_robots = size(poses, 1);
            obj.initial_poses = poses;
            obj.states = [poses(:,1:3) zeros(obj.n_robots,2) poses(:,4) zeros(obj.n_robots, 1)];
            % obj.state_estimates = obj.states;
            obj.last_command = zeros(obj.n_robots, 3);
            
            obj.sim = p.Results.sim;
            obj.sim_noise = p.Results.sim_noise;
            obj.URI = p.Results.uri;
            obj.control_mode = p.Results.control_mode;
            obj.Ts = p.Results.Ts;
            obj.run_time = p.Results.run_time;
            obj.control_law = p.Results.control_law;
        end
        
        function connect(obj)
            % CONNECT Setup the connect to ROS without starting the control
            % law.
            %
            % SYNOPSIS connect(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            if obj.connected == false
                obj.setup_ros_connection();
            end
            
        end
            
        
        function start(obj)
            % START Begin the simulation or setup connections to ROS. If control is on, commands will begin being sent to ROS.
            %
            % SYNOPSIS start(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
          
            if obj.sim == false
                % Turn control off and setup ROS connection.
                obj.control_on = false;
                obj.connect();
                
                % Go to initial poses
                reached_poses = obj.go_to_poses(obj.initial_poses);
                
                if reached_poses
                    % Clear history and start control.
                    obj.reset_history();
                    obj.reset_time();
                    obj.control_on = true;
                    obj.start_ros_control();
                    disp('Initial poses reached. Starting control...')
                else
                    disp('Initial poses not reached. Try running start again.');
                end
                
                
            else
                obj.run_simulation();
            end
            
        end
        
        function stop(obj)
            % STOP Turns off control and sets all input signals to zero for all robots. Has no effect on simulation.
            %
            % SYNOPSIS stop(obj)
            % 
            % INPUT obj: the object
            %
            % OUTPUT none
            
            if obj.sim == false
                obj.ros_stop();
            end
        end
        
        function shutdown(obj)
            % SHUTDOWN Stop all robots and close connection to ROS. Has no effect on simulation.
            % 
            % SYNOPSIS shutdown(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            if obj.sim == false
                obj.ros_shutdown();
            end
        end
        
        function enable_control(obj)
            % ENABLE_CONTROL If in run mode, start receiving state
            % estimates from system and enable closed loop control using
            % supplied control law.
            %
            % SYNOPSIS enable_control(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            if obj.sim == false
                obj.control_on = true;
                obj.start_ros_control();
            end
        end
        
        function disable_control(obj)
            % DISABLE_CONTROL Stop execution of control law. State
            % estimates will continue to be polled.
            %
            % SYNOPSIS disable_control(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            obj.control_on = false;
        end
        
        function command(obj, command_array)
            % COMMAND Send a one time input to robots through ROS. Has no
            % effect on simulation.
            %
            % SYNOPSIS command(obj, command_array)
            %
            % INPUT obj: the object
            % command_array: n_robots X M array where the second dimension
            % is formatted according to the description of the command
            % output in the documentation of the control law. Command type
            % much match the currently selected control strategy (velocity,
            % waypoint, or direct).
            %
            % OUTPUT none
            
            if obj.sim == false
                obj.ros_command(command_array)
            end 
        end
           
        function [succeeded] = go_to_poses(obj, pose_array, varargin)
            % GO_TO_POSES Send robots to specfic poses.
            %
            % SYNOPSIS go_to_poses(obj, pose_array, timeout (optional
            % default = 60 seconds))
            %
            % INPUTS obj: the object
            % pose_array: an n_robots X 4 matrix where the second dimension
            % is the goal pose of the robot [x y z theta]
            % timeout (optional) : time allowed to reach goal poses in
            % seconds
            %
            % OUTPUT succeeded: a logical, returns true if poses were
            % reached for all robots, false otherwise
            
            if length(varargin) >= 1
                timeout = cell2mat(varargin(1));
            else
                timeout = 60;
            end
            
            if obj.sim == false
                succeeded = obj.ros_go_to_poses(pose_array, timeout);
            else
                succeeded = obj.sim_go_to_poses(pose_array);
            end
           
        end
        
        % Methods for properties access
        
        function history = get_history(obj, robot_ID, option) % Need to add support for waypoint history/maybe make this robot dependent
            % GET_HISTORY Returns the indicated time history for the
            % specified robot
            %
            % SYNOPSIS get_history(obj, robot_ID, option)
            %
            % INPUT obj: the object
            % robot_ID: an integer, the index of the robot whose history is
            % desired
            % option: string, indicates what history is desired
            %   'states': returns n_time_stepsX7 array with second dimension
            %   as [x y z vx vz theta theta_dot]
            %   'state_times': returns n_time_steps vector with times of
            %   state updates
            %   'x', 'y', 'z', 'vx', 'vz', 'theta', 'theta_dot': returns
            %   n_time_steps vector with corresponding state history
            %   'commands': returns n_time_steps X M array with second
            %   dimension as [u1 u2 u3] or [x y z theta]
            %   'command_times': returns n_time_steps vector with times of
            %   command sends
            %   'u1' 'u2' 'u3': returns n_time_steps vector with
            %   corresponding command history
            %
            % OUTPUT history: a matrix or vector (n_time_steps X M) with
            % the desired history
            
            
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
                case 'u1'
                    history = squeeze(obj.command_history(ID, :, 2));
                case 'u2'
                    history = squeeze(obj.command_history(ID, :, 3));
                case 'u3'
                    history = squeeze(obj.command_history(ID, :, 4));
                case 'u4'
                    history = squeeze(obj.command_history(ID, :, 5));
            end
            
        end
        
        function reset_history(obj)
            % RESET_HISTORY Clear the state and command history
            %
            % SYNOPSIS reset_history(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            obj.state_estimate_history = [];
            obj.command_history = [];   
        end
        
        function reset_time(obj)
            % RESET_TIME Set time recorded in history back to zero.
            %
            % SYNOPSIS reset_time(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
                       
            obj.first_callback = true;
        end
        
    end
    
    methods (Access = protected)
        
        % ROS interaction functions
        
        function setup_ros_connection(obj)
            % SETUP_ROS_CONNECTION Setup ros_websocket, create pubs/subs,
            % connect listener handle to subscriber
            %
            % SYNOPSIS setup_ros_connection(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            if obj.connected == false
                % Create websocket object to connect to ROS
                obj.ws = ros_websocket(obj.URI);
                
                % Initialize publisher objects
                obj.vel_pub = Publisher(obj.ws, 'velocity_input', 'dcsl_messages/TwistArray');
                obj.wp_pub = Publisher(obj.ws, 'waypoint_input', 'geometry_msgs/PoseArray');
                obj.direct_pub = obj.setup_direct_pub(obj.ws);
                
                % Initialize subscriber object
                obj.sub = Subscriber(obj.ws, 'state_estimate', 'dcsl_messages/StateArray');
                
                % Use listener handle to connect callback method to execute
                % when subscriber receives a message
                % obj.lh = event.listener(obj.sub, 'OnMessageReceived', @(h,e) obj.control_callback(h, e));
                
                % Indicate that ROS connection is active
                obj.connected = true;
            end
            
        end
        
        function start_ros_control(obj)
            % START_ROS_CONTROL Setup listener handle to receive state
            % estimates and do control if control is enabled.
            %
            % SYNOPSIS start_ros_control(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            % Use listener handle to connect callback method to execute
            % when subscriber receives a message
            obj.lh = event.listener(obj.sub, 'OnMessageReceived', @(h,e) obj.control_callback(h, e));
        
        end
        
        
        function obj = control_callback(obj, ~, e)
            % CALLBACK Envoked on receipt of state estimate. Records data
            % and sends proper control input back to ROS if enabled.
            %
            % SYNOPSIS [obj] = callback(obj, ~, e)
            %
            % INPUTS obj: the object
            % ~: placeholder for event handle
            % e: event data sent to callback function on event trigger
            %
            % OUTPUT obj: the object
            
            % Receive state data from the event data
            states_struct = e.data;
            
            % Calculate time past since first state_estimate received from
            % system
            wall_time = struct('secs', states_struct.header.stamp.secs, 'nsecs', states_struct.header.stamp.nsecs * 10^(-9));
            if obj.first_callback
                obj.start_time = wall_time;
                obj.first_callback = false;
            end
            time = (wall_time.secs - obj.start_time.secs) + (wall_time.nsecs - obj.start_time.nsecs);
            
            
            
            % Record state estimates into memory and history
            obj.state_estimates = obj.states_struct2mat(states_struct);
            obj.state_estimate_history(:, end+1, :) = [ones(obj.n_robots, 1)*time obj.state_estimates];
            
            % Execute closed loop control if enabled
            if obj.control_on
                % Get commands from control law
                commands = obj.control_law(time, obj.state_estimates);
                
                % Record commands into memory and history
                obj.last_command = commands;
                obj.command_history(:, end+1, :) = [ones(obj.n_robots, 1)*time commands];
                
                % Execute commands
                if time > obj.run_time
                    obj.control_on = false;
                    obj.stop();
                else
                    obj.ros_command(commands);
                end
            end
        end
        
        function ros_shutdown(obj)
            % ROS_SHUTDOWN Sends stop commands to all robots and closes
            % connection to ROS.
            %
            % SYNOPSIS ros_shutdown(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            if obj.connected
                obj.ros_stop
                obj.sub.unsubscribe
                obj.vel_pub.unadvertise
                obj.wp_pub.unadvertise
                obj.direct_pub.unadvertise
                delete(obj.lh)
                delete(obj.ws)
            end
        end
        
        function ros_command(obj, command_array)
            % ROS_COMMAND Publish supplied command. Adhear to the active
            % control strategy. 
            %
            % SYNOPSIS ros_command(obj, command_array)
            %
            % INPUTS obj: the object
            % command_array: an n_robots X M inputs array formatted as
            % described in control law description.
            %
            % OUTPUT none
            
            % Convert commands to appropriate struct and publish
            switch obj.control_mode
                case 'velocity'
                    commands_struct = obj.commands_mat2vel_struct(command_array);
                    obj.vel_pub.publish(commands_struct);
                case 'waypoint'
                    commands_struct = obj.commands_mat2wp_struct(command_array);
                    obj.wp_pub.publish(commands_struct);
                case 'direct'
                    commands_struct = obj.commands_mat2dir_struct(command_array);
                    obj.direct_pub.publish(commands_struct);
            end
        end
        
        function [succeeded] = ros_go_to_poses(obj, poses, timeout)
            % ROS_GO_TO_POSES
            %
            % SYNOPSIS
            %
            % INPUTS obj: the object
            % poses: an n_robots X 4 matrix with the desired poses as the
            % second dimension in the format [x y z theta]
            % timeout: time in seconds to allow the robots to reach the
            % goal poses
            %
            % OUTPUT succeeded: a logical. true = goal poses reached, false
            % otherwise
            
            time_step = 2;
            eps = 0.2;
            
            % Find error
            error = Inf;
           
            time = 0;
            
            % Start go to poses control
            wp_lh = event.listener(obj.sub, 'OnMessageReceived', @(h,e) obj.go_to_poses_callback(h,e, poses));
            
            % Do control to direct robots to points
            while error > eps && timeout > time
                
                % Wait for time_step
                pause(time_step)
                drawnow(); % Process event queue
                
                %Calculate error
                
                if size(obj.state_estimates,1) == obj.n_robots
                    error = 0;
                    for i=1:obj.n_robots
                        x_e = (poses(i,1) - obj.state_estimates(i,1))^2;
                        y_e = (poses(i,2) - obj.state_estimates(i,2))^2;
                        z_e = (poses(i,3) - obj.state_estimates(i,3))^2;
                        theta_e = (poses(i,4) - obj.state_estimates(i,6))^2;
                        error = error + x_e + y_e + z_e + theta_e;
                    end
                end
                time = time + time_step; % Consider switch to timing functions.
            end
            
            % Stop go to poses control
            delete(wp_lh)
            
            % Stop robots
            obj.ros_stop();
            
            if error < eps
                succeeded = true;
            else
                succeeded = false;
                warning('Poses not reached before timeout in ros_go_to_poses method.');
                disp(error);
            end
               
        end
        
        function go_to_poses_callback(obj,~,e, poses)
            % GO_TO_POSES_CALLBACK Envoked on receipt of state estimate
            % during ros_go_to_poses method.
            %
            % SYNOPSIS go_to_poses_callback(obj,~,e, poses)
            %
            % INPUTS obj: the object
            % ~: placeholder for event handle
            % e: event data sent to callback function on event trigger
            % poses: an n_robots X 4 matrix where the 2nd dimension is the
            % goal poses [x y z theta]
            %
            % OUTPUT none
            
            % Receive state data from the event data
            states_struct = e.data;
            obj.state_estimates = obj.states_struct2mat(states_struct);
            
            % Send goal poses
            commands_struct = obj.commands_mat2wp_struct(poses);
            obj.wp_pub.publish(commands_struct);
            
        end
        
        function [commands_struct] = commands_mat2vel_struct(obj, commands_mat)
            % COMMANDS_MAT2VEL_STRUCT Converts commands matrix to a
            % structure formatted as a TwistArray ROS message
            %
            % SYNOPSIS [commands_struct] = commands_mat2vel_struct(obj, commands_mat)
            %
            % INPUTS obj: the object
            % commands_matrix: a n_robots X 3 matrix where the second
            % dimension is formatted as [u_x_dot u_theta_dot u_z_dot]
            %
            % OUTPUT commands_struct: a struct formatted as a TwistArray
            % ROS message (defined in the dcsl_messages package) containing
            % the commands from commands_mat
            
            % Initialize array of structs
            twists = repmat(struct('linear', {}, 'angular', {}), obj.n_robots, 1);
            
            % Populate structs
            for i = 1:obj.n_robots
                twists(i).linear = struct('x', commands_mat(i,1), 'y', 0, 'z', commands_mat(i, 3));
                twists(i).angular = struct('x', 0, 'y', 0, 'z', commands_mat(i, 2));
            end
            
            % Add array of structs to a struct with one property called
            % twists
            commands_struct = struct('twists', twists);
        end
        
        function [commands_struct] = commands_mat2wp_struct(obj, commands_mat)
            % COMMANDS_MAT2WP_STRUCT Converts commands matrix to a
            % structure formatted as a PoseArray ROS message
            %
            % SYNOPSIS [commands_struct] = commands_mat2wp_struct(obj, commands_mat)
            %
            % INPUTS obj: the object
            % commands_matrix: a n_robots X 4 matrix where the second
            % dimension is formatted as [x_goal y_goal z_goal theta_goal]
            %
            % OUTPUT commands_struct: a struct formatted as a PoseArray
            % ROS message (defined in the geometry_msgs package) containing
            % the goal poses from commands_mat
            
            % Initialize array of structs
            poses = repmat(struct('position', {}, 'orientation', {}), obj.n_robots, 1);
            
            % Populate structs
            for i = 1:obj.n_robots
                poses(i).position = struct('x', commands_mat(i,1), 'y', commands_mat(i, 2), 'z', commands_mat(i, 3));
                poses(i).orientation = struct('x', 0, 'y', 0, 'z', commands_mat(i, 4), 'w', 0);
            end
            
            % Add array of structs to a struct with one property called
            % poses
            commands_struct = struct('poses', poses);
        end
        
        function states_matrix = states_struct2mat(obj, states_struct)
            % STATES_STRUCT2MAT Convert the received states struct to a
            % matrix.
            %
            % SYNOPSIS [states_matrix] = states_struct2mat(obj,
            % states_struct)
            %
            % INPUTS obj: the object
            % states_struct: a structure formatted as a StateArray ROS
            % messages (from dcsl_messages package)
            %
            % OUTPUT: states_matrix a n_robots X 7 matrix with the second
            % dimenstion formatted as [x y z x_dot z_dot theta theta_dot]
            
            states_matrix = zeros(obj.n_robots, 7);
            for i=1:obj.n_robots
                states_matrix(i,:) = [states_struct.states(i).pose.position.x states_struct.states(i).pose.position.y states_struct.states(i).pose.position.z states_struct.states(i).twist.linear.x states_struct.states(i).twist.linear.z states_struct.states(i).pose.orientation.z states_struct.states(i).twist.angular.z];
            end
        end
        
        % Simulation methods
        
        function run_simulation(obj)
            % RUN_SIMULATION Simulate behavior in MATLAB of robot under
            % control law.
            %
            % SYNOPSIS run_simulation(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            % Calculate the number of time steps to reach run time
            t_steps = ceil(obj.run_time/obj.Ts);
            
            % Seed state estimate
            obj.state_estimates = obj.states;
            
            % Record state_estimate at t = 0
            obj.state_estimate_history(:, end+1, :) = [zeros(obj.n_robots,1) obj.state_estimates];
            
            for k = 0:t_steps
                
                % Calculate current time
                t = k*obj.Ts;
                
                % Calculate control inputs and record
                commands = obj.control_law(t, obj.state_estimates);
                obj.command_history(:, end+1, :) = [ones(obj.n_robots,1)*t commands];
            
                % Propagate states and record into history
                [obj.states, obj.state_estimates] = obj.propagate(obj.states, commands, obj.Ts, obj.sim_noise);
                obj.state_estimate_history(:, end+1, :) = [ones(obj.n_robots,1)*(t+obj.Ts) obj.state_estimates];
            end
        end
        
        function sim_go_to_poses(obj, poses)
            % SIM_GO_TO_POSES Sets the current states and state estimates
            % of the robots to the poses with zero velocities.
            %
            % SYNOPSIS sim_go_to_poses(obj, poses)
            % 
            % INPUTS obj: the object
            % poses: an n_robots X 4 matrixs with the second dimension the
            % goal pose for the robot in the format [x y z theta]
            %
            % OUTPUT none
            
            obj.states = [poses(:, 1) poses(:,2) poses(:,3) zeros(obj.n_robots, 1) zeros(obj.n_robots,1) poses(:,4) zeros(obj.n_robots,4)];
            obj.state_estimate_history = obj.states;
        end
        
    end
    
    methods (Abstract)
        
        % Return a publisher object for the robot's direct inputs, given...
        % a ros_websocket object
        [direct_pub] = setup_direct_pub(obj, ros_websocket) 
        
        % Return a struct formatted for the correct ROS message type for...
        % direct control of robot given an n_robots X M inputs matrix
        [commands_struct] = commands_mat2dir_struct(obj, commands_mat)
        
        % Stop all robots
        ros_stop(obj)
        
        % Propagate forward in simulation the state of the robot for the...
        % given time step, given the current state, and commands. Return
        % the states at the end of the time step and the states plus the
        % measurement noise as the state_estimates. This should account for
        % the current control_mode setting.
        %
        % INPUTS obj: the object
        % states: an n_robots X M states array containing the current true
        % states (no noise added) of the robots
        % commands: an n_robots X M inputs array containing the input
        % commands (in the correct format for the control mode) to be
        % applied for the duration of the time step.
        % Ts: the measurement time step in seconds
        % sim_noise: a vector containing the standard deviation of gaussian
        % noise to be applied to the states to yield the state estimate.
        % Typically in the format [x_noise y_noise z_noise theta_noise].
        [states, state_estimates] = propagate(obj, states, commands, Ts, sim_noise)
        
        
    end
    
end

