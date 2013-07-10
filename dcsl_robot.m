classdef (Abstract) dcsl_robot < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess = private)
        n_robots % Number of robots
        
        state_estimates % Current estimate of the states of the robots
        
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
        vel_pub     % ROS publisher for velocity commands
        wp_pub      % ROS publisher for waypoint commands
        direct_pub  % ROS published for direct commands to robot
        sub         % ROS subscriber for state estimates
        ws          % ros_websocket object
        lh          % Listen handle for subscriber callback
        
        states      % Current state of the robots, used for simulation
        
        start_time  % ROS walltime at the start of the run
        
        control_on = true % Indicator whether to actively apply the control law
        
        last_command % To retain in memory the previously applied command
        first_callback = true % Indicator for initialization during callback
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
            addRequired(p, 'run_time', @isnumeric);
            addOptional(p, 'sim', defaultSim, @islogical);
            addOptional(p, 'uri', defaultURI, @ischar);
            addOptional(p, 'sim_noise', defaultNoise, @(x) ismatrix(x) && (length(x)==4));
            addOptional(p, 'Ts', defaultMeasureDT, @(x) isnumeric(x) && (x>0));
            
            % Parse args
            parse(p, initial_poses, control_law, control_mode, run_time, varargin{:});
            
            % Assign args to properties
            poses = p.Results.initial_poses;
            obj.n_robots = size(poses, 1);
            obj.states = [poses(:,1:3) zeros(obj.n_robots,2) poses(:,4) zeros(obj.n_robots, 1)];
            obj.state_estimates = obj.states;
            obj.last_command = zeros(obj.n_robots, 3);
            
            obj.sim = p.Results.sim;
            obj.sim_noise = p.Results.sim_noise;
            obj.URI = p.Results.uri;
            obj.control_mode = p.Results.control_mode;
            obj.Ts = p.Results.Ts;
            obj.run_time = p.Results.run_time;
            obj.control_law = p.Results.control_law;
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
                obj.setup_ros_connection();
            else
                obj.run_simulation();
            end
            
        end
        
        function stop(obj)
            % STOP Sets all input signals to zero for all robots. Has no effect on simulation.
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
        
    end
    
    methods (Abstract)
        
        % Setup ros_websocket, create pubs/subs, connect listener handle
        setup_ros_connection(obj) 
        
        % Stop all robots and close connection to ROS
        ros_shutdown(obj) 
        
        % Stop all robots
        ros_stop(obj) 
        
        % Publish supplied command. Adhear to the active control...
        % strategy. Command array should be as defined in control law
        % description. 
        ros_command(obj, command_array)
        
        % Simulate behavior of robots under current control strategy...
        % for given runtime. Save state_estimate_history and
        % command_history.
        run_simulation(obj) % Consider implementing this here and making propagate the abstract method.
       
        
        
    end
    
end

