classdef Belugas < dcsl_robot
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        params = struct('m1', 30, 'm3', 15, 'J', 1.4, 'eta3Up', 0.92, 'eta3Down', 0.94, ...
            'eta1', 0.73, 'Kd3', 60, 'Kt', 0.17, 'KOmega', 3.3, 'Kd1', 66, 'r', 0.35, ...
            'Kg', 0.6, 'zOffset', 1.5, 'Kdz', 0.004); % Parameter struct for beluga dynamic model
        
    end
    
    properties (Access = private)
        K
        vx_setpoints
        vz_setpoints
        z_range
        theta_dot_setpoints
    end
    
    methods (Access = public)
        
        function obj = Belugas(initial_poses, control_law, control_mode, run_time, varargin)
            % Inherit from superclass
            obj = obj@dcsl_robot(initial_poses, control_law, control_mode, run_time, varargin{:});
            
            addpath('yamlmatlab')
            
            YamlStruct = ReadYaml('vel_controller.yaml',1);
            
            obj.K = reshape(cell2mat(YamlStruct.K_flat), cell2mat(YamlStruct.K_shape));
            obj.vx_setpoints = cell2mat(YamlStruct.axis_2_coordinates);
            obj.vz_setpoints = cell2mat(YamlStruct.axis_3_coordinates);
            obj.z_range = cell2mat(YamlStruct.axis_1_coordinates);
            obj.theta_dot_setpoints = cell2mat(YamlStruct.axis_4_coordinates);
            
        end
        
    end
    
    methods (Access = public)
        
        % ROS related methods
        
        function ros_stop(obj) 
            % ROS_STOP Stops the robots' movement through the ROS system.
            %
            % SYNOPSIS ros_stop(obj)
            %
            % INPUT obj: the object
            %
            % OUTPUT none
            
            obj.control_on = false;
            drawnow()
            pause(0.1)
            message = obj.commands_mat2dir_struct(zeros(obj.n_robots, 3));
            % Hack for 1 robot
            if obj.n_robots == 1
                special_arg = 'array';
            else
                special_arg = {};
            end
            obj.direct_pub.publish(message, special_arg);
        end
        
        function [direct_pub] = setup_direct_pub(obj, ros_websocket)
            %
            
            direct_pub = Publisher(ros_websocket, 'direct_input', 'dcsl_messages/BelugaArray');
        end
        
        function [commands_struct] = commands_mat2dir_struct(obj, commands_mat)
            %
            
            belugas = repmat(struct('thrust_motor', {}, 'servo', {}, 'vertical_motor', {}), obj.n_robots, 1);
            
            for i = 1:obj.n_robots
                belugas(i).thrust_motor = int16(commands_mat(i,1));
                belugas(i).servo = commands_mat(i,2);
                belugas(i).vertical_motor = int16(commands_mat(i,3));
            end
            commands_struct = struct('belugas', belugas);
        end
        
        % Simulation related methods
        
        function [states_out, measurements_out] = propagate(obj, states_in, commands_in, dt, noise)
            %
            
            % Preallocate
            states_out = zeros(obj.n_robots, 7);
            measurements_out = zeros(obj.n_robots, 7);
            
            % Propagate each robot
            for i=1:obj.n_robots

                % Get control inputs
                switch obj.control_mode
                    case 'direct'
                        U = commands_in(i,:);
                    case 'velocity'
                        U = obj.vel_law(states_in(i,:), commands_in(i, :));
                    case 'waypoint'
                        U = obj.wp_law(states_in(i,:), commands_in(i, :));
                end
                
                % Limit control inputs here
                
                % Propagate states
                t_span = [0 dt];
                [~, x_out] = ode45(@(t,x) obj.dynamics(t, x, U), t_span, states_in(i,:));
                states_out(i,:) = x_out(end, :);
                states_out(i,6) = wrapToPi(states_out(i,6));
                measurements_out(i,:) = states_out(i,:) + [normrnd(0, noise(1)), normrnd(0, noise(2)), normrnd(0, noise(3)), 0, 0, normrnd(0, noise(4)) 0];
            end     
        end
        
        function [dX] = dynamics(obj, ~, X, U)
            % Extract variables
            z        = X(3);
            u        = X(4);
            w        = X(5);
            theta    = X(6);
            thetaDot = X(7);
            
            % Extract control inputs
            ut   = U(1);    % horizontal thruster input
            uphi = U(2);    % horizontal thruster servo input (radians)
            uz   = U(3);    % vertical thruster input
            
            % Kinematics
            xDot = cos(theta)*u;
            yDot = sin(theta)*u;
            zDot = w;
            
            % Force model
            % Body-1 (axial) force
            %F1 = Kt1*ut - Kd1*u;
            F1 = (1-obj.params.eta1)*obj.params.Kt*ut*cos(uphi)- obj.params.Kd1*u*abs(u);
            
            % Body-2 (sideslip) force
            % F2 = 0; (by assumption)
            
            % Body-3 (vertical) force
            % logic to determine if actuator is pushing up or down (different
            % efficiency coefficients)
            if uz < 0       % want to descend
                eta = obj.params.eta3Down;
            elseif uz > 0   % want to ascend
                eta = obj.params.eta3Up;
            else            % uz = 0
                eta = 0;
            end
            
            % F3thrust = eta*(abs(uz)*(abs(uz)+11.25))/(abs(w) + wOffset);
            F3thrust = (1-eta)*uz*obj.params.Kt;
            F3drag   = -obj.params.Kd3*w*abs(w);
            F3tether = obj.params.Kg*(obj.params.zOffset - z);
            
            F3 = F3thrust + F3drag + F3tether;
            
            % Torque model
            Gamma = -1*(1-obj.params.eta1)*obj.params.Kt*ut*obj.params.r*sin(uphi) - obj.params.KOmega*thetaDot*abs(thetaDot) - obj.params.Kdz*uz;
            
            % Accelerations
            uDot = F1/obj.params.m1;
            wDot = F3/obj.params.m3;
            
            thetaDotDot = Gamma/obj.params.J;
            
            % Output the derivative of the state
            dX = [xDot yDot zDot uDot wDot thetaDot thetaDotDot]';
        end
            
        function u_direct = vel_law(obj, state, vel_cmd)
            
            x_star_reduced = [state(3) vel_cmd(1) vel_cmd(3) vel_cmd(2)];
            x_reduced = [state(3) state(4) state(5) state(7)];
            
            u_star = obj.calc_u_nominal(x_star_reduced);
            K_star = obj.interpolate_K(x_star_reduced);
            e = x_reduced-x_star_reduced;
            u_direct = (-K_star*e' + u_star')';
        end
       
        function [K_out] = interpolate_K(obj, x)
            
            n_inputs = 3;
            n_states = 4;
            
            [g1, g2, g3, g4] = ndgrid(obj.z_range, obj.vx_setpoints, obj.vz_setpoints, obj.theta_dot_setpoints);
            K_out = zeros(n_inputs, n_states);
            
            for j=1:n_states
                for i=1:n_inputs
                    K_out(i,j) = interpn(g1, g2, g3, g4, obj.K(:,:,:,:,i,j), x(1),x(2),x(3),x(4));
                end
            end
        end
        
        function [u_star] = calc_u_nominal(obj, x_star)
            
            x3 = x_star(1);
            x4 = x_star(2);
            x5 = x_star(3);
            x7 = x_star(4);
            
            if x5 >= 0
                eta3 = obj.params.eta3Up;
            else
                eta3 = obj.params.eta3Down;
            end
            
            u3 = (obj.params.Kd3*x5*abs(x5) - obj.params.Kg*(obj.params.zOffset - x3))/((1-eta3)*obj.params.Kt);
            if x4 ~= 0
                u2 = atan(-(obj.params.KOmega*x7*abs(x7) + obj.params.Kdz*u3)/(obj.params.Kd1*obj.params.r*x4*abs(x4)));
            else
                u2 = sign(x7) * -pi/2;
            end
            if u2 ~= 0
                u1 = -(obj.params.KOmega*x7*abs(x7) + obj.params.Kdz*u3)/((1-obj.params.eta1)*obj.params.Kt*obj.params.r*sin(u2));
            else
                u1 = (obj.params.Kd1*x4*abs(x4))/((1-obj.params.eta1)*obj.params.Kt);
            end
            u_star = [u1 u2 u3];
        end
        
        function [u_thrust, u_phi, u_vert] = wp_law(obj, state, waypoint)
        end
    end
    
end

