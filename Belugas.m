classdef Belugas < dcsl_robot
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
    end
    
    methods (Access = private)
        
        function setup_ros_connection(obj) % Should be an abstract method
            obj.ws = ros_websocket(obj.URI);
            obj.vel_pub = Publisher(obj.ws, 'velocity_input', 'dcsl_messages/TwistArray');
            obj.wp_pub = Publisher(obj.ws, 'waypoint_input', 'geometry_msgs/PoseArray');
            obj.direct_pub = Publisher(obj.ws, 'cmd_array', 'dcsl_messages/BelugaArray');
            obj.sub = Subscriber(obj.ws, 'state_estimate', 'dcsl_messages/StateArray');
            obj.lh = event.listener(obj.sub, 'OnMessageReceived', @(h,e) obj.callback(h, e));
        end
        
        function obj = callback(obj, ~, e)
        end
        
        function ros_shutdown(obj)
            obj.ros_stop
            obj.sub.unsubscribe
            obj.pub.unadvertise
            delete(obj.lh)
            delete(obj.ws)
        end
        
        function ros_stop(obj) 
        end
        
        function ros_command(obj, command_array)
        end
        
        function run_simulation(obj)
        end
        
    end
    
end

