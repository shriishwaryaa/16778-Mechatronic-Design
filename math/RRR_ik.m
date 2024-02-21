classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_lengths
        link_masses
        joint_masses
        end_effector_mass
    end
    
    methods
        % Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(link_lengths, link_masses, joint_masses, end_effector_mass)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(link_lengths, 2) ~= 1
               error('Invalid link_lengths: Should be a column vector, is %dx%d.', size(link_lengths, 1), size(link_lengths, 2));
            end
            
            if size(link_masses, 2) ~= 1
               error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
               error('Invalid joint_masses: Should be a column vector.');
            end
            
            if ~isnumeric(end_effector_mass)
               error('Invalid end_effector_mass: Should be a number.'); 
            end
            
            robot.dof = size(link_lengths, 1);
            
            if size(link_lengths, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of link lengths.');
            end
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of link lengths. Did you forget the base joint?');
            end
            
            robot.link_lengths = link_lengths;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
            robot.end_effector_mass = end_effector_mass;  
        end
       
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            ll = robot.link_lengths;
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(3,3, robot.dof + 1);
            n = robot.dof;
            tempFrame =  [cos(thetas(1)) -sin(thetas(1)) 0
                          sin(thetas(1))  cos(thetas(1)) 0
                          0               0              1];
            frames(:,:,1) = tempFrame;

            for f = 2 : n
                transLinMat = [1 0 ll(f-1)
                               0 1  0
                               0 0  1];
                
                rotLinMat =   [cos(thetas(f)) -sin(thetas(f))  0
                               sin(thetas(f))  cos(thetas(f))  0
                               0               0               1];
                currentFrame = transLinMat * rotLinMat;
                tempFrame = tempFrame * currentFrame;
                frames(:,:,f) = tempFrame;


            end
            endFrame = [1 0 ll(n)
                        0 1 0
                        0 0 1];
            
             frames(:,:,n+1) = frames(:,:,n)*endFrame;
            
           
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        % Returns [x; y; theta] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
           
            % Extract the components of the end_effector position and
            % orientation.
            x = H_0_ee(1,3);
            y = H_0_ee(2,3);
            th = atan2(H_0_ee(2, 1), H_0_ee(1, 1));
           
            % Pack them up nicely.
            ee = [x; y; th];
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end

        function jacobians = jacobians(robot, thetas)
            
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(3,robot.dof,robot.dof+1); 
            epsilon = 0.001;
            matdx = eye(robot.dof);
           
          
            base_frames = robot.fk(thetas); % Find the frames


            for joint = 1 : robot.dof
                dx = matdx(:,joint) * epsilon;
                thetaP = dx + thetas;
                thetaN = thetas - dx;
                %base_frames = robot.fk(thetas); % Find the frames
                numerMat = robot.fk(thetaP) - robot.fk(thetaN);
                frames_x = numerMat(1,3,:); % Get x values of the origins
                %disp(base_frames);
                %disp(frames_x);
                frames_y = numerMat(2,3,:); % Get y values of the origins
               
                for frame = 1 : size(base_frames,3)
                    
                    jacobians(1, joint, frame) = frames_x(frame)./(2*dx(joint));

                   
                    jacobians(2, joint, frame) = frames_y(frame)./(2*dx(joint));

                    
                    jacobians(3, joint, frame) = (1/epsilon).*dx(joint);
                end
            end
            

        end

        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 2 && size(goal_position, 1) ~= 3) || ...
               size(goal_position, 2) ~= 1
                error('Invalid goal_position: Should be a 2 or 3 length column vector, is %dx%d.', size(goal_position, 1), size(goal_position, 2));
            end

            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;

            % Step size for gradient update
            step_size = 0.1;

            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00005;

            % Also, limit to a maximum number of iterations.
            max_iter = 100;
            num_iter = 0;


            % Run gradient descent optimization
            while (num_iter < max_iter)
               
                jac = robot.jacobians(thetas);
                jT = jac(:,:,end)';
                jtSmall = jT(:,1:2);
                endE = robot.ee(thetas);
                if (size(goal_position, 1) == 2) % [x;y] goal
                    diff = endE(1:2) - goal_position;
                    cost_gradient = (jtSmall*(diff));
                
                else % [x;y;theta] goal
                    diff = endE - goal_position;
                    cost_gradient = (jT * diff);
                end

                
                thetas = thetas - (cost_gradient * step_size);
                
                mag = norm(cost_gradient);
                if(mag == stopping_condition)
                    return;
                end
                num_iter = num_iter + 1;
            end

        end


    end
end
