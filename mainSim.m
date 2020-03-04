function [R,D,robot_mask] = mainSim(simTime, robot_start, debris_start, gain, sensor_range, Q)
% Get size of states
num_robots = size(robot_start,1);
num_debris = size(debris_start,1);
% Initialize mask for deciding which controller to use
robot_mask = ones(num_robots, length(simTime));
debris_mask = zeros(num_debris, length(simTime));
% Initialize global states matrices
R = zeros(num_robots, 2, length(simTime));
D = zeros(num_debris, 2, length(simTime));
R(:,:,1) = robot_start;
D(:,:,1) = debris_start;
% Loop over simulation time
for t_ind = 1:length(simTime)-1
    % Get time span for numerical integration
    dt = [simTime(t_ind),simTime(t_ind+1)];
    % Get mask for controller switching
    for ii = 1:num_robots
        for jj = 1:num_debris
            distance = norm(R(ii,:,t_ind)-D(jj,:,t_ind));
            % If within sensor range and not seen before
            if distance <= sensor_range && debris_mask(jj,t_ind) == 0
                disp('mask updated')
                robot_mask(ii,t_ind) = 0;
                % Debris mask holds associated robot index
                debris_mask(jj,t_ind:end) = ii;
            end
        end
    end
    % Reshape global states into ode45 compatible dimensions
    IC_robots = reshape(R(:,:,t_ind), [2*num_robots,1]);
    IC_debris = reshape(D(:,:,t_ind), [2*num_debris,1]);
    % Propagate robot one time step using coverage controller
    [t_r, robot_state] = ode45(@(t,r) ...
        coverageController(t,r,gain,Q,robot_mask(:,t_ind)), dt, IC_robots);
    % Propagate debris state one time step using "current" velocity field
    [t_d, debris_state] = ode45(@(t,d) debrisPropagator(t,d), dt, IC_debris);
    % Reshape states to account for mask
    robot_state = reshape(robot_state(end,:)', [num_robots,2]);
    debris_x = reshape(debris_state(:,1:num_debris),[num_debris,1,length(t_d)]);
    debris_y = reshape(debris_state(:,num_debris+1:end),[num_debris,1,length(t_d)]);
    debris_history = [debris_x debris_y];
    debris_state = reshape(debris_state(end,:)', [num_debris,2]);
    % Consensus controller propagation
    for k = 1:num_debris
        if debris_mask(k,t_ind) ~= 0
            robot_id = debris_mask(k,t_ind);
            disp('debris detected')
            disp(robot_id)
            target_state = squeeze(debris_history(k,:,:))';
            interceptor_IC = robot_state(robot_id,:);
            [~, x] = ode45(@(t,x) debrisIntercept(t,x,target_state,t_d),dt,interceptor_IC);
            % Repack correct robots into robot_state
            robot_state(robot_id,:) = x(end,:);
        end
    end
    % Repack updated states into global matrices
    R(:,:,t_ind+1) = robot_state;
    D(:,:,t_ind+1) = debris_state;
end
end