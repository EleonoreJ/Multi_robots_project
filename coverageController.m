function p_dot = coverageController(~, p, k, Q, mask)
num_robots = length(p)/2;
p = reshape(p, [num_robots, 2]);
p_dot = zeros(num_robots, 2);

p_mask = p(mask==1,:);
px = p_mask(:,1);
py = p_mask(:,2);
p_dot_mask = p_dot(mask==1,:);

[V,C] = VoronoiBounded(px, py, Q);
for robot_i = 1:length(C)
    [cx,cy] = centroid(polyshape(V(C{robot_i},:)));
    Cv = [cx,cy];
    p_dot_mask(robot_i,:) = k*(Cv-p_mask(robot_i,:));
end

p_dot(mask==1,:) = p_dot_mask;

p_dot = reshape(p_dot, [2*num_robots, 1]);
end