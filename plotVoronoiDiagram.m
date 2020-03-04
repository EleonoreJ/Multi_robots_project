function plotVoronoiDiagram(p,t,bnd, mask)

fig1 = figure();
v = VideoWriter('VoronoiDiagram.mp4','MPEG-4');
v.Quality = 100;
open(v)

num_robots = size(p,2)/2;
Q = [bnd(1) bnd(4);bnd(2) bnd(4);bnd(2) bnd(3);bnd(1) bnd(3);bnd(1) bnd(4)];

%p = reshape();

for i = 1:size(p,1)
    clf
    
    p_coverage = p(mask==1);
    
    px = p(i,1:num_robots)';
    py = p(i,num_robots+1:end)';
    
    [V,C] = VoronoiBounded(px, py, Q);
    
    hold on
    for j = 1:length(C)
        pgon = polyshape(V(C{j},:));
        plot(pgon,'FaceColor','none')
        [cx,cy] = centroid(pgon);
        plot(cx,cy,'+k')
        plot(px(j),py(j),'bo')  
    end
    axis(bnd)
%     set(gca,'position',[0 0 1 1],'units','normalized')
    title(['Voronoi Diagram - ', num2str(num_robots), ...
           ' robots - ', 'Time: ', num2str(t(i)), ' seconds'])
    xlabel('X-Position'); ylabel('Y-Position')
    hold off
    drawnow limitrate
    frame = getframe(fig1);
    writeVideo(v,frame);
end
close(v)
end