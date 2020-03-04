function xdot = debrisIntercept(t,x,d_during_step,t_step)

d_during_sim = interp1(t_step, d_during_step, t);
xdot = (d_during_sim' - x);

end