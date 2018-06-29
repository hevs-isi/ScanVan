function [ sv_scene ] = pose_scene( p3d_1, p3d_2, p3d_3, sv_r, sv_t )
% Author: Marcelo Kaihara

longueur = size(p3d_1,1);
sv_r_12 = sv_r(1:3,1:3);
sv_r_23 = sv_r(1:3,4:6);
sv_r_31 = sv_r(1:3,7:9);
sv_t_12 = sv_t(1:3);
sv_t_23 = sv_t(4:6);

c1 = zeros(1,3);
c2 = c1 + sv_t_12;
c3 = sv_t_12 + (sv_r_12 *sv_t_23')';

sv_scene = zeros(longueur,3);

azim1m = p3d_1;
azim2m = (p3d_2 * sv_r_23) * sv_r_31;
azim3m = p3d_3 * sv_r_31;

for i=1:longueur    
    inter = intersection ([c1;c2;c3],[azim1m(i,:);azim2m(i,:);azim3m(i,:)]);
    sv_scene(i,:)=inter;    
end

