function [ sv_scene, sv_r, sv_t ] = pose_estimation( p3d_1, p3d_2, p3d_3, iterations )
%pose_estimation Estimates the pose of the scene from the points on the sphere
%Author: Marcelo Kaihara

sv_u=ones(size(p3d_1,1),1);
sv_v=sv_u;
sv_w=sv_u;

for i=1:iterations
    [sv_r, sv_t]=estimation_rot_trans(p3d_1, p3d_2, p3d_3, sv_u, sv_v, sv_w);
    [sv_u, sv_v, sv_w]=estimation_rayons(p3d_1, p3d_2, p3d_3, sv_r, sv_t);    
end

[sv_scene]=pose_scene(p3d_1, p3d_2, p3d_3, sv_r, sv_t);

end

