function [ xq, yq ] = projectPoint(x3d, y3d, z3d)
%PROJECTPOINT converts the physical 3D point to the image coordinates.

% Parameters obtained from the calibration procedure
K=[1.4986603443937638e+03 1.6870849124257068e+00 1.4881906663395866e+03
    0 1.4992803274771227e+03 1.4850937271164917e+03
    0 0 1];

xi=9.9588245927201180e-01;

D=[6.5913171255175158e-03 -1.6489395502742404e-02
    3.6727665027947484e-04 7.6426376377712309e-04];

k0=D(1,1);
k1=D(1,2);
p0=D(2,1);
p1=D(2,2);

sv_u = K(1,1); % f1*eta camera contraction factor
sv_v = K(2,2); % f2*eta camera contraction factor
s = K(1,2); % sheer

sv_x = K(1,3); % u0 camera principal point coordinate
sv_y = K(2,3); % v0 camera principal point coordinate

d = norm([x3d y3d z3d]);

x_p = x3d/d;
y_p = -y3d/d;
z_p = z3d/d;

xu = x_p / (z_p + xi); % x coordinate in the camera frame
yu = y_p / (z_p + xi); % y coordinate in the camera frame

sv_r = (1 + k0*(xu^2 + yu^2) + k1*(xu^2 + yu^2)^2)*xu + 2*p0*xu*yu + p1*(3*xu^2 + yu^2);
sv_t = (1 + k0*(xu^2 + yu^2) + k1*(xu^2 + yu^2)^2)*yu + p0*(xu^2 + 3*yu^2) + 2*p1*xu*yu;

xq = sv_r * sv_u + s * sv_t + sv_x + 1;
yq = sv_t * sv_v + sv_y + 1;

end

