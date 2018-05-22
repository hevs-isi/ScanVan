function [ xq, yq ] = projectPointEqui(x3d, y3d, z3d)
%PROJECTPOINTEQUI project the 3D physical point into the equirectangular projection

img_w = 3008;
img_h = img_w;

Knew = [img_w/pi 0 img_w/2
        0 img_h/pi 0
        0 0 1];

d = norm([x3d y3d z3d]);

x_p = x3d/d;
y_p = y3d/d;
z_p = z3d/d;

theta = atan(x_p/z_p);
phi = acos(y_p);
 
v1 = [theta phi 1]';
pp1 = Knew * v1;

xq = pp1(1);
yq = pp1(2);

end

