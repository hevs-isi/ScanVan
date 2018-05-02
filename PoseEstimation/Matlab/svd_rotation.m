function [ vmu ] = svd_rotation(v, u)
% Takes into consideration the reflexion of (det=-1) to guess the 
% rotation based on the vectors Vt and U of the SVD.
% Author: Marcelo Kaihara

m=eye(3);
m(3,3)=det(v*u);
vmu=v*m*u;

end

