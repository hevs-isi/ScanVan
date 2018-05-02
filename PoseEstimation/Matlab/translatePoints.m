function [ newPoints ] = translatePoints( points, scale, shift )
% Moves the sphere points by the shift
% Author: Marcelo Kaihara

x=points(:,1);
y=points(:,2);
z=points(:,3);

% Creates a matrix with the coordinates of the points of the sphere
M=[ x(:)'
    y(:)'
    z(:)'
    ones(1,numel(x))];

% Apply a translation
T=[scale 0 0 shift(1)
   0 scale 0 shift(2)
   0 0 scale shift(3)
   0 0 0 1];

M2=T*M;

% Goes back to the cartesian coordinate system
x_s = M2(1,:)./M2(4,:);
y_s = M2(2,:)./M2(4,:);
z_s = M2(3,:)./M2(4,:);

newPoints=[x_s(:), y_s(:), z_s(:)];

end

