function [ newPoints ] = projectPoints( points, center )
% Projects the points to the unit sphere centered in center
% Author: Marcelo Kaihara

x_s=points(:,1);
y_s=points(:,2);
z_s=points(:,3);

n=sqrt((x_s-center(1)).^2+(y_s-center(2)).^2+(z_s-center(3)).^2);
x1=(x_s-center(1))./n;
y1=(y_s-center(2))./n;
z1=(z_s-center(3))./n;

newPoints=[x1(:), y1(:), z1(:)];

end

