%% Undistorts images obtained from the omnidirectional camera to equirectangular representation
% Author: Marcelo Kaihara

img1=imread('./img1.bmp');

figure; 
imshow (img1);
hold on;

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

img_w = 3008;
img_h = img_w;

radius = 1;

[x,y]=meshgrid(1:size(img1,2),1:size(img1,1));
xq=zeros(img_h,img_w);
yq=zeros(img_h,img_w);
Knew = [img_w/pi 0 img_w/2
        0 img_h/pi 0
        0 0 1];
Pinv = Knew^-1;    

for y_u=1:img_h
    for x_u=1:img_w

        spherical = Pinv*[x_u y_u 1]';

        theta= spherical(1);  % longitude
        phi = spherical(2);   % latitude
        
        x_p = sin(theta)*sin(phi);
        y_p = -cos(phi);        
        z_p = cos(theta)*sin(phi);        
        
        xu = x_p / (z_p + xi); % x coordinate in the camera frame
        yu = y_p / (z_p + xi); % y coordinate in the camera frame       
        
        sv_r = (1 + k0*(xu^2 + yu^2) + k1*(xu^2 + yu^2)^2)*xu + 2*p0*xu*yu + p1*(3*xu^2 + yu^2);
        sv_t = (1 + k0*(xu^2 + yu^2) + k1*(xu^2 + yu^2)^2)*yu + p0*(xu^2 + 3*yu^2) + 2*p1*xu*yu;        
          
        sv_sq = sv_r^2 + sv_t^2;
        
        if (sv_sq < radius)            
            xq(y_u,x_u) = sv_r * sv_u + s * sv_t + sv_x + 1;
            yq(y_u,x_u) = sv_t * sv_v + sv_y + 1;
        end
    end
end

img_undistort1(:,:,1) = interp2(x,y,double(img1(:,:,1)),xq,yq);
img_undistort1(:,:,2) = interp2(x,y,double(img1(:,:,2)),xq,yq);
img_undistort1(:,:,3) = interp2(x,y,double(img1(:,:,3)),xq,yq);

img_undistort=uint8(img_undistort1);
 
figure;
imshow (img_undistort);
hold on;

% Check the correspondance of the mesured physical 3D points

y = -6.0;

% Check point on the image;
for x3d=-16.9:0.5:17.1   
    y3d = y; 
    z3d = 1.85;
    
    [xq, yq] = projectPointEqui (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');
end

for x3d=-2:0.5:2
    y3d = y;
    z3d = 31.3;
    
    [xq, yq] = projectPointEqui (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');
end

for x3d=-2:0.5:2
    y3d = y + 0.5;
    z3d = 5;
    
    [xq, yq] = projectPointEqui (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');
end

for z3d = 5.5:0.5:31.3
    y3d = y;
    x3d = 2;
    
    [xq, yq] = projectPointEqui (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');    
end

for z3d = 5.5:0.5:31.3
    y3d = y;
    x3d = -2;
    
    [xq, yq] = projectPointEqui (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');    
end



