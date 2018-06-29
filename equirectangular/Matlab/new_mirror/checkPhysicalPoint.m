I = imread('img1.bmp');
imshow (I);
hold on;

y = -6.0;

% Check point on the image;
for x3d=-16.9:0.5:17.1   
    y3d = y; 
    z3d = 1.85;
    
    [xq, yq] = projectPoint (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');
end

for x3d=-2:0.5:2
    y3d = y;
    z3d = 31.3;
    
    [xq, yq] = projectPoint (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');
end

for x3d=-2:0.5:2
    y3d = y + 0.5;
    z3d = 5;
    
    [xq, yq] = projectPoint (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');
end

for z3d = 5.5:0.5:31.3
    y3d = y;
    x3d = 2;
    
    [xq, yq] = projectPoint (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');    
end

for z3d = 5.5:0.5:31.3
    y3d = y;
    x3d = -2;
    
    [xq, yq] = projectPoint (x3d, y3d, z3d);
    
    plot (xq, yq, '.r');    
end
