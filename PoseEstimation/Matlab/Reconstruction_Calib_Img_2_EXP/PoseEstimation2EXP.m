I1=imread('img_equi_0_0.bmp');
Ig1 = rgb2gray(I1);

P1 = [627 516;
    729 467;
    858 421;
    1023 381;
    1227 354;
   1458 344;
   1694 352;
   1905 377;
   2081 416;
   641 565;
   742 515;
   869 467;
   1031 426;
   1230 399;
   1452 388;
   1680 395;
   1886 421;
   2061 460;
   654 619;
   753 569;
   879 520;
   1040 479;
   1233 452;
   1447 440;
   1666 445;
   1869 471;
   2042 511;
   668 682;
   767 632;
   890 582;
   1049 542;
   1235 514;
   1441 500;
   1654 506;
   1851 533;
   2020 573;
   680 753;
   778 703;
   902 656;
   1057 615;
   1238 586;
   1436 572;
   1643 578;
   1834 605;
   2002 645;
   692 835;
   789 787;
   913 741;
   1065 702;
   1239 672;
   1432 659;
   1632 666;
   1819 691;
   1984 730];

figure; 
imshow (I1);
hold on;
plot (P1(:,1),P1(:,2),'-r');
plot (P1(:,1),P1(:,2),'xg');
plot (P1(1,1),P1(1,2),'og');  
   
I2 = imread('img_equi_0_1.bmp');
Ig2 = rgb2gray(I2);

P2 = [1136 729;
    1212 716;
    1293 705;
    1378 698;
    1465 695;
    1553 696;
    1640 701;
    1725 710;
    1807 722;
    1136 772;
    1212 759;
    1291 749;
    1376 741;
    1463 739;
    1549 740;
    1637 745;
    1721 753;
    1803 766;
    1136 818;
    1211 805;
    1291 795;
    1374 789;
    1460 787;
    1546 788;
    1634 792;
    1717 800;
    1799 812;
    1137 869;
    1211 856;
    1290 846;
    1373 840;
    1458 839;
    1544 839;
    1630 843;
    1713 852;
    1794 863;
    1136 923;
    1211 911;
    1289 902;
    1372 897;
    1456 895;
    1541 895;
    1627 900;
    1709 908;
    1789 918;
    1137 981;
    1210 969;
    1288 962;
    1371 957;
    1454 955;
    1538 956;
    1623 961;
    1705 968;
    1785 978];

figure; 
imshow (I2);
hold on;
plot (P2(:,1),P2(:,2),'-r');
plot (P2(:,1),P2(:,2),'xg');
plot (P2(1,1),P2(1,2),'og');  
   
I3 = imread('img_equi_0_2.bmp');
Ig3 = rgb2gray(I3);

P3 = [1295 975;
    1339 972;
    1383 970;
    1429 968;
    1475 969;
    1520 969;
    1566 972;
    1611 974;
    1656 978;
    1294 1010;
    1338 1007;
    1382 1004;
    1428 1003;
    1474 1003;
    1519 1004;
    1565 1006;
    1611 1009;
    1655 1012;
    1293 1044;
    1337 1042;
    1382 1039;
    1427 1038;
    1473 1039;
    1518 1040;
    1564 1041;
    1609 1043;
    1654 1047;
    1293 1081;
    1336 1078;
    1381 1076;
    1426 1076;
    1472 1076;
    1517 1077;
    1563 1078;
    1608 1081;
    1652 1084;
    1293 1118;
    1336 1116;
    1380 1114;
    1425 1114;
    1471 1114;
    1516 1115;
    1562 1117;
    1607 1119;
    1651 1122;
    1292 1157;
    1335 1155;
    1379 1154;
    1425 1154;
    1470 1154;
    1515 1155;
    1560 1157;
    1605 1158;
    1650 1161];

figure; 
imshow (I3);
hold on;
plot (P3(:,1),P3(:,2),'-r');  
plot (P3(:,1),P3(:,2),'xg');  
plot (P3(1,1),P3(1,2),'og');      
    
%------------------------------------
w = 3008;
h = 3008;
Knew = [ w/pi 0 w/2;
    0 h/pi 0
    0 0 1];
Pinv = Knew^-1;

Pp1 = zeros(size(P1));
for i=1:size(P1,1)
    x = P1(i,1);
    y = P1(i,2);
    A = Pinv * [x y 1]';
    theta = A(1,1);
    phi = A(2,1);
    Pp1(i,:) = [theta phi];
end
    
Pp2 = zeros(size(P2));
for i=1:size(P2,1)
    x = P2(i,1);
    y = P2(i,2);
    A = Pinv * [x y 1]';
    theta = A(1,1);
    phi = A(2,1);
    Pp2(i,:) = [theta phi];
end
    
Pp3 = zeros(size(P3));
for i=1:size(P3,1)
    x = P3(i,1);
    y = P3(i,2);
    A = Pinv * [x y 1]';
    theta = A(1,1);
    phi = A(2,1);
    Pp3(i,:) = [theta phi];
end

figure;
plot(Pp1(:,1),Pp1(:,2),'r.');
hold on
plot(Pp1(:,1),Pp1(:,2),'-b');
plot(Pp1(1,1),Pp1(1,2),'og');
xlabel('theta');
ylabel('phi');
plot(Pp2(:,1),Pp2(:,2),'r.');
plot(Pp2(:,1),Pp2(:,2),'-b');
plot(Pp2(1,1),Pp2(1,2),'og');
plot(Pp3(:,1),Pp3(:,2),'r.');
plot(Pp3(:,1),Pp3(:,2),'-b');
plot(Pp3(1,1),Pp3(1,2),'og');
axis([-pi/2 pi/2 0 pi]);
grid;
    
%-------------------------------

p3d_1 = zeros(size(Pp1,1),3);
for i=1:size(Pp1,1)
    theta = Pp1(i,1);
    phi = Pp1(i,2);
    xs = sin(theta)*sin(phi);
    ys = cos(phi);
    zs = cos(theta)*sin(phi);
    p3d_1(i,:) = [xs ys zs];
end

p3d_2 = zeros(size(Pp2,1),3);
for i=1:size(Pp2,1)
    theta = Pp2(i,1);
    phi = Pp2(i,2);
    xs = sin(theta)*sin(phi);
    ys = cos(phi);
    zs = cos(theta)*sin(phi);
    p3d_2(i,:) = [xs ys zs];
end

p3d_3 = zeros(size(Pp3,1),3);
for i=1:size(Pp3,1)
    theta = Pp3(i,1);
    phi = Pp3(i,2);
    xs = sin(theta)*sin(phi);
    ys = cos(phi);
    zs = cos(theta)*sin(phi);
    p3d_3(i,:) = [xs ys zs];
end

% Plot the projected
figure;
plot3(p3d_1(:,1),p3d_1(:,2),p3d_1(:,3),'r.');
xlabel('x');
ylabel('y');
zlabel('z');
hold on
plot3(p3d_1(:,1),p3d_1(:,2),p3d_1(:,3),'-b');
plot3(p3d_1(1,1),p3d_1(1,2),p3d_1(1,3),'og');

plot3(p3d_2(:,1),p3d_2(:,2),p3d_2(:,3),'g.');
plot3(p3d_2(:,1),p3d_2(:,2),p3d_2(:,3),'-b');
plot3(p3d_2(1,1),p3d_2(1,2),p3d_2(1,3),'og');

plot3(p3d_3(:,1),p3d_3(:,2),p3d_3(:,3),'b.');
plot3(p3d_3(:,1),p3d_3(:,2),p3d_3(:,3),'-b');
plot3(p3d_3(1,1),p3d_3(1,2),p3d_3(1,3),'og');
axis([-1 1 -1 1 -1 1]);
grid;

    
% Reconstruct the scene
[sv_scene, sv_r, sv_t] = pose_estimation( p3d_1, p3d_2, p3d_3, 50 );
figure;
plot3(sv_scene(:,1), sv_scene(:,2), sv_scene(:,3),'b.');
grid;
    
        
    
    
    
    
   
   
   
   
   
   