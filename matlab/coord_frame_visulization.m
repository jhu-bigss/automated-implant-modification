function coord_frame_visulization(H)

T = H(1:3,4);
R = H(1:3,1:3);

color = {'r', 'g', 'b'};

for i = 1:3
    CoordEndPos =  T + R(:,i)*10;
    plot3([T(1),CoordEndPos(1)],[T(2),CoordEndPos(2)],[T(3),CoordEndPos(3)], 'color', color{i}, 'LineWidth', 2);hold on
end
axis equal;xlabel('x'); ylabel('y'); zlabel('z'); box on
xlim([-50, 50]);
ylim([-50, 50]);
zlim([-50, 50]);
end