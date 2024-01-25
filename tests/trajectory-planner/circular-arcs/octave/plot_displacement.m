d=csvread('displacement_data.txt');

t = d(:,1);
xyz=d(:,2:4);
figure(1)
plot(t, xyz,'Linewidth',2);

grid on
legend('X','Y','Z')

