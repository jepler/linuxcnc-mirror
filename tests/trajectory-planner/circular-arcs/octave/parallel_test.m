% Demonstrate dot product of unit vectors in various directions as a sanity check
theta=0:pi/36:pi;
u0=[1;0;0];
u1=[cos(theta);sin(theta);0*theta];

udot = u0'*u1;
plot(udot)
title(sprintf('Dot product of unit vectors as angle varies from %0.3f to %0.3f', theta(1), theta(end)))
xlabel('Angle, rad')
grid on
