d=csvread('spindle_data.txt');

d(:,5)=d(:,5)*1000;
% Spindle data
figure(1)
plot(d(:,1:5),'Linewidth',2);

grid on
legend('spindle displacement','raw pos', 'pos desired', 'pos progress', 'pos_error')

% Position error
figure(2)
plot(d(:,5),'Linewidth',2)
grid on
legend('pos error')