d=csvread('tc_state_data.txt');

figure(1)
plot(d(:,[1:5,8]),'Linewidth',2);

grid on
legend('vr', 'vf','vmax','v_current','acc')
