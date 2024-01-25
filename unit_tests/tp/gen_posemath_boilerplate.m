%% Generate arithmetic test examples for Posemath library
% Suitable for copy/paste into a unit test
s = (rand(3,1)-0.5)*rand(1)*10;
e = (rand(3,1)-0.5)*rand(1)*10;
k = (rand(1)-0.5)*2.0;


fprintf('    static const PmCartesian v1 = {%0.17g, %0.17g, %0.17g};\n', s);
fprintf('    static const PmCartesian v2 = {%0.17g, %0.17g, %0.17g};\n', e);
fprintf('    static const double k = %0.17g;\n', k);

fprintf('    static const double mag_v1 = %0.17g;\n',norm(s));
fprintf('    static const double mag_v2 = %0.17g;\n',norm(e));
fprintf('    static const double mag_diff = %0.17g;\n',sqrt(sum((e-s).^2)));
fprintf('    static const double dot = %0.17g;\n', e'*s);
fprintf('    static const PmCartesian cross = {%0.17g, %0.17g, %0.17g};\n', cross(s,e));
fprintf('    static const PmCartesian diff = {%0.17g, %0.17g, %0.17g};\n', e-s);
fprintf('    static const PmCartesian sum = {%0.17g, %0.17g, %0.17g};\n', e+s);
fprintf('    static const PmCartesian v1_neg = {%0.17g, %0.17g, %0.17g};\n', -s);
fprintf('    static const PmCartesian v1_mult_k = {%0.17g, %0.17g, %0.17g};\n', s*k);
fprintf('    static const PmCartesian v1_div_k = {%0.17g, %0.17g, %0.17g};\n', s/k);
fprintf('    static const PmCartesian elem_mult = {%0.17g, %0.17g, %0.17g};\n', s.*e);
fprintf('    static const PmCartesian elem_div = {%0.17g, %0.17g, %0.17g};\n', s./e);