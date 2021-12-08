function OptimizeViaQp()
warning off
global params_
params_.em.intermediate_optima = [];
tic
while (1)
    WriteFilesForEmPlanner();
    !ampl rem.run
    LoadEmSolution();
    elem.x = params_.em_x; elem.y = params_.em_y;
    params_.em.intermediate_optima = [params_.em.intermediate_optima, elem];
    [~, kappa_em] = ComputeCurvature(params_.em_x, params_.em_y);
    ind = find(abs(kappa_em) > params_.vehicle.kappa_max, 1);
    if (isempty(ind))
        break;
    else
        params_.em.weight_smoothness = params_.em.weight_smoothness * 2.0;
    end
    if (toc > 20)
        return;
    end
end
end

function WriteFilesForEmPlanner()
global params_
dp_x = params_.dp_x(1);
dp_y = params_.dp_y(1);
for ii = 2 : length(params_.dp_x)
    seg_length = hypot(params_.dp_x(ii) - params_.dp_x(ii-1), params_.dp_y(ii) - params_.dp_y(ii-1));
    nfe = round(seg_length / 0.001);
    list_x = linspace(params_.dp_x(ii-1), params_.dp_x(ii), nfe);
    dp_x = [dp_x, list_x(2:end)];
    list_y = linspace(params_.dp_y(ii-1), params_.dp_y(ii), nfe);
    dp_y = [dp_y, list_y(2:end)];
end
index = round(linspace(1, length(dp_y), params_.opti.nfe));
dp_x = dp_x(index);
dp_y = dp_y(index);

dp_s = [];
dp_l = [];
for ii = 1 : params_.opti.nfe
    [s, l] = XY2SL(dp_x(ii), dp_y(ii));
    dp_s = [dp_s, s];
    dp_l = [dp_l, l];
end
[~, ia, ~] = unique(dp_s);
dp_s = dp_s(ia);
dp_l = dp_l(ia);

em_s = linspace(params_.scene.referenceline.s(1), params_.scene.referenceline.s(params_.opti.nfe), params_.opti.nfe);
em_l = interp1(dp_s, dp_l, em_s);
em_ub = interp1(params_.scene.referenceline.s, params_.scene.referenceline.ub, em_s);
em_lb = interp1(params_.scene.referenceline.s, params_.scene.referenceline.lb, em_s);

global em_temp_space_
em_temp_space_.s = em_s;
em_temp_space_.lw = em_l;
em_temp_space_.ub = em_ub;
em_temp_space_.lb = em_lb;

delete('EM_task');
fid = fopen('EM_task', 'w');
for ii = 1 : params_.opti.nfe
    fprintf(fid, '%g 1 %f \r\n', ii, em_l(ii));
    fprintf(fid, '%g 2 %f \r\n', ii, em_ub(ii));
    fprintf(fid, '%g 3 %f \r\n', ii, em_lb(ii));
end
fclose(fid);

s_max = em_s(end) - em_s(1);
ds = s_max / (params_.opti.nfe - 1);
nr = round(params_.vehicle.lr / ds);
nf = round((params_.vehicle.lf + params_.vehicle.lw) / ds);
w = params_.vehicle.lb;
[~, l0] = XY2SL(params_.task.x0, params_.task.y0);
[~, lf] = XY2SL(params_.task.xf, params_.task.yf);
delete('Em_params');
fid = fopen('Em_params', 'w');
fprintf(fid, '1 %f \r\n', nr);
fprintf(fid, '2 %f \r\n', nf);
fprintf(fid, '3 %f \r\n', s_max);
fprintf(fid, '4 %f \r\n', w);
fprintf(fid, '5 %f \r\n', l0);
fprintf(fid, '6 %f \r\n', lf);
fprintf(fid, '7 %f \r\n', params_.em.weight_smoothness);
fclose(fid);
end

function LoadEmSolution()
global params_ em_temp_space_
load l.txt
params_.em_x = zeros(1, params_.opti.nfe);
params_.em_y = zeros(1, params_.opti.nfe);
for ii = 1 : params_.opti.nfe
    [params_.em_x(ii), params_.em_y(ii)] = SL2XY(em_temp_space_.s(ii), l(ii));
end
end

function [s, kappa] = ComputeCurvature(x, y)
nfe = length(x);
kappa = zeros(1, nfe);
s = 0;
incremental_s = 0;
for ii = 2 : nfe
    incremental_s = incremental_s + hypot(x(ii) - x(ii-1), y(ii) - y(ii-1));
    s = [s, incremental_s];
end

for ii = 2 : (nfe - 1)
    x_tuple = x((ii-1) : (ii+1));
    y_tuple = y((ii-1) : (ii+1));
    kappa(ii) = ComputeCurvatureViaThreePoints(x_tuple, y_tuple);
end
kappa(1) = kappa(2);
kappa(nfe) = kappa(nfe-1);
end

function kappa = ComputeCurvatureViaThreePoints(xx, yy)
x = reshape(xx, 3, 1);
y = reshape(yy, 3, 1);
t_a = norm([x(2) - x(1), y(2) - y(1)]);
t_b = norm([x(3) - x(2), y(3) - y(2)]);
M =[[1, -t_a, t_a^2];
    [1, 0,    0    ];
    [1,  t_b, t_b^2]];
a = M \ x;
b = M \ y;
kappa = 2.0 .* (b(3) * a(2) - a(3) * b(2)) / (a(2)^2 + b(2)^2)^1.5;
end