function cost = SearchViaDp()
global state_space params_ dp
NS = params_.dp.ns;
NL = params_.dp.nl;

dp.ns_id_list = round(linspace(1, params_.opti.nfe, NS));
% 记录的字段
% cost
% parent_id
% cur_config
% parent_config

state_space(1, 1).cost = Inf;
state_space(1, 1).parent_id = [0, -999];
state_space(1, 1).cur_config = [999, 999];
state_space(1, 1).parent_config = [999, 999];
state_space(NS, NL).cost = Inf;
for jj = 1 : NS
    for kk = 1 : NL
        state_space(jj, kk).cost = Inf;
    end
end

% % Enumeration
cur_l_min = params_.scene.referenceline.lb(dp.ns_id_list(1));
cur_l_max = params_.scene.referenceline.ub(dp.ns_id_list(1));
cur_l_list = linspace(cur_l_min, cur_l_max, NL);
for kk = 1 : NL
    cur_s = params_.scene.referenceline.s(dp.ns_id_list(1));
    cur_l = cur_l_list(kk);
    state_space(1, kk).parent_config = [params_.task.x0, params_.task.y0];
    [x, y] = SL2XY(cur_s, cur_l);
    state_space(1, kk).cur_config = [x, y];
    state_space(1, kk).cost = CalculateCost(cur_l, state_space(1, kk).parent_config, state_space(1, kk).cur_config);
    state_space(1, kk).parent_id = [0, -999];
end

for jj = 1 : (NS - 1)
    for kk = 1 : NL
        parent_node_ind = [jj, kk];
        parent_config = state_space(jj, kk).cur_config;
        cur_l_min = params_.scene.referenceline.lb(dp.ns_id_list(jj+1)) + params_.vehicle.lb / 2;
        cur_l_max = params_.scene.referenceline.ub(dp.ns_id_list(jj+1)) - params_.vehicle.lb / 2;
        cur_l_list = linspace(cur_l_min, cur_l_max, NL);
        for nn = 1 : NL
            cur_s = params_.scene.referenceline.s(dp.ns_id_list(jj+1));
            if (jj + 1 ~= NS)
                cur_l = cur_l_list(nn);
                [x, y] = SL2XY(cur_s, cur_l);
                state_space(jj+1, nn).cur_config = [x, y];
            else
                cur_l = 0;
                state_space(jj+1, nn).cur_config = [params_.task.xf, params_.task.yf];
            end
            delta_cost = CalculateCost(cur_l, parent_config, state_space(jj+1, nn).cur_config);
            cur_cost_candidate = state_space(jj, kk).cost + delta_cost;
            if (cur_cost_candidate < state_space(jj+1, nn).cost)
                state_space(jj+1, nn).cost = cur_cost_candidate;
                state_space(jj+1, nn).parent_id = parent_node_ind;
                state_space(jj+1, nn).parent_config = parent_config;
            end
        end
    end
end

% % Node backtrack
x = state_space(NS, 1).cur_config(1);
y = state_space(NS, 1).cur_config(2);
cur_id = state_space(NS, 1).parent_id(2);
%state_space(NS, 1).parent_id
for ii = (NS-1) : -1 : 1
    x = [state_space(ii, cur_id).cur_config(1), x];
    y = [state_space(ii, cur_id).cur_config(2), y];
    %state_space(ii, cur_id).parent_id
    cur_id = state_space(ii, cur_id).parent_id(2);
end
x = [params_.task.x0, x];
y = [params_.task.y0, y];
params_.dp_x = x;
params_.dp_y = y;
cost = state_space(NS, NL).cost;
end

function delta_cost = CalculateCost(cur_l, parent_config, cur_config)
global params_
delta_cost = norm(parent_config - cur_config) + abs(cur_l);
nfe = 10;
x = linspace(parent_config(1), cur_config(1), nfe);
y = linspace(parent_config(2), cur_config(2), nfe);
occluded_list = [];
for ii = 1 : nfe
    occluded_list = [occluded_list, ComputeOcclusionCost(x(ii), y(ii))];
end
delta_cost = delta_cost + (mean(occluded_list) + max(occluded_list)) * params_.dp.w_occlusion_awareness;
end

function cost = ComputeOcclusionCost(x, y)
global params_
number_of_valid_receivers = 0;
for ii = 1 : params_.task.num_receivers
    xs = params_.rec{ii}.x;
    ys = params_.rec{ii}.y;
    zs = params_.rec{ii}.z;
    if (IsEgoVehicleConnectedToReceiver(x, y, params_.vehicle.height, xs, ys, zs))
        number_of_valid_receivers = number_of_valid_receivers + 1;
    end
end
cost = 4 - min(number_of_valid_receivers, 4);
end

function is_connected = IsEgoVehicleConnectedToReceiver(x, y, z, xs, ys, zs)
is_connected = 0;
global params_
nfe = 50;
x_list = linspace(x, xs, nfe);
y_list = linspace(y, ys, nfe);
z_list = linspace(z, zs, nfe);
for kk = 1 : nfe
    x = x_list(kk);
    y = y_list(kk);
    z = z_list(kk);
    ii = max(ceil(x / 5), 1);
    jj = max(ceil(y / 5), 1);
    h_box = params_.terrain(ii, jj);
    if (h_box >= z)
        return;
    end
end
is_connected = 1;
end