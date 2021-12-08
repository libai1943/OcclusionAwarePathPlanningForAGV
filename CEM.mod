param BasicParameters{i in {1..12}};
param NE == BasicParameters[1];
param Em_params{j in {1..7}};
param Nr = Em_params[1];
param Nf = Em_params[2];
param max_s = Em_params[3];
param w = Em_params[4];
param l0 = Em_params[5];
param lfff = Em_params[6];
param smoothness_weight_parameter = Em_params[7];
param EM_task{i in {1..NE}, j in {1..3}};
param ds = max_s / (NE - 1);
param lf = ds * Nf;
param lr = ds * Nr;

var l{i in {1..NE}};
var dl{i in {1..NE}};
var ddl{i in {1..NE}};
var dddl{i in {1..NE}};

######### Optimization Objective #########
minimize cost_function:
sum{i in {1..NE}}((l[i] - EM_task[i,1])^2) + smoothness_weight_parameter * sum{i in {1..NE}}(10 * dl[i]^2 + 100 * ddl[i]^2 + 10000 * dddl[i]^2);
 

s.t. intgrate_dddl {i in {2..NE}}:
ddl[i] = ddl[i-1] + dddl[i] * ds;

s.t. intgrate_ddl {i in {2..NE}}:
dl[i] = dl[i-1] + ddl[i-1] * ds + 0.5 * (ds^2) * dddl[i];

s.t. intgrate_dl {i in {2..NE}}:
l[i] = l[i-1] + dl[i-1] * ds + 0.5 * (ds^2) * ddl[i-1] + (ds^3) * dddl[i] / 6;

s.t. Previous_dddl0:
dddl[1] = 0;
s.t. Previous_ddl0:
ddl[1] = 0;
s.t. Previous_l0:
l[1] = l0;

s.t. Final_dddl:
dddl[NE] = 0;
s.t. Final_ddl:
ddl[NE] = 0;
s.t. Final_dl:
dl[NE] = 0;
s.t. Final_l:
l[NE] = lfff;

######### Collision-Avoidance #########
s.t. front_part_ego_vehicle_body {i in {1..(NE - Nf)}, j in {0..Nf}}:
EM_task[i+j,3] + 0.5 * w <= l[i] + lf * dl[i] * j / Nf <= EM_task[i+j,2] - 0.5 * w;

s.t. rear_part_ego_vehicle_body {i in {(Nr+1)..NE}, j in {1..Nr}}:
EM_task[i-j,3] + 0.5 * w <= l[i] - lr * dl[i] * j / Nr <= EM_task[i-j,2] - 0.5 * w;


#s.t. bound_dl {i in {1..NE}}:
#-0.2 * ds <= dl[i] <= 0.2 * ds;

#s.t. bound_ddl {i in {1..NE}}:
#-0.1 * ds <= ddl[i] <= 0.1 * ds;

#s.t. bound_dddl {i in {1..NE}}:
#-0.01 * ds <= dddl[i] <= 0.01 * ds;

data;
param Em_params:= include Em_params;
param EM_task:= include EM_task;
param BasicParameters := include BasicParameters;