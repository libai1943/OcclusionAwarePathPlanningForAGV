function InitializeParams()
global params_
params_.task.maximum_height = 4;
params_.task.ceiling_height = 5;
params_.task.num_box_in_each_axle = 10;
params_.task.num_grids_per_box = 7;
params_.task.num_receivers = 8;

params_.demo.xmin = 0;
params_.demo.xmax = 50;
params_.demo.ymin = 0;
params_.demo.ymax = 50;
params_.demo.colorpool = [237,28,36; 0,162,232; 255,127,39; 218,112,214; 255,192,203; 123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;

params_.astar.num_grids = params_.task.num_grids_per_box * params_.task.num_box_in_each_axle;
params_.astar.dx = (params_.demo.xmax - params_.demo.xmin) / params_.astar.num_grids;
params_.astar.dy = (params_.demo.ymax - params_.demo.ymin) / params_.astar.num_grids;

params_.vehicle.height = 0.25; % vehicle height
params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lf + params_.vehicle.lw + params_.vehicle.lr; % length
params_.vehicle.phymax = 0.85;
params_.vehicle.wmax = 1.5;
params_.vehicle.kappa_max = tan(params_.vehicle.phymax) / params_.vehicle.lw;

params_.dp.ns = 80;
params_.dp.nl = 6;
params_.dp.w_occlusion_awareness = 100;
params_.opti.nfe = 200;
params_.em.weight_smoothness = 0.0001;
end