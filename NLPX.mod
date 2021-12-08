param BasicParameters{i in {1..11}};
param Nfe == BasicParameters[1];
param box_scale == BasicParameters[2];
param x0 == BasicParameters[3];
param y0 == BasicParameters[4];
param xf == BasicParameters[5];
param yf == BasicParameters[6];
param theta0 == BasicParameters[7];
param thetaf == BasicParameters[8];
param lw == BasicParameters[9];
param phymax == BasicParameters[10];
param wmax == BasicParameters[11];

var tf >= 10;
var hi = tf / (Nfe - 1);
set I := {1..Nfe};
param Waypoints{i in {1..Nfe}, j in {1..2}};

var z{i in {1..(8*Nfe)}};

minimize obj:
tf + 100 * sum{i in {1..Nfe}}(z[7*Nfe+i]^2 + (z[6*Nfe+i]^2)) +
sum{i in {3..(Nfe-2)}}((z[i] - Waypoints[i,1])^2 + (z[Nfe + i] - Waypoints[i,2])^2);


s.t. FirstODE {i in {2..Nfe}}:
z[i] - z[i-1] - hi * z[3*Nfe+i-1] * cos(z[2*Nfe+i-1]) = 0;

s.t. SecondODE {i in {2..Nfe}}:
z[Nfe+i] - z[Nfe+i-1] - hi * z[3*Nfe+i-1] * sin(z[2*Nfe+i-1]) = 0;

s.t. ThirdODE {i in {2..Nfe}}:
z[3*Nfe+i] - z[3*Nfe+i-1] - hi * z[4*Nfe+i-1] = 0;

s.t. FourthODE {i in {2..Nfe}}:
z[2*Nfe+i] - z[2*Nfe+i-1] - hi * z[3*Nfe+i-1] * tan(z[5*Nfe+i-1]) / lw = 0;

s.t. FifthODE {i in {2..Nfe}}:
z[5*Nfe+i] - z[5*Nfe+i-1] - hi * z[6*Nfe+i-1] = 0;

s.t. SixthODE {i in {2..Nfe}}:
z[4*Nfe+i] - z[4*Nfe+i-1] - hi * z[7*Nfe+i-1] = 0;


s.t. Bonds_phy {i in I}:
-phymax <= z[5*Nfe+i] <= phymax;
s.t. Bonds_v {i in I}:
0 <= z[3*Nfe+i] <= 5.0;
s.t. Bonds_a {i in I}:
-2 <= z[4*Nfe+i] <= 2;
s.t. Bonds_jerk {i in I}:
-10 <= z[7*Nfe+i] <= 10;
s.t. Bonds_w {i in I}:
-wmax <= z[6*Nfe+i] <= wmax;

s.t. Bonds_x {i in I}:
Waypoints[i,1] - box_scale <= z[i] <= Waypoints[i,1] + box_scale;
s.t. Bonds_y {i in I}:
Waypoints[i,2] - box_scale <= z[Nfe+i] <= Waypoints[i,2] + box_scale;

data;
param: Waypoints := include Waypoints;
param: BasicParameters := include BasicParameters;