function Quad_dynamics(block)
setup(block);

function setup(block)

  % Register the number of ports.
  %------
  block.NumInputPorts  = 4;
  %------
  block.NumOutputPorts = 12;
  
  % Set up the port properties to be inherited or dynamic.
  
  for i = 1:4 % thetase are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
  %------
  %------
  for i = 1:12
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

%  % Register the parameters.
  block.NumDialogPrms     = 1;
  
  % Set up the continuous states.
  block.NumContStates = 12;

  block.SampleTimes = [0 0];
  
  
  block.SetAccelRunOnTLC(false);
  

  block.SimStateCompliance = 'DefaultSimState';
  
   block.RegBlockMethod('CheckParameters', @CheckPrms);


  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  

  %
  block.RegBlockMethod('Outputs', @Outputs);


  %
  block.RegBlockMethod('Derivatives', @Derivatives);


 function CheckPrms(block)
     %quad   = block.DialogPrm(1).Data;
     CS     = block.DialogPrm(1).Data;

function InitializeConditions(block)
% Initialize 12 States

CS = block.DialogPrm(1).Data;

% CS.P, CS.Q, CS.R are in deg/s ... convert to rad/s
P = CS.p*pi/180; Q = CS.q*pi/180; R = CS.r*pi/180; 
% CS.Phi, CS.theta, CS.psi are in deg ... convert to rads
Phi = CS.phi*pi/180; theta = CS.theta*pi/180; psi = CS.psi*pi/180;
U = CS.u; V = CS.v; W = CS.w; 
X = CS.X; Y = CS.Y; Z = CS.Z;

init = [P,Q,R,Phi,theta,psi,U,V,W,X,Y,Z];
for i=1:12
block.OutputPort(i).Data = init(i);
block.ContStates.Data(i) = init(i);
end

function Outputs(block)
for i = 1:12
  block.OutputPort(i).Data = block.ContStates.Data(i);
end


function Derivatives(block)
% Name all the states and motor inputs

% Load model data selected in parameter block
%which('model')

CS = block.DialogPrm(1).Data;

% P Q R in units of rad/sec
P = block.ContStates.Data(1);
Q = block.ContStates.Data(2);
R = block.ContStates.Data(3);
% Phi theta psi in radians
Phi = block.ContStates.Data(4);
theta = block.ContStates.Data(5);
psi = block.ContStates.Data(6);
% U V W in units of m/s
U = block.ContStates.Data(7);
V = block.ContStates.Data(8);
W = block.ContStates.Data(9);
% X Y Z in units of m
X = block.ContStates.Data(10);
Y = block.ContStates.Data(11);
Z = block.ContStates.Data(12);

% w values in rev/min! NOT radians/s!!!!
w1 = block.InputPort(1).Data;
w2 = block.InputPort(2).Data;
w3 = block.InputPort(3).Data;
w4 = block.InputPort(4).Data;
w  = [w1; w2; w3; w4];   %converting RRM to rad/sec


%%% Kinematics
T_rot = [1,tan(theta)*sin(Phi), tan(theta)*cos(Phi);
    0,         cos(Phi),         -sin(Phi);
    0,sin(Phi)/cos(theta),cos(Phi)/cos(theta)]; %Transformation matrix from body to inertial

R_rot = [cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(Phi)-sin(psi)*cos(Phi) cos(psi)*sin(theta)*cos(Phi)+sin(psi)*sin(Phi);
       sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(Phi)+cos(psi)*cos(Phi) sin(psi)*sin(theta)*cos(Phi)-cos(psi)*sin(Phi);
       -sin(theta)         cos(theta)*sin(Phi)                            cos(theta)*cos(Phi)];

%%% Body Frame Forces and Moments
Eb = [CS.b; CS.b * CS.l; CS.b * CS.l; CS.d] .* (CS.Mix'  * w.^2);
U1 = Eb(1); U2 = Eb(2); U3 = Eb(3); U4 = Eb(4); %Finding U1, U2, U3, U4

%%%%%%
Omega = CS.motor_orientation * w;               %Overall propeller's speed
%%%%%%%

M_b =  diag([CS.m, CS.m, CS.m, CS.Ixx, CS.Iyy, CS.Izz]);  %System inertia matrix

%%%%%%%%% Coriolis-centripetal Effect
C_b = zeros(6,6);       %Coriolis-centripetal Matrix

C_b(1:3,4:6) = [0, CS.m * W, -CS.m * V;
    -CS.m * W, 0, CS.m * U;
    CS.m * V, -CS.m * U, 0];

C_b(4:6,4:6) = [0, CS.Izz * R, -CS.Iyy * Q;
    -CS.Izz * R, 0, CS.Ixx * P;
    CS.Iyy * Q, -CS.Ixx * P, 0];

%%%%%%%%%%%%%%% Gyroscopic Effect
O_b_moment = zeros(6,1);

O_b_moment(4:6,1) = 2*pi/60 * CS.Jm * [-P; Q; 0] * Omega;

%%%%%%%%%%  Gravitional Effect
G_b = [R_rot' * [0;0;-CS.m*9.81]; zeros(3,1)];  %Gravity in Body Frame

%%%%%%%%% Movement Moment
U_b = [zeros(2,1);      %Movement Moment
    U1;
    U2;
    U3;
    U4];
        
%%%%%%% Solution of EoM
x = [U;V;W;P;Q;R];

dot_x = M_b \ (-C_b * x + G_b + U_b + O_b_moment);   %Derrivatives of States

%%% Transformation ~~ Kinematics
J_rot = zeros(6,6);
J_rot(1:3, 1:3) = R_rot;
J_rot(4:6, 4:6) = T_rot;

d_x = J_rot * x;

dP = dot_x(4);
dQ = dot_x(5);
dR = dot_x(6);
dPhi = d_x(4);
dthetata = d_x(5);
dpsi = d_x(6);
dU = dot_x(1);
dV = dot_x(2);
dW = dot_x(3);
dX = d_x(1);
dY = d_x(2);
dZ = d_x(3);

% Rough rule to impose a "ground" boundary...could easily be improved...
if ((Z<=0) && (dZ<=0)) % better  version then before?
    dZ = 0;
    block.ContStates.Data(12) = 0;
end
f = [dP dQ dR dPhi dthetata dpsi dU dV dW dX dY dZ].';
  %This is the state derivative vector
block.Derivatives.Data = f;
