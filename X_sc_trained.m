function [Y,Xf,Af] = X_sc_trained_4(X,~,~)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Generated by Neural Network Toolbox function genFunction, 06-Jul-2017 14:26:39.
%
% [Y] = myNeuralNetworkFunction(X,~,~) takes these arguments:
%
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = Qx4 matrix, input #1 at timestep ts.
%
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = Qx2 matrix, output #1 at timestep ts.
%
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-22.7612441096744;-22.8305230451392;-114.066309004053;-114.432365749345];
x1_step1.gain = [0.0439343295639521;0.0438010113926368;0.00876683052806127;0.00873878638662786];
x1_step1.ymin = -1;

% Layer 1
b1 = [-2.9939391165527711;-4.4977905077674079;0.88527668122822933;0.37578163918193735;0.65402550262686199;1.0001365242112468;-0.6252178309205324;-0.76404264338760786;-0.19566786513197262;2.9231437078669056];
IW1_1 = [1.6046772757660643 -2.3221854784844362 -0.77258389443004716 -0.56753654993757263;0.68306813563508129 -0.41284445516425899 1.9838994966225052 0.58590654527447117;-5.193491530695745 1.6230614405051091 -3.4040760800135486 0.84639231095326728;3.1633454223402024 -1.0181095188107756 -3.2156324043426241 -1.0785900847309422;-1.1593824024245571 -0.83182407638320677 -1.6854308997004885 -2.5934453757976006;2.2678541302952646 1.142081345119111 4.2757325421845085 0.51860319143267075;-2.2393525291208598 -5.1267649670109572 -1.9866463813080699 -3.5245293259681634;-0.99421470236143605 3.1199516145533299 -1.4410816514043581 4.4806612270734227;-1.2516543443555415 3.9719677803578159 -1.3640710854712004 4.0910704231921837;0.98390226113023971 1.8331709703396368 1.091392532588177 0.80437560772964189];

% Layer 2
b2 = [0.52654387398923741;-0.91245559599301562];
LW2_1 = [-0.68774812883079417 -0.20533164851490945 0.34478501138779394 -0.048286396162836046 -0.209807226373115 -0.53773504894246005 -0.29345108767112627 0.31232983941335679 -0.70191233104162964 -1.0628554681365854;-1.1454560458834875 0.25462752512897574 4.9865862712085747 0.094613025732848549 -3.7594675371325201 -5.0012759718670265 -5.4560530055846783 3.8306007154901121 -7.8582282739975167 0.11488587403064157];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [1;0.134228187919463];
y1_step1.xoffset = [1;0.1];

% ===== SIMULATION ========

% Format Input Arguments
isCellX = iscell(X);
if ~isCellX
    X = {X};
end

% Dimensions
TS = size(X,2); % timesteps
if ~isempty(X)
    Q = size(X{1},1); % samples/series
else
    Q = 0;
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS
    
    % Input 1
    X{1,ts} = X{1,ts}';
    Xp1 = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*Xp1);
    
    % Layer 2
    a2 = repmat(b2,1,Q) + LW2_1*a1;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a2,y1_step1);
    Y{1,ts} = Y{1,ts}';
end

% Final Delay States
Xf = cell(1,0);
Af = cell(2,0);

% Format Output Arguments
if ~isCellX
    Y = cell2mat(Y);
end
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
y = bsxfun(@minus,x,settings.xoffset);
y = bsxfun(@times,y,settings.gain);
y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
x = bsxfun(@minus,y,settings.ymin);
x = bsxfun(@rdivide,x,settings.gain);
x = bsxfun(@plus,x,settings.xoffset);
end