function [NN] = inicialize_adp2(numIpt)
%clear;
%clc;

% load NN_Init3_good.mat;
NN=[];

NN.WA_Inputs = numIpt;
NN.Num_u = 1;  %NB number of the u(control variables)
% WR_Inputs = WA_Inputs + Num_u;  % the inputs of the RNN
NN.WC_Inputs = NN.WA_Inputs + NN.Num_u; % the inputs of the CNN ,include a additional s signal for reference

% were all 6 initially
NN.NC_Hidden=14;  % Number of nodes in hidden layer(CNN)
NN.NA_Hidden=12;  % Number of nodes in hidden layer(ANN)

Num = 1; % was 10 initially

NN.wc1=(rand(NN.WC_Inputs,NN.NC_Hidden)-0.5)*2/Num; % wc_inputs by nc_hidden dimension of weights (input to hidden)
NN.wc2=(rand(NN.NC_Hidden,1)-0.5)*2/Num; % nc_hidden by 1 dimension of weights (hiddon to output)

NN.wa1=(rand(NN.WA_Inputs,NN.NA_Hidden)-0.5)*2/Num;
%NN.wa1=(rand(WA_Inputs,NA_Hidden)-1)*2/Num;
NN.wa2=(rand(NN.NA_Hidden,NN.Num_u)-0.5)*2/Num;
%NN.wa2=(rand(NA_Hidden,Num_u)-1)*2/Num;

% naction = 8; % 1st:Up, 2nd:Down, 3rd:Left, 4th:Right, 5th:UpLeft, 6th:UpRight, 7th: DNLeft, 8th:DNRight
% ncols = 5;   % cloumn number : max num of blank in each row
% nrows = 11;   % row number: max num of blank in each cloumn
% nstates = ncols*nrows; % num of states
% 
% statevector = [];
% 
% for i=1:nrows % actually refer to y
%     for j=1:ncols % actually refer to x
%         temp = [j i];
%         statevector = [statevector; temp];
%     end
% end
% 
% Qtable = [statevector zeros(size(statevector,1), naction)];
% Qshift = size(statevector,2);
% 
% D = ones(nstates,1)/nstates;
% num_sel = 1;
% 
% % Qtable = [statevector zeros(size(statevector,1), naction)];
% % Qshift = size(statevector,2);
% %% Q related parameters added here
% QPARA=[];
% QPARA.Qcounter = zeros(size(statevector,1), naction);
% QPARA.countMax = 15;
% 
% QPARA.naction=naction;
% QPARA.ncols=ncols;
% QPARA.nrows=nrows;
% QPARA.nstates=nstates;
% 
% QPARA.Qtable = Qtable;
% QPARA.Qshift = Qshift;
% QPARA.statevector = statevector;
% QPARA.D= ones(nstates,1)/nstates;
% QPARA.num_sel= num_sel;
%% end



NN.Jprev = 0;

NN.wc1hist = NN.wc1(:,1)';
NN.wa1hist = NN.wa1(:,1)';

NN.wc2hist = NN.wc2(:,1)';
NN.wa2hist = NN.wa2(:,1)';

NN.reinfhist = [];
NN.Jhist = [];

NN.errorchist = [];
NN.errorahist = [];

% load inipara.mat;

