function [newAction,NN] = hdp_main2(x,r_flow,NN)
%% This code is for HDP algorithm. The comparison need to be presented
%% between HDP and GrHDP

K = 1.5; % control signal gain

Ncrit = 15; % maximum iteration steps
Nact  = 18;

% Ncrit = 8;
% Nact  = 12;

alpha = 0.95;
Tc = 1e-10;
Ta = 1e-10;
% Tr = 1e-7;

% were all 6 initially
NC_Hidden=NN.NC_Hidden;  % Number of nodes in hidden layer(CNN)
NA_Hidden=NN.NA_Hidden;  % Number of nodes in hidden layer(ANN)

Initla = 0.01; % initial learning rate
Initlc = 0.01; % initial learning rate
Init_msethrshld = 1; % initial mse threshold

num_step = 10;
num_drop = 5;
thrshld_drop = 2;
%%
Uc = 0;

WA_Inputs = NN.WA_Inputs;%10;
Num_u = NN.Num_u;  %NB number of the u(control variables)
WC_Inputs = NN.WC_Inputs; % the inputs of the CNN ,include a additional s signal for reference

Jprev = NN.Jprev;
wa1 = NN.wa1;
wa2 = NN.wa2;
wc1 = NN.wc1;
wc2 = NN.wc2;

newSt = x;
NF = 5*ones(1,size(newSt,2)); % scalar (1) %4
% NF = []; 
inputs = newSt./NF; % scaled inputs

% action network -> new control (eqn. 21 ~ 24)
ha = inputs*wa1;
g = (1 - exp(-ha))./(1 + exp(-ha));
va = g*wa2;
newAction = 1./(1 + exp(-va));
%newAction = 1./(1 + exp(-va));

% critical network -> cost function (eqn. 13 ~ 15)
inp=[inputs, newAction];
qc=inp*wc1;
p = (1 - exp(-qc))./(1 + exp(-qc));
J=p*wc2;

%================================%%
%          reinforcement         %%
%================================%%
reinf = 10*r_flow; %5 8 10

%===============================%%
% learning rate update scheme   %%
%===============================%%
if (isempty(NN.msethrshld_hist))
    la = Initla;
    lc = Initlc;
    msethrshld = Init_msethrshld;
else
    la = NN.la;
    lc = NN.lc;
    msethrshld = NN.msethrshld;    
end

   
%================================================%%
% internal weights updating cycles for critnet   %%
%================================================%%                   

     %% Goal Network Adaptation
            
            
            cyc = 0;
            ecrit = alpha*J-(Jprev-reinf); % this is raw equation
		    Ec = 0.5 * ecrit^2;
            Ec_hist = Ec;

            while (Ec>Tc && cyc<=Ncrit),

                gradEcJ=alpha*ecrit;
    				%----for the first layer(input to hidden layer)----------
                    gradqwc1 = [inputs'; newAction'];
                    for i=1:NC_Hidden,
                        gradJp = wc2(i);
                        gradpq = 0.5*(1-p(i)^2);
                        wc1(:,i) = wc1(:,i) - lc*gradEcJ*gradJp*gradpq*gradqwc1;
                    end
                
                    %----for the second layer(hidden layer to output)-----------
                    gradJwc2=p';
                    wc2 = wc2- lc*gradEcJ*gradJwc2;
                 
                    %----compute new  J----
                    inp=[inputs newAction];
                    qc=inp*wc1;
                    p = (1 - exp(-qc))./(1 + exp(-qc));
                    J=p*wc2;


                cyc = cyc +1;
                ecrit = alpha*J-(Jprev-reinf);
                Ec = 0.5 * ecrit^2;
                Ec_hist = [Ec_hist;Ec];
               
            end % end of "while (Ec>0.05 & cyc<=Ncrit)"
          
%             crit_cyc=[crit_cyc cyc];

            if length(Ec_hist) < Ncrit+2;
                Ec_hist = [Ec_hist; nan(Ncrit+2-length(Ec_hist),1)];
            end
                
            %normalization weights for critical network

            if (max(max(abs(wc1)))>1)
                wc1=wc1/max(max(abs(wc1)));
                disp('wc1 norm')
            end
            if max(max(abs(wc2)))>1
                wc2=wc2/max(max(abs(wc2)));
                disp('wc2 norm')
            end

      
            %%=============================================%%
            %% internal weights updating cycles for actnet %%
            %%=============================================%%                
            cyc = 0;
            
            eact = (J - Uc);
            Ea = 0.5*eact^2;
            Ea_hist = Ea;
           % Ea_prev = Ea;
            J_hist = J;
            
            while (Ea>Ta && cyc<=Nact),
                    graduv = newAction.*(1-newAction);%0.5*(1-newAction.^2);         
                    gradEaJ = eact;

                        gradJu_ = 0; 
                        
                        for j=1:Num_u
                            for i=1:NC_Hidden,
                            %% check here when change sigmoid function
                            gradJu_ = gradJu_ + wc2(i)*0.5*(1-p(i)^2)*wc1(WA_Inputs+j,i);
                            end
                            gradJu(j)=gradJu_;
                            gradJu_ = 0; 
                        end
%                     end %end of "switch(CNN)"
                
                    %----for the first layer(input to hidden layer)-----------
                    % FJ start: revision because of the multi-inputs
                    for i=1:NA_Hidden,
                        gradvg = wa2(i,:);  % the coeffience from ith hidden neuron to outupt u respectively
                        gradgh = 0.5*(1-g(i)^2); % the derivative of ith hidden neuron
%                         gradgh = g(i)*(1-g(i));
                        gradhwa1 = inputs'; % change from row vector to column vector inorder to match wa1's column
                        delta_wa1(:,i)=(-la)*sum(gradEaJ*gradJu.*graduv.*gradvg*gradgh)*gradhwa1; % utilize the sum of derative with respective ui(t) 
                        %wa1(:,i) = wa1(:,i) + delta_wa1;
                    end
                    wa1=wa1+delta_wa1;
                    % FJend: revision because of the multi-inputs
                    
                    %----for the second layer(hidden layer to output)-----------
                    gradvwa2 = g'; % change from row vector to column vector inorder to match wa2's column
                    %delta_wa2=-la*gradEaJ*gradJu*graduv*gradvwa2;
                    delta_wa2=-la*gradvwa2*(gradEaJ*gradJu.*graduv);
                    wa2 = wa2 + delta_wa2;
                  
                    %----compute new J and newAction-------
                    ha = inputs*wa1;
                    g = (1 - exp(-ha))./(1 + exp(-ha));
                    va = g*wa2;
                    newAction = 1./(1 + exp(-va));
                    
                    inp=[inputs newAction];
                    qc=inp*wc1;
                    p = (1 - exp(-qc))./(1 + exp(-qc));
                    J=p*wc2;
                    
                
                    cyc = cyc+1;
                    eact = (J - Uc);
                    Ea = 0.5*eact^2;
                    
                    Ea_hist= [Ea_hist;Ea];
                    J_hist = [J_hist;J];
                
            end %end of "while (Ea>Ta & cyc<=Nact)"
            
            if length(Ea_hist) < Nact+2
                Ea_hist = [Ea_hist; nan(Nact+2-length(Ea_hist),1)];
            end
            
            if (max(max(abs(wa1)))>1)
                wa1=wa1/max(max(abs(wa1)));
                disp('wa1 norm')
            end
            if max(max(abs(wa2)))>1
                wa2=wa2/max(max(abs(wa2)));
                disp('wa2 norm')
            end          
            
           

%% End of HDP algorithm    

% if size(NN.reinfhist,1) >= num_step
%     sum_sqrderr = sum(-NN.reinfhist(length(NN.reinfhist)-num_step+1:length(NN.reinfhist)));
%     if (sum_sqrderr <= msethrshld)
%         la = la/num_drop;
%         lc = lc/num_drop;
%         msethrshld = msethrshld/thrshld_drop;
%         disp(['learning rate changed at', num2str(length(NN.reinfhist)), '-th step']);
%     elseif (sum_sqrderr > 4*Init_msethrshld)
%         if (msethrshld ~= Init_msethrshld)
%             la = Initla;
%             lc = Initlc;
%             msethrshld = Init_msethrshld;
%             disp('learning rate reset')
%         end
%     end
%     NN.sum_sqrderr_hist = [NN.sum_sqrderr_hist; sum_sqrderr];
%     NN.msethrshld_hist = [NN.msethrshld_hist; msethrshld];
% end
        

newAction = K*newAction;
% newAction(1) = 0.5*newAction(1)+1;
% newAction(2) = newAction(2)+2;

NN.wa1 = wa1;
NN.wa2 = wa2;
NN.wc1 = wc1;
NN.wc2 = wc2;
NN.la = la;
NN.lc = lc;
NN.msethrshld = msethrshld;

NN.wc1hist = [NN.wc1hist; NN.wc1(:,1)'];
NN.wa1hist = [NN.wa1hist; NN.wa1(:,1)'];

NN.wc2hist = [NN.wc2hist; NN.wc2(:,1)'];
NN.wa2hist = [NN.wa2hist; NN.wa2(:,1)'];

NN.reinfhist = [NN.reinfhist; reinf];
NN.Jhist = [NN.Jhist; J];

NN.errorchist = [NN.errorchist, Ec_hist];
NN.errorahist = [NN.errorahist, Ea_hist];

NN.lahist = [NN.lahist; la];
NN.lchist = [NN.lchist; lc];
% 
NN.Jprev=J;
%NN.J_hist_ = [NN.J_hist_, J_hist];

