%------------------------------------------------------------------------%
%extreme case 2 (flow 1 dominates: q=12/1)
function [] = main_v1(mm,ii,flag,T,w,q)
%clear all; close all;
%load v0ai_240_1.mat
%load xiInit_240_1.mat
global m bdry p p1 p2 D BDRY
global tau v0a0 v0ai vmax r A1a A1aa A2a A1r B1a B2a B1r lambda A1 B1 VS dist
global nr rr 
% Number of Pedestrians%
%m = input('Please select the number of Pedestrians (must be even, recommend: 80):');
% Timespan of simulation %
%T = input('Please select the desired length of the simulation (recommend: 10): ');
%file_name = input('Enter the name of the file to be saved: ', 's');
tstep = 0.1; % sampling time interval
K = 1;
%flag = 1; % 0--open-loop; 1--ADP
%% ---------- pedestrian dynamics model parameters -------------- %
m = mm;
%T = 20; % total simulation time
D = zeros(1,m);
tau = 0.5; % Relaxation time
v0a0 = 2; % Initial desired velocity
v0ai = random('norm', v0a0, 0.3, [1,m]);
vmax = 1.3*v0a0; % Maximum desired velocity
r = 0.2*ones(1,m); % Pedestrian radii
% Pedestrian Interaction constants %
A1a = 0;
A1aa = 2.6;
A2a = 2;
A1r = 10;
B1a = 1;
B2a = 0.2;
B1r = 1;
lambda = 0.25;
% Boundary Interaction constants %
A1 = 15;
B1 = 0.1;
% Radius of Verlet Sphere %
VS = 1.5;
% Distance to waypoint at which it deem to have been reached %
dist = 2;
flow_cr = 4.1;
pres_cr = 0.6;
%q = 1/2;

%------------- Desired destination ----------------%
if (K == 1)
    p = [6*ones(1,m*q)',6*ones(1,m*q)';4*ones(1,m-m*q)',8*ones(1,m-m*q)'];
%     p1 = [[50*ones(1,(3/0.1)+1)]',[4.5:0.1:7.5]'];
%     p2 = [[50*ones(1,(3/0.1)+1)]',[4.5:0.1:7.5]'];
    p1 = [50,6];
    p2 = [50,6];
end

% ------------Boundary array in the environment------%
if (K == 1)
    bdry = [[-40:0.1:8,4:0.1:8,4*ones(1,(39/0.1)+1)]',...
        [8*ones(1,(48/0.1)+1),4*ones(1,(4/0.1)+1),-35:0.1:4]'];
   % BDRY = 1;
end

%% ----------- Initial State Vector and Environment Setup------------------%
if (K == 1)
    %way = 1;
    Vx = [random('norm',v0a0,0.3,[1,m*q]),zeros(1,(m-m*q))];
    Vy = [zeros(1,m*q),random('norm',v0a0,0.3,[1,(m-m*q)])];
%     Vx = zeros(1,m);
%     Vy = zeros(1,m);
%     X = [12*rand(1,m*q)-12,4*rand(1,(m-m*q))]; %8,-6
%     Y = [4*rand(1,m*q)+4,10*rand(1,(m-m*q))-9];%8,-5
    X = [20*rand(1,m*q)-20,4*rand(1,(m-m*q))]; %8,-6
    Y = [4*rand(1,m*q)+4,12*rand(1,(m-m*q))-11];%8,-5

    %      X = [6*rand(1,m/2)-5,4*rand(1,m/2)];
    %      Y = [4*rand(1,m/2)+4,6*rand(1,m/2)-4];
    
    %     X = [4*rand(1,m)];
    %     Y = [6*rand(1,m)-4];
end

% -----robot position------- %
RoInit = [0.5,2.5]; % robot initial condition
nr = 1; % number of robot
rr = 0.5*(ones(1,nr)); %robot radius
%w = 0.2*pi; % robot moving frequency
A = 1.5; % amplitude

% Setting xiInit - the initial position and velocity vector %
for i = 1:m
    xiInit((2*i)-1) = Vx(i);
    xiInit((2*i)) = Vy(i);
end
for i = 1:m
    xiInit((2*m)+(2*i)-1) = X(i);
    xiInit((2*m)+(2*i)) = Y(i);
end
for j = 1:nr;
    xiInit = [xiInit, RoInit(j,:)]; % initial state including pedestrians and robot
end

% -------- initial robot control-----------%
if nr ~=0
    %urInit(1) =  -w*xiInit(4*m+2*nr+1)+w*(1+1);
    urInit(1) = 0;
    urInit(2) = 0; % d2x/dt2 = -w*x + w*(C+A) -- C is peak position; A is magnitute
else
    urInit = [];
end

x = 2.5; y = 5; Ro = 1.5; R = 0.8;% define region
xl = 4; xh = 7;
yl = 4; yh = 8;
ds = 0.5;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
sampleSt = [];  % state sampled every tstep=0.1 sec
dens_temp = [];       % temporal evolution of density 
flowrate_temp = [];  % temporal evolution of flow rate;
flowaccum_temp = [];        % number of peds evacuated
pressure_temp = [];   % temporal evolution of crowd pressure
flow_sum = 0;        
%prs = [];       % crowd pressure index in observed region
%v_var = [];     % velocity variance in observed region
outflow = [];
outflow_sum = [];
J_flow = [];     % cost function (flow)
Jhist_rho = [];     % cost function (density)
Jhist_pres = [];    % cost function (penalty)
totalSt = [];   % state over total simulation duration T
totalT1 = [];   % total time instance associated with total state totalSt over T
% totalSt and totalT1 are defined for ploting result
pres_loc = zeros(M,N);
v_var = zeros(M,N);
omg_acc = 0;
xrhist = [];
urhist = [];
uhist = [];
x_input = zeros(1,5);
inp_adp = 5;
Tn=[];
Tn1=[];
%% --------- The solver for the system -----------%
it = 0;         % the it-th time step
ts = 0;
dt = 10;
newSt = xiInit; % initialize state and control for first iteration
ur = urInit;
%ur = urInit;
if flag == 1
    %[NN] = inicialize_adp(inp_adp); % ADP initialization
    load NN_c1.mat;
    newAction = 0;
end

while it <= T
    options1 = odeset('AbsTol',1d-3,'RelTol',1d-3);
    tic
    if (it < 0.5)
        [t1,xi1] = ode23(@ped_dynamics,[0 tstep],newSt,options1,ur,q);
    else
        [t1,xi1] = ode23(@ped_dynamics_im2,[0 tstep],newSt,options1,ur,q);
    end
    Tn1 = [Tn1, toc];
    a=size(xi1);
    newSt=xi1(a(1),:);  % state update
    
%     % calculate variables %
%     dens = 0;
%     cnt = 0;
%     v_num = 0;
%     fn_all = 0;    
%     v_diff = [];      
%     % calculate density at current timestep
%     for n = 1:2:2*m
%         dis = norm([newSt(1,2*m+n)-x, newSt(1,2*m+n+1)-y]);
%         if dis < Ro
%             fn = (pi*Ro^2)^(-1)*exp(-dis^2/Ro^2);
%             v_numn = newSt(1,n)*fn;
%             cnt = cnt + 1;
%         else
%             fn = 0;
%             v_numn = 0;
%         end
%         dens = dens + fn; % density
%         v_num = v_num + v_numn;        
%     end
%     % calculate average velocity, flowrate and pressure at current timestep
%     if cnt ~= 0;
%         v_avg = v_num/dens; % average velocity
%         for n = 1:2:2*m
%             dis = norm([newSt(1,2*m+n)-x, newSt(1,2*m+n+1)-y]);
%             if dis < Ro
%                 fn = (pi*Ro^2)^(-1)*exp(-dis^2/Ro^2);
%                 v_diff = [v_diff, fn*(newSt(1,n)-v_avg)^2];
%                 fn_all = fn_all + fn;
%             end
%         end
%         flowrate = dens*v_avg; % flow rate
%         pres = dens*(sum(v_diff)/fn_all); % crowd pressure 
%         flow_sum = flow_sum + tstep*flowrate;
%     else
%         v_avg = 0;
%         flowrate = 0;
%         pres = 0;
%         flow_sum = flow_sum + 0;
%     end
    
%     % -------- calculate flowrate v2 (avg.)------ %
%     dens = 0;
%     cnt = 0;
%     v_num = 0;
%     v_avg = 0;
%     % calculate density
%     for n = 1:2:2*m
%         if (newSt(1,2*m+n)>xl && newSt(1,2*m+n)<xh && ...
%                 newSt(1,2*m+n+1)>yl && newSt(1,2*m+n+1)<yh)
%             v_num = v_num + newSt(1,n);
%             cnt = cnt + 1;
%         end
%     end            
%     if cnt ~= 0;
%         dens = cnt/12;
%         v_avg = v_num/cnt;
%         flowrate = dens*v_avg;
%         flow_sum = flow_sum + tstep*flowrate;
%     else
%         flowrate = 0;
%         flow_sum = flow_sum + 0;
%     end

    % -------- calculate flowrate v3 (local)------ %
    cnt1 = 0;
    cnt2 = 0;
    for i=1:2:2*m;
        if (newSt(1,2*m+i)>4)
            cnt1 = cnt1 + 1;
        end
        if (newSt(1,2*m+i)>4 && newSt(1,2*m+i)<8)
            cnt2 = cnt2 + 1;
        end
    end
    outflow_cnt1(ts+1) = cnt1;
    outflow_cnt2(ts+1) = cnt2;
    if ts+1 == 1
        flowrate = (outflow_cnt1(ts+1)-0)/4;
        flow_sum = flow_sum + flowrate;        
    else
        flowrate =(outflow_cnt1(ts+1)-outflow_cnt2(ts))/4;
        flow_sum = flow_sum + flowrate;       
    end 
    
    

    % calculate spatial average of pressure
    % ------------- calculate average pressure v1 ----------------%
%     pres_mean = 0;
%     for i = 1:M
%         for j = 1:N
%             den = 0;
%             cnt = 0;            
%             v_diff = [];
%             v_num = 0;
%             fn_all = 0;
%             % calculate density
%             for n = 1:2:2*m
%                 dis = norm([newSt(1,2*m+n)-(xl+ds*(j-1)), newSt(1,2*m+n+1)-(yl+ds*(i-1))]);
%                 if dis < R
%                     fn = (pi*R^2)^(-1)*exp(-dis^2/R^2);
%                     v_numn = [newSt(1,n),newSt(1,n+1)]*fn;                    
%                     cnt = cnt + 1;
%                 else
%                     fn = 0;
%                     v_numn = 0;
%                 end
%                 den = den + fn;
%                 v_num = v_num + v_numn;                
%             end            
%             if cnt ~= 0;
%                 v_avg = v_num/den;
%                 for n = 1:2:2*m
%                     dis = norm([newSt(1,2*m+n)-(xl+ds*(j-1)), newSt(1,2*m+n+1)-(yl+ds*(i-1))]);
%                     if dis < R
%                         fn = (pi*R^2)^(-1)*exp(-dis^2/R^2);
%                         v_diff = [v_diff, fn*(norm([newSt(1,n),newSt(1,n+1)] - v_avg))^2];                        
%                         fn_all = fn_all + fn;
%                     end
%                 end
%                 v_var(i,j) = sum(v_diff)/fn_all;
%             else
%                 v_var(i,j) = 0;
%             end
%             pres_loc(i,j) = den*v_var(i,j);
%             pres_mean = pres_mean + pres_loc(i,j)/(N*M); %spatial average of pressure            
%         end
%     end
    % ------------------------ end v1 ---------------------------%
    
%     % ------------- calculate average pressure v2 ----------------% 
%     dens = 0;
%     cnt = 0;
%     v_diff = [];
%     v_num = [];
%     % calculate density
%     for n = 1:2:2*m
%         if (newSt(1,2*m+n)>=1 && newSt(1,2*m+n)<=4 && ...
%                 newSt(1,2*m+n+1)>=4 && newSt(1,2*m+n+1)<=7)
%             v_num = [v_num; [newSt(1,n), newSt(1,n+1)]];
%             cnt = cnt + 1;
%         end
%     end            
%     if cnt ~= 0;
%         dens = cnt/9;
%         v_avg = [sum(v_num(:,1)), sum(v_num(:,2))]/cnt;
%         for n = 1:2:2*m
%             if (newSt(1,2*m+n)>=1 && newSt(1,2*m+n)<=4 && ...
%                     newSt(1,2*m+n+1)>=4 && newSt(1,2*m+n+1)<=7)
%                 v_diff = [v_diff, (norm([newSt(1,n),newSt(1,n+1)] - v_avg))^2];
%             end
%         end
%         var = mean(v_diff);
%     else
%         var = 0;
%     end
%     pres_mean = dens*var;
%     % --------------------------- v2 end ----------------------------%
    
    
    % ---------------------- Call ADP function ------------------------- %

%     if (pres_mean - pres_cr > 0)
%         rp = 1;   % reward r2
%     else
%         rp = 0;
%     end
%     if rp == 0;
%         xi = 1; % set weights between two rewards
%     else
%         xi = 0.5;
%     end
    

%%%%---- testing ADP -------- ZN
% r_flow=1;
% rp=0;
% xr_(1)=2*xr(1)-1;% convert coordinates
% xr_(2)=2*xr(2)-3;% convert coordinates
% [newSt_adp,NN,QPARA] = hdp_main2(xr_,r_flow,rp,NN,QPARA);
% newSt_adp(1)=(newSt_adp(1)-1)*0.5+1;
% newSt_adp(2)=(newSt_adp(2)-1)*0.5+2;
%%%%------ end ---------------

   % rp = max(0,(pres_mean-pres_cr));
   win = 20; % define smooth window size (sec)
   
   if flag == 1
       if mod(ts,dt) == 0 && ts~=0
           flowrate_ = sum(J_flow(length(J_flow)-8:length(J_flow)))+flowrate; % accumulated outflow in one second
           flowrate_temp = [flowrate_temp; flowrate_];
%            inp_rho = circshift(inp_rho',1)'; % input: time history of density difference
%            inp_rho(1) = flowrate_ - rho_cr;
           if length(flowrate_temp)<win
               flowrate_smth = sum(flowrate_temp)/length(flowrate_temp);
           else
               flowrate_smth = sum(flowrate_temp(length(flowrate_temp)-(win-1):length(flowrate_temp)))/win;
           end 
           x_input = circshift(x_input',1)';
           x_input(1) = flowrate_smth - flow_cr;
           r_flow = (flowrate_smth - flow_cr)^2; % reward r(t): current outflow 
           %rdens = flowrate_;           
           %x_input = flowrate_smth - flow_cr;
           if it >= 5
               tic
               [newAction,NN] = hdp_main2(x_input,r_flow,NN);
               Tn = [Tn; toc];
           end
       else
           if isempty(Jhist_rho);
               r_flow = nan;
           else
               r_flow = Jhist_rho(length(Jhist_rho));
           end
       end
       
   end
    % ------------------------- ADP end ----------------------------%
    
    % --------------- robot position update ----------------%
    if nr ~=0
        if flag == 1
            omg_inst = newAction;
        elseif flag ==0
            omg_inst = w;
        end
        omg_acc = omg_acc + omg_inst;
        ur(1) = A*omg_inst*sin(omg_acc*tstep);
        ur(2) = 0;
          
    end 
    % -------------- end -------------------%
    
  
%     % ----------robot control update----------- %
%     if nr ~=0
%         if flag == 0
%             omg_inst = w;
%         elseif flag == 1;
%             omg_inst = newAction;
%         end
%         omg_acc = omg_acc + omg_inst;
%         %ur(1) = -w*xi1(a(1),4*m+2*nr+1)+w*(1+1);
%         ur(1) = -A*omg_inst*sin(omg_acc*tstep+pi);
%         ur(2) = 0; %control update
%     else
%         omg_inst = [];
%         ur = [];
%     end
%     % -----------------end----------------%
    
    % time history of variables %
    sampleSt = [sampleSt; newSt];
    totalSt = [totalSt; xi1];
    totalT1 = [totalT1; t1+T-it];
    %dens_temp = [dens_temp; dens];    
%    pressure_temp = [pressure_temp; pres_mean];
    J_flow = [J_flow; flowrate];  
%    Jhist_rho = [Jhist_rho; r_flow];
%     Jhist_pres = [Jhist_pres; rp];    
    flowaccum_temp = [flowaccum_temp; flow_sum]; 
    if nr ~= 0
        xrhist = [xrhist; [newSt(4*m+1),newSt(4*m+2)]];
        urhist = [urhist; ur];
        uhist = [uhist, omg_inst];    
    end
    it = it+tstep;
    ts = ts + 1;
%     if it >= T/2
%         q = 0.5;
%     end
    % reset pedestrian position if it reaches the end
    outflow_cnt = 0;    
    for i=1:2:2*q*m;
        if (newSt(2*m+i)>8)
            %newSt(i) = random('norm',v0a0,0.3);
            newSt(i) = v0ai((i+1)/2);
            newSt(i+1) = 0;
            newSt(2*m+i) = -10; %-8.5 %-3
            newSt(2*m+i+1) = newSt(2*m+i+1);
            D((i+1)/2) = 0;
            p((i+1)/2,:) = [6,6];
            outflow_cnt = outflow_cnt + 1;
        end
    end
    for i = 2*q*m+1:2:2*m
        if (newSt(2*m+i)>8)
            newSt(i) = 0;
            %newSt(i+1) = random('norm',v0a0,0.3);
            newSt(i+1) = v0ai((i+1)/2);
            newSt(2*m+i) = newSt(2*m+i+1)-4;
            newSt(2*m+i+1) = -10;%-7.5; %-5.5;%-2
            D((i+1)/2) = 0;
            p((i+1)/2,:) = [4,8];
            outflow_cnt = outflow_cnt + 1;
        end
    end
    outflow = [outflow, outflow_cnt/4];
    outflow_sum = [outflow_sum, sum(outflow)];
    
end

if nr==0
    filename = [num2str(m),'w',num2str(ii),'v',num2str(10*v0a0),'q',num2str(fix(10*q))];
elseif nr~=0
    filename = [num2str(m),'w',num2str(ii),'v',num2str(10*v0a0),'q',num2str(fix(10*q)),'r'];
end
save(filename)
clear all
%% --------------------------Plot Results---------------------------%%
% bdry = [[-8:0.01:8,4:0.01:8,4*ones(1,(12/0.01)+1)]',...
%     [8*ones(1,(16/0.01)+1),4*ones(1,(4/0.01)+1),-8:0.01:4]'];
% % Plotting the paths of the pedestrians and robot %
% figure(1)
% % Plotting the boundaries %
% plot(bdry(:,1),bdry(:,2),'kx')
% if (K == 1)
%     axis([0,8,0,8]);
% end
% hold on
% 
% % Plotting the pedestrian paths %
% for i = (2*m)+1:2:(4*m)
%     if (i<=3*m)
%         plot(totalSt(:,i),totalSt(:,i+1),'b-','MarkerSize',4)
%         if (K == 1)
%             axis([0,10,0,10]);
%         end
%         hold on
%     else
%         plot(totalSt(:,i),totalSt(:,i+1),'r-','MarkerSize',4)
%         if (K == 1)
%             axis([0,8,0,8]);
%         end
%         hold on
%     end
% end
% 
% % plotting robot position %
% if (nr ~=0);
%     for i = (2*nr)+1:2:(4*nr)
%         plot(totalSt(:,4*m+i),totalSt(:,4*m+i+1),'m:','MarkerSize',8)
%         hold on
%     end
% end
% title('Pedestrian and robot trajectories')
% xlabel('position x (m)')
% ylabel('position y (m)')
% 
% 
% % %plotting average velocity vs. time
% % figure(2)
% % grid on
% % v_sum_x = zeros(size(totalSt,1),1);
% % v_sum_y = zeros(size(totalSt,1),1);
% %
% % for j=1:size(totalSt,1)
% %     count = 0;
% %     for i = 1:2:2*m
% %         if (totalSt(j,2*m+i)>16 && totalSt(j,2*m+i)<18 && totalSt(j,2*m+i+1)>4 && totalSt(j,2*m+i+1)<5) % or from 14 to 16
% %             v_sum_x(j,1) = v_sum_x(j,1) + totalSt(j,i);
% %             count = count + 1;
% %         end
% %     end
% %     if count~=0
% %         v_sum_x(j,1) = v_sum_x(j,1)/count;
% %     else
% %         v_sum_x(j,1)=0;
% %     end
% % end
% %
% % hold on
% % plot(totalT1,sqrt(v_sum_x.^2+v_sum_y.^2),'b-');
% % title('Average Velocity in Observed Region')
% % xlabel('Time (sec)')
% % ylabel('Average Velocity (m/s)')
% %pause
% 
% 
% figure(2)
% % plot temporal evolution of cost function (flow)
% subplot(2,1,1)
% plot([0:0.1:T],J_flow,'k','LineWidth',2);
% title('Temporal evolution of flow rate','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Flow rate','FontSize',12)
% grid on
% set(gca,'FontSize',12);
% %plot accumulated flow
% subplot(2,1,2)
% plot([0:0.1:T],flowaccum_temp, 'k-','LineWidth',2);
% title('Accumulated number of pedestrians passed through','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Number of persons','FontSize',12)
% grid on
% set(gca,'FontSize',12);
% 
% figure(3)
% % plot density
% subplot(2,1,1)
% plot([0:0.1:T],dens_temp,'b','LineWidth',2);
% title('Temporal evolution of density','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Density (1/m^{2})','FontSize',12)
% grid on;
% set(gca,'FontSize',12);
% % plot crowd pressure
% subplot(2,1,2)
% plot([0:0.1:T],pressure_temp,'b','LineWidth',2);
% title('Temporal evolution of crowd pressure','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Pressure (1/s^{2})','FontSize',12)
% grid on;
% set(gca,'FontSize',12);
% 
% 
% %% Making the Movie - plots the pedestrian positions at each frame %%
% %aviObject = avifile(file_name);
% figure(4)
% figH = figure(4);
% for j = 1:50:size(totalSt,1)
%     %Plotting the boundaries %
%     hold off
%     plot(bdry(:,1),bdry(:,2),'kx')
%     if (K == 1)
%         axis([0,8,0,8]);
%     end
%     hold on
%     
%     % Plotting the pedestrians %
%     for i = (2*m)+1:2:(4*m)
%         if (i<=3*m)
%             plot(totalSt(j,i),totalSt(j,i+1),'bo','MarkerSize',12)
%             if (K == 1)
%                 axis([0,8,0,8]);
%             end
%             hold on
%         else
%             plot(totalSt(j,i),totalSt(j,i+1),'ro','MarkerSize',12)
%             if (K == 1)
%                 axis([0,8,0,8]);
%             end
%             hold on
%         end
%     end
%     % plotting the robot position  %
%     if (nr ~=0);
%         for i = (2*nr)+1:2:(4*nr)
%             plot(totalSt(j,4*m+i),totalSt(j,4*m+i+1),'mp','MarkerSize',12)
%             hold on
%         end
%     end
%     title(['Time(sec): ' num2str(floor(totalT1(j)))]);
%     F(j) = getframe(figH);
%     %aviObject = addframe(aviObject,F(j));
% end
% %aviObject = close(aviObject);
% %pause
% clf
