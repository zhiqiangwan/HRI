%% --------------------------Plot Results--------------------------- %%
bdry = [[-8:0.01:8,4:0.01:8,4*ones(1,(12/0.01)+1)]',...
    [8*ones(1,(16/0.01)+1),4*ones(1,(4/0.01)+1),-8:0.01:4]'];
% Plotting the paths of the pedestrians and robot %
figure(1)
% Plotting the boundaries %
plot(bdry(:,1),bdry(:,2),'kx')
if (K == 1)
    axis([0,8,0,8]);
end
hold on

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

% plotting robot position %
if (nr ~=0);
    for i = 4*m+1:2:(4*m+2*nr)
        plot(totalSt(:,i),totalSt(:,i+1),'m:','MarkerSize',8)
        hold on
    end
end
title('Pedestrian and robot trajectories')
xlabel('position x (m)')
ylabel('position y (m)')


% %plotting average velocity vs. time
% figure(2)
% grid on
% v_sum_x = zeros(size(totalSt,1),1);
% v_sum_y = zeros(size(totalSt,1),1);
%
% for j=1:size(totalSt,1)
%     count = 0;
%     for i = 1:2:2*m
%         if (totalSt(j,2*m+i)>16 && totalSt(j,2*m+i)<18 && totalSt(j,2*m+i+1)>4 && totalSt(j,2*m+i+1)<5) % or from 14 to 16
%             v_sum_x(j,1) = v_sum_x(j,1) + totalSt(j,i);
%             count = count + 1;
%         end
%     end
%     if count~=0
%         v_sum_x(j,1) = v_sum_x(j,1)/count;
%     else
%         v_sum_x(j,1)=0;
%     end
% end
%
% hold on
% plot(totalT1,sqrt(v_sum_x.^2+v_sum_y.^2),'b-');
% title('Average Velocity in Observed Region')
% xlabel('Time (sec)')
% ylabel('Average Velocity (m/s)')
%pause


figure(2)
% plot temporal evolution of cost function (flow)
flowrate_qsmth = [];
for i = 1:size(flowrate_temp,1)    
    if i<win
        flowrate_qsmth = [flowrate_qsmth; sum(flowrate_temp(1:i))/i];
    else
        flowrate_qsmth = [flowrate_qsmth; sum(flowrate_temp(i-(win-1):i))/win];
    end
end
subplot(2,1,1)
hold on
%plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,J_flow,'b','LineWidth',2);
plot(0:length(flowrate_temp)-1,flowrate_temp,'b','LineWidth',2)
plot(0:length(flowrate_qsmth)-1,flowrate_qsmth,'r','LineWidth',2)
title('Temporal evolution of outflow','FontSize',12)
xlabel('Time (sec)','FontSize',12)
ylabel('Outflow (1/m.s)','FontSize',12)
legend('original','smoothed')
grid on
box on
set(gca,'FontSize',12);
%plot accumulated flow
subplot(2,1,2)
plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowaccum_temp, 'b-','LineWidth',2);
title('Accumulated outflow','FontSize',12)
xlabel('Time (sec)','FontSize',12)
ylabel('Accumulated flow (1/m)','FontSize',12)
hold on
grid on
set(gca,'FontSize',12);

% figure(3)
% % plot density
% subplot(2,1,1)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,dens_temp,'b','LineWidth',2);
% title('Temporal evolution of density','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Density (1/m^{2})','FontSize',12)
% grid on;
% set(gca,'FontSize',12);
% % plot crowd pressure
% subplot(2,1,2)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,pressure_temp,'b','LineWidth',2);
% title('Temporal evolution of crowd pressure','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Pressure (1/s^{2})','FontSize',12)
% grid on;
% set(gca,'FontSize',12);

if nr ~=0
    figure(4)
    %plot time history of robot control
    subplot(2,1,1)
    plot(0:0.1:0.1*length(sampleSt(:,1))-0.1, urhist(:,1),'LineWidth',2)
    xlabel('Time (s)','FontSize',12);
    ylabel('u_r (m/s)','FontSize',12);
    set(gca,'FontSize',14)
    grid on
    hold on
    %plot time history of robot frequency
    subplot(2,1,2)
    plot(0:0.1:0.1*length(sampleSt(:,1))-0.1, uhist,'LineWidth',2)
    xlabel('Time (s)','FontSize',12);
    ylabel('\omega (rad/s)','FontSize',12);
    set(gca,'FontSize',14)
    grid on
    hold on

    figure(5)
    %plot time history of robot position x
    subplot(2,1,1)
    plot(0:0.1:0.1*length(sampleSt(:,1))-0.1, xrhist(:,1),'LineWidth',2)
    xlabel('Time (s)','FontSize',12);
    ylabel('x (m)','FontSize',12);
    set(gca,'FontSize',14)
    grid on
    hold on
    %plot time history of robot position y
    subplot(2,1,2)
    plot(0:0.1:0.1*length(sampleSt(:,1))-0.1, xrhist(:,2),'LineWidth',2)
    xlabel('Time (s)','FontSize',12);
    ylabel('y (m)','FontSize',12);
    set(gca,'FontSize',14)
    grid on
    hold on
    
%     figure(6)
%     %plot time history of r1
%     %subplot(2,1,1)
%     plot(0:0.1:0.1*length(sampleSt(:,1))-0.1, -Jhist_rho,'LineWidth',2)
%     xlabel('Time (s)','FontSize',12);
%     ylabel('Reward r(t)','FontSize',12);
%     set(gca,'FontSize',14)
%     grid on
%     hold on
    %     %plot time history of r2
    %     subplot(2,1,2)
    %     plot(0:0.1:0.1*length(sampleSt(:,1))-0.1, Jhist_pres,'LineWidth',2)
    %     xlabel('Time (s)','FontSize',12);
    %     ylabel('Cost penalty','FontSize',12);
    %     set(gca,'FontSize',14)
    %     grid on
    %     hold on
    if flag == 1
        figure(7)
        subplot(2,1,1)
        plot(NN.wc1hist)
        title('wc1','FontSize',12)
        set(gca,'FontSize',14)
        subplot(2,1,2)
        plot(NN.wc2hist)
        title('wc2','FontSize',12)
        set(gca,'FontSize',14)
        
        figure(8)
        subplot(2,1,1)
        plot(NN.wa1hist)
        title('wa1','FontSize',12)
        set(gca,'FontSize',14)
        subplot(2,1,2)
        plot(NN.wa2hist)
        title('wa2','FontSize',12)
        set(gca,'FontSize',14)
        
        figure(9)
        subplot(2,1,1)
        hold on
        box on
        plot(NN.errorahist(1,:))
        plot(NN.errorahist(20,:))
        title('Ea','FontSize',12)
        legend('initial','final')
        set(gca,'FontSize',14)
        subplot(2,1,2)
        hold on
        box on
        plot(NN.errorchist(1,:))
        plot(NN.errorchist(17,:))
        title('Ec','FontSize',12)
        legend('initial','final')
        set(gca,'FontSize',14)
        
        figure(10)
        hold on
        box on
        xlim([0,T])
        plotyy(1:length(NN.reinfhist),NN.reinfhist/8,1:length(NN.Jhist),NN.Jhist)
        hold on
        grid on
        legend('reinfhist','Jhist')
        set(gca,'FontSize',14)
    end
    
end

%% Making the Movie - plots the pedestrian positions at each frame %%
%aviObject = avifile(file_name);
figure(4)
figH = figure(4);
for j = 1:100:size(totalSt,1)
    %Plotting the boundaries %
    hold off
    plot(bdry(:,1),bdry(:,2),'kx')
    if (K == 1)
        axis([0,8,0,8]);
    end
    hold on
    
    % Plotting the pedestrians %
    for i = (2*m)+1:2:(4*m)
        if (i<=2*m+q*2*m)
            plot(totalSt(j,i),totalSt(j,i+1),'bo','MarkerSize',14)
            if (K == 1)
                axis([0,8,0,8]);
            end
            hold on
        else
            plot(totalSt(j,i),totalSt(j,i+1),'ro','MarkerSize',14)
            if (K == 1)
                axis([0,8,0,8]);
            end
            hold on
        end
    end
    % plotting the robot position  %
    if (nr ~=0);
        for i = 4*m+1:2:4*m+2*nr
            plot(totalSt(j,i),totalSt(j,i+1),'mp','MarkerSize',14)
            hold on
        end
    end
    title(['Time(sec): ' num2str(T-floor(totalT1(j)))]);
    F(j) = getframe(figH);
    %aviObject = addframe(aviObject,F(j));
end
%aviObject = close(aviObject);
%pause
clf
% 
% %% plot temporal pressure
% % ---------------------- calculate average pressure v1 in main.m -------------------%
% xl = 1; xh = 4;
% yl = 4; yh = 7;
% R = 0.8;
% ds = 0.5;
% N = length(xl:ds:xh);
% M = length(yl:ds:yh);
% pressure_temp=[];
% for ti = 1:size(sampleSt,1)
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
%                 dis = norm([sampleSt(ti,2*m+n)-(xl+ds*(j-1)), sampleSt(ti,2*m+n+1)-(yl+ds*(i-1))]);
%                 if dis < R
%                     fn = (pi*R^2)^(-1)*exp(-dis^2/R^2);
%                     v_numn = [sampleSt(ti,n),sampleSt(ti,n+1)]*fn;
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
%                     dis = norm([sampleSt(ti,2*m+n)-(xl+ds*(j-1)), sampleSt(ti,2*m+n+1)-(yl+ds*(i-1))]);
%                     if dis < R
%                         fn = (pi*R^2)^(-1)*exp(-dis^2/R^2);
%                         v_diff = [v_diff, fn*(norm([sampleSt(ti,n),sampleSt(ti,n+1)] - v_avg))^2];
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
%     pressure_temp = [pressure_temp; pres_mean];
% end
% 
% % plot crowd pressure
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,pressure_temp,'b','LineWidth',2);
% title('Temporal evolution of crowd pressure','FontSize',12)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Pressure (1/s^{2})','FontSize',12)
% grid on;
% hold on;
% set(gca,'FontSize',12);
% 
% % %% ------------- calculate average pressure v2 (p4) ---------------------%
% % xl = 1; xh = 4;
% % yl = 4; yh = 7;
% % R = 0.5;
% % ds = 0.2;
% % N = length(xl:ds:xh);
% % M = length(yl:ds:yh);
% % pressure_temp=[];
% % for ti = 1:size(sampleSt,1)    
% %     rho = 0;
% %     cnt = 0;
% %     v_avg = [];
% %     v_diff = [];
% %     for n = 1:2:2*m
% %         if (sampleSt(i,2*m+n)>xl && sampleSt(i,2*m+n)<xh && ...
% %                 sampleSt(i,2*m+n+1)>yl && sampleSt(i,2*m+n+1)<yh)            
% %             cnt = cnt + 1;
% %         end
% %     end            
% %     if cnt ~= 0;
% %         rho = cnt/9;
% %     end
% %     for i = 1:M
% %         for j = 1:N 
% %             den = 0;
% %             cnt = 0;            
% %             v_num = 0;
% %             fn_all = 0;
% %             % calculate density
% %             for n = 1:2:2*m
% %                 dis = norm([sampleSt(ti,2*m+n)-(xl+ds*(j-1)), sampleSt(ti,2*m+n+1)-(yl+ds*(i-1))]);
% %                 if dis < R
% %                     fn = (pi*R^2)^(-1)*exp(-dis^2/R^2);
% %                     v_numn = [sampleSt(ti,n),sampleSt(ti,n+1)]*fn;
% %                     cnt = cnt + 1;
% %                 else
% %                     fn = 0;
% %                     v_numn = 0;
% %                 end
% %                 den = den + fn;
% %                 v_num = v_num + v_numn;
% %             end
% %             if cnt ~= 0;
% %                 v_avg = [v_avg; v_num/den];
% %             else
% %                 v_avg = [v_avg; [0,0]];
% %             end            
% %         end
% %     end    
% %     for i = 1:size(v_avg,1)
% %         v_avg(i,:) = v_avg(i,:) - mean(v_avg);
% %         v_diff(i) = norm(v_avg(i,:))^2;
% %     end
% %     pres_mean = mean(v_diff);
% %     pressure_temp = [pressure_temp; pres_mean];
% % end
% % 
% % % plot crowd pressure
% % plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,pressure_temp,'b','LineWidth',2);
% % title('Temporal evolution of crowd pressure','FontSize',12)
% % xlabel('Time (sec)','FontSize',12)
% % ylabel('Pressure (1/s^{2})','FontSize',12)
% % grid on;
% % hold on;
% % set(gca,'FontSize',12);
% 
% % %% ------------- calculate average pressure v3 ----------------% 
% % pressure_temp=[];
% % for i = 1:size(sampleSt,1)
% %     dens = 0;
% %     cnt = 0;
% %     v_diff = [];
% %     v_num = [];
% %     % calculate density
% %     for n = 1:2:2*m
% %         if (sampleSt(i,2*m+n)>1 && sampleSt(i,2*m+n)<4 && ...
% %                 sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<7)
% %             v_num = [v_num; [sampleSt(1,n), sampleSt(1,n+1)]];
% %             cnt = cnt + 1;
% %         end
% %     end            
% %     if cnt ~= 0;
% %         dens = cnt/9;
% %         v_avg = [sum(v_num(:,1)), sum(v_num(:,2))]/cnt;
% %         for n = 1:2:2*m
% %             if (sampleSt(i,2*m+n)>1 && sampleSt(i,2*m+n)<4 && ...
% %                     sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<7)
% %                 v_diff = [v_diff, (norm([sampleSt(i,n),sampleSt(i,n+1)] - v_avg))^2];
% %             end
% %         end
% %         var = mean(v_diff);
% %     else
% %         var = 0;
% %     end
% %     pres_mean = dens*var;
% %     pressure_temp = [pressure_temp,pres_mean];
% % end
% %     
% % % plot crowd pressure
% % plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,pressure_temp,'b','LineWidth',2);
% % title('Temporal evolution of crowd pressure','FontSize',12)
% % xlabel('Time (sec)','FontSize',12)
% % ylabel('Pressure (1/s^{2})','FontSize',12)
% % grid on;
% % set(gca,'FontSize',12);
% 
% %% ---------- outflow at bottleneck (local) ----------------- %%
% x = 5; y = 5; % define region x=4,y=5;
% Ro = 4; % 3
% pres_loc = [];
% dens_loc = [];
% var_loc = [];
% 
% for t = 1:size(sampleSt,1)
%     dens = 0;
%     cnt = 0;
%     v_avg = 0;
%     v_diff = [];
%     v_var = 0;
%     v_num = 0;
%     v_deno = 0;
%     fn_all = 0;
%     % calculate density
%     for n = 1:2:2*m
%         dist = norm([sampleSt(t,2*m+n)-x, sampleSt(t,2*m+n+1)-y]);
%         if dist < Ro
%             fn = (pi*Ro^2)^(-1)*exp(-dist^2/Ro^2);
%             v_numn = sampleSt(t,n)*fn;
%             cnt = cnt + 1;
%         else
%             fn = 0;
%             v_numn = 0;
%         end
%         dens = dens + fn;
%         v_num = v_num + v_numn;
%         v_deno = v_deno + fn;
%     end
%     if cnt ~= 0;
%         v_avg = v_num/v_deno;
%         for n = 1:2:2*m
%             dist = norm([sampleSt(t,2*m+n)-x, sampleSt(t,2*m+n+1)-y]);
%             if dist < Ro
%                 fn = (pi*Ro^2)^(-1)*exp(-dist^2/Ro^2);
%                 v_diff = [v_diff, fn*(sampleSt(t,n)-v_avg)^2];
%                 fn_all = fn_all + fn;
%             end
%         end
%         v_var = sum(v_diff)/fn_all;
%     else
%         v_avg = 0;
%         v_var = 0;
%     end
%     pres_loc(t) = dens*v_var;
%     dens_loc(t) = dens;
%     var_loc(t) = v_avg(1);
%     flowrate(t) = dens*v_avg(1);
% end
% % plot velocity vs. density
% figure(3)
% hold on
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Outflow (1/m.s)','FontSize',12)
% grid on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
% 
% %% ------------------- outflow at bottleneck (avg.) ----------------- %%
% flowrate_q = [];
% flowrate_qsum = [];
% flowrate_d = [];
% flowrate_dsum = [];
% for i = 1:size(sampleSt,1)
%     dens = 0;
%     cnt = 0;
%     v_num = 0;
%     v_avg = 0;
%     % calculate density
%     for n = 1:2:2*m
%         if (sampleSt(i,2*m+n)>4 && sampleSt(i,2*m+n)<7 && ...
%                 sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<8)
%             v_num = v_num + sampleSt(i,n);
%             cnt = cnt + 1;
%         end
%     end            
%     if cnt ~= 0;
%         dens = cnt/12;
%         v_avg = v_num/cnt;
%     end
%     flowrate_q = [flowrate_q; dens*v_avg];
%     flowrate_qsum = [flowrate_qsum; sum(flowrate_q)];
%     flowrate_d = [flowrate_d; (dens*v_avg-5)^2];
%     flowrate_dsum = [flowrate_dsum; sum(flowrate_d)];
% end
% figure(4)
% subplot(2,1,1)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_q,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Outflow (person/m.s)','FontSize',12)
% grid on
% hold on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
% subplot(2,1,2)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,0.1*flowrate_qsum,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Accumulated Outflow (person/m)','FontSize',12)
% grid on
% hold on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
% 
% figure(5)
% subplot(2,1,1)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_d,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Reward r(t)','FontSize',12)
% grid on
% hold on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
% subplot(2,1,2)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,0.1*flowrate_dsum,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Cost function J(t)','FontSize',12)
% grid on
% hold on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
%     
% %% count at x=8 %%
% figure(6)
% outflow_sec = 0;
% for i = 1:size(outflow,2)/10
%     outflow_sec = [outflow_sec; sum(outflow((i-1)*10+1:(i-1)*10+10))];
% end
% subplot(2,1,1)
% plot(0:0.1*length(sampleSt(:,1)),outflow_sec,'k','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Outflow (person/m.s)','FontSize',12)
% grid on
% hold on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
% subplot(2,1,2)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,outflow_sum,'k','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Accumulated Outflow (person/m)','FontSize',12)
% grid on
% hold on
% %axis([0.2 2.2 0.2 1.6])
% set(gca,'FontSize',12);
% 
% %% count at x=4 %%
% flowrate_q = [];
% flowrate_qsum = [];
% outflow_cnt1 = [];
% outflow_cnt2 = [];
% for t = 1:size(sampleSt,1)
%     cnt1 = 0;
%     cnt2 = 0;
%     for i=1:2:2*m;
%         if (sampleSt(t,2*m+i)>4)
%             cnt1 = cnt1 + 1;
%         end
%         if (sampleSt(t,2*m+i)>4 && sampleSt(t,2*m+i)<8)
%             cnt2 = cnt2 + 1;
%         end
%     end
%     outflow_cnt1(t) = cnt1;
%     outflow_cnt2(t) = cnt2;
%     if t == 1
%         flowrate_q = [flowrate_q; (outflow_cnt1(t)-0)/4];
%     else
%         flowrate_q = [flowrate_q; (outflow_cnt1(t)-outflow_cnt2(t-1))/4];
%     end 
%     flowrate_qsum = [flowrate_qsum; sum(flowrate_q)];
% end
% outflow_sec = 0;
% for i = 1:size(outflow,2)/10
%     outflow_sec = [outflow_sec; sum(flowrate_q((i-1)*10+1:(i-1)*10+10))];
% end
% figure(7)
% subplot(2,1,1)
% plot(0:0.1*length(sampleSt(:,1)),outflow_sec,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Outflow (person/m.s)','FontSize',12)
% set(gca,'FontSize',12);
% grid on
% hold on
% 
% subplot(2,1,2)
% plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',2)
% xlabel('Time (sec)','FontSize',12)
% ylabel('Outflow (person/m)','FontSize',12)
% set(gca,'FontSize',12);
% grid on
% hold on

