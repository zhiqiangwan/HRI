%clc; clear all;
accflow_table = [];
dens_table = [];
vel_table = [];
pressure_table = [];
for iii = 1
    for jjj = 1:1:15
% %         if iii==1
% %             iii_=2;
% %         elseif iii==2
% %             iii_=3;
% %         elseif iii==3
% %             iii_=5;
% %         elseif iii==4
% %             iii_=6;
% %         elseif iii==5
% %             iii_=7;
% %         end           
%         if jjj_==0
%             jjj=1;
%         elseif jjj_==5
%             jjj=2;
%         elseif jjj_==10
%             jjj=3;
%         elseif jjj_==15
%             jjj=4;
%         elseif jjj_==20
%             jjj=5;
%         elseif jjj_==25
%             jjj=6;
%         elseif jjj_==30
%             jjj=7;
%         elseif jjj_==35
%             jjj=8;
%         end            

        filename_ = ['210w',num2str(jjj),'v20q',num2str(5),'r.mat'];
        load(filename_) 
%         % calculate flow v1
%         flowrate_q = [];
%         flowrate_qsum = [];
%         flowrate_d = [];
%         flowrate_dsum = [];
%         for i = 1:size(sampleSt,1)
%             dens = 0;
%             cnt = 0;
%             v_num = 0;
%             v_avg = 0;
%             % calculate density
%             for n = 1:2:2*m
%                 if (sampleSt(i,2*m+n)>4 && sampleSt(i,2*m+n)<7 && ...
%                         sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<8)
%                     v_num = v_num + sampleSt(i,n);
%                     cnt = cnt + 1;
%                 end
%             end
%             if cnt ~= 0;
%                 dens = cnt/12;
%                 v_avg = v_num/cnt;
%             end
%             flowrate_q = [flowrate_q; v_avg*dens];
%             flowrate_qsum = [flowrate_qsum; 0.1*sum(flowrate_q)];
%             flowrate_d = [flowrate_d; (dens*v_avg-5)^2];
%             flowrate_dsum = [flowrate_dsum; 0.1*sum(flowrate_d)];
%         end
%        % end v1 %
        
        % calculate flow v3
        flowrate_q = [];
        flowrate_qsum = [];
        flowrate_qsmth = [];
        outflow_cnt1 = [];
        outflow_cnt2 = [];
        for t = 1:size(sampleSt,1)
            cnt1 = 0;
            cnt2 = 0;
            for i=1:2:2*m;
                if (sampleSt(t,2*m+i)>4 && sampleSt(t,2*m+i+1) > 4)
                    cnt1 = cnt1 + 1;
                end
                if (sampleSt(t,2*m+i)>4 && sampleSt(t,2*m+i)<8 && sampleSt(t,2*m+i+1) > 4)
                    cnt2 = cnt2 + 1;
                end
%                 if (sampleSt(t,2*m+i+1)>2.5)
%                     cnt1 = cnt1 + 1;
%                 end
%                 if (sampleSt(t,2*m+i+1)>2.5 && sampleSt(t,2*m+i)<8)
%                     cnt2 = cnt2 + 1;
%                 end
             end
            outflow_cnt1(t) = cnt1;
            outflow_cnt2(t) = cnt2;
            if t == 1
                flowrate_q = [flowrate_q; (outflow_cnt1(t)-0)/4];
            else
                flowrate_q = [flowrate_q; (outflow_cnt1(t)-outflow_cnt2(t-1))/4];
            end
            flowrate_qsum = [flowrate_qsum; sum(flowrate_q)];
        end
        outflow_sec = 0;
        for i = 1:size(outflow,2)/10
            outflow_sec = [outflow_sec; sum(flowrate_q((i-1)*10+1:(i-1)*10+10))];
            win = 10; % moving average window size
            if length(outflow_sec)<win
                flowrate_qsmth = [flowrate_qsmth; sum(outflow_sec)/length(outflow_sec)];
            else
                flowrate_qsmth = [flowrate_qsmth; sum(outflow_sec(length(outflow_sec)-(win-1):length(outflow_sec)))/win];
            end
        end
        % end v3 %
        accflow_table(iii,jjj+1)=flowrate_qsum(length(flowrate_qsum))-flowrate_qsum(400);
        
        % calculate velocity v1 %
        vel_avg = [];
        rho_avg = [];               
        for i = 1:size(sampleSt,1)
            dens = 0;
            cnt = 0;
            v_num = 0;
            v_avg = 0;
            % calculate density
            for n = 1:2:2*m
                if (sampleSt(i,2*m+n)>2 && sampleSt(i,2*m+n)<4 && ...
                        sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<7)
                    v_num = v_num + norm([sampleSt(i,n),sampleSt(i,n+1)]);
                    cnt = cnt + 1;
                end
            end
            if cnt ~= 0;
                dens = cnt/6;
                v_avg = v_num/cnt;
            end
            vel_avg = [vel_avg; v_avg];
            rho_avg = [rho_avg; dens];   
        end
        % end v1 %
        
%         % calculate velocity v2
%         x = 3; y = 6; % define region x=4,y=5;
%         Ro = 1.5; % 3        
%         rho_avg = [];
%         vel_avg = [];
%         
%         for t = 1:size(sampleSt,1)
%             dens = 0;
%             cnt = 0;
%             v_avg = 0;
%             v_num = 0;
%             v_deno = 0;
%             % calculate density
%             for n = 1:2:2*m
%                 dist = norm([sampleSt(t,2*m+n)-x, sampleSt(t,2*m+n+1)-y]);
%                 if dist < Ro
%                     fn = (pi*Ro^2)^(-1)*exp(-dist^2/Ro^2);
%                     v_numn = sampleSt(t,n)*fn;
%                     cnt = cnt + 1;
%                 else
%                     fn = 0;
%                     v_numn = 0;
%                 end
%                 dens = dens + fn;
%                 v_num = v_num + v_numn;
%                 v_deno = v_deno + fn;
%             end
%             if cnt ~= 0;
%                 v_avg = v_num/v_deno;                
%             else
%                 v_avg = 0;               
%             end
%             rho_avg = [rho_avg; dens];
%             vel_avg = [vel_avg; v_avg];
%         end
%         % end v2 %
       
        dens_table(iii,jjj+1) = sum(rho_avg(401:length(rho_avg)))/(length(rho_avg)-400);
        vel_table(iii,jjj+1) = sum(vel_avg(401:length(vel_avg)))/(length(vel_avg)-400);
        flowrate_table(iii,jjj+1) = sum(flowrate_qsmth(21:length(flowrate_qsmth)))/(length(flowrate_qsmth)-20);
    end
    
    
%     % calculate crowd pressure %
%     ds = 0.1;
%     xl=1; xh=5;
%     yl=3; yh=8;
%     N = length(xl:ds:xh);
%     M = length(yl:ds:yh);
%     prs = [];
%     dens = [];
%     R = 1;
%     for t=1:size(sampleSt,1)
%         v = zeros(M,N);
%         v_avg = 0;
%         v_diff = [];
%         v_var = 0;
%         count = 0;
%         % number of pedestrians
%         for i=1:2:2*m
%             if (sampleSt(t,2*m+i)>=xl && sampleSt(t,2*m+i)<=xh && sampleSt(t,2*m+i+1)>=yl && sampleSt(t,2*m+i+1)<=yh) % range of the specific section (or from 14 to 16)
%                 count = count + 1;
%             end
%         end
%         dens(t) = count/[(xh-xl)*(yh-yl)];
%         % local velocity
%         for i = 1:M
%             for j = 1:N
%                 v_num = 0;
%                 v_deno = 0;
%                 cnt = 0;
%                 for n=1:2:2*m
%                     dist = norm([sampleSt(t,2*m+n)-(xl+ds*(j-1)), sampleSt(t,2*m+n+1)-(yl+ds*(i-1))]);
%                     if dist < R
%                         fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
%                         v_numn = [sampleSt(t,n),sampleSt(t,n+1)]*fn;
%                         cnt = cnt + 1;
%                     else
%                         fn = 0;
%                         v_numn = 0;
%                     end
%                     v_num = v_num + v_numn;
%                     v_deno = v_deno + fn;
%                 end
%                 
%                 if cnt ~= 0;
%                     v(i,j) = norm(v_num/v_deno);
%                 else
%                     v(i,j) = 0;
%                     %Jk = 0;
%                 end
%             end
%         end
%         % velocity variance
%         v_avg = sum(v(:))/(N*M);
%         for i = 1:M
%             for j = 1:N
%                 v_diff = [v_diff, (v(i,j) - v_avg)^2];
%             end
%         end
%         v_var = sum(v_diff)/(N*M);
%         prs = [prs; dens(t)*v_var];        
%     end
%     
%     pressure_table(iii,jjj) = sum(prs(101:500))/400;
end

% figure(1)
% hold on
% plot(0.1:0.1:1.5,accflow_table(1:5,:)','-o')
% legend('q_1/q_2=1/3','q_1/q_2=1/2','q_1/q_2=1/1','q_1/q_2=2/1','q_1/q_2=3/1')
% xlabel('Omega','Fontsize',12)
% ylabel('Accumulated outflow','Fontsize',12)
% set(gca,'FontSize',12);
% ylim([165,200])

figure(1)
hold on
grid on
plot(0:0.1:4,accflow_table(1,:)','-o')
legend('q_1/q_2=4/7','q_1/q_2=1/2','q_1/q_2=1/1','q_1/q_2=2/1','q_1/q_2=3/1')
xlabel('Omega','Fontsize',12)
ylabel('Accumulated outflow','Fontsize',12)
set(gca,'FontSize',12);

figure(2)
hold on
grid on
plot(0:0.1:4,dens_table(1,:)','-o')
legend('q_1/q_2=4/7','q_1/q_2=1/2','q_1/q_2=1/1','q_1/q_2=2/1','q_1/q_2=3/1')
xlabel('Omega','Fontsize',12)
ylabel('Crowd density','Fontsize',12)
set(gca,'FontSize',12);

figure(3)
hold on
grid on
plot(0:0.1:4,vel_table(1,:)','-o')
legend('q_1/q_2=4/7','q_1/q_2=1/2','q_1/q_2=1/1','q_1/q_2=2/1','q_1/q_2=3/1')
xlabel('Omega','Fontsize',12)
ylabel('Average velocity','Fontsize',12)
set(gca,'FontSize',12);

figure(4)
hold on
grid on
plot(0:0.1:4,flowrate_table(1,:)','-o')
legend('q_1/q_2=4/7','q_1/q_2=1/2','q_1/q_2=1/1','q_1/q_2=2/1','q_1/q_2=3/1')
xlabel('Omega','Fontsize',12)
ylabel('Average outflow','Fontsize',12)
set(gca,'FontSize',12);

% figure(4)
% hold on
% plot(0:0.1:1.5,pressure_table(1:5,:)','-o')
% legend('q_1/q_2=1/3','q_1/q_2=1/2','q_1/q_2=1/1','q_1/q_2=2/1','q_1/q_2=3/1')
% xlabel('Omega','Fontsize',12)
% ylabel('Crowd pressure','Fontsize',12)
% set(gca,'FontSize',12);
%%
% clc;clear all
% 
% accflow_loc = [];
% for iii=1:5
%     for jjj=1:11
%         xxx=(iii-1)*0.5+1;
%         yyy=(jjj-1)*0.5+2;
%         filename_ = ['250_v20r_',num2str(xxx),'_',num2str(yyy),'.mat'];
%         load(filename_)
%         accflow_loc(iii,jjj)=flowaccum_temp(300);
%     end
% end
% figure(1)
% hold on
% surf([1:0.5:3],[2:0.5:7],accflow_loc','Linestyle','None')
% axis([0,8,0,8]);
% xlabel('Position x (m)','FontSize',12)
% ylabel('Position y (m)','FontSize',12)
% set(gca,'FontSize',12);
% caxis([50,120])
% bdry = [[-8:0.01:8,4:0.01:8,4*ones(1,(12/0.01)+1)]',...
%     [8*ones(1,(16/0.01)+1),4*ones(1,(4/0.01)+1),-8:0.01:4]'];
% % Plotting the boundaries %
% plot(bdry(:,1),bdry(:,2),'kx')
% if (K == 1)
%     axis([0,8,0,8]);
% end
% hold on
% 
% %%
% clc;clear all
% for iii=1:5
%     for jjj=1:11
%         xxx=(iii-1)*0.5+1;
%         yyy=(jjj-1)*0.5+2;
%         filename_ = ['250_v20r_',num2str(xxx),'_',num2str(yyy),'.mat'];
%         load(filename_)
%         figure(2)
%         % plot temporal evolution of cost function (flow)
%         subplot(2,1,1)
%         plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,J_flow,'b','LineWidth',1);
%         title('Temporal evolution of flow','FontSize',12)
%         xlabel('Time (sec)','FontSize',12)
%         ylabel('Flow (1/m.s)','FontSize',12)
%         hold on
%         grid on
%         set(gca,'FontSize',12);
%         %plot accumulated flow
%         subplot(2,1,2)
%         plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowaccum_temp, 'b-','LineWidth',1);
%         title('Accumulated flow','FontSize',12)
%         xlabel('Time (sec)','FontSize',12)
%         ylabel('Accumulated flow','FontSize',12)
%         hold on
%         grid on
%         set(gca,'FontSize',12);
%     end
% end
% % figure(1)
% % hold on
% % surf([1:0.5:3],[2:0.5:7],accflow_loc','Linestyle','None')
% % axis([0,8,0,8]);
% % xlabel('Position x (m)','FontSize',12)
% % ylabel('Position y (m)','FontSize',12)
% % set(gca,'FontSize',12);
% % caxis([50,120])
% % bdry = [[-8:0.01:8,4:0.01:8,4*ones(1,(12/0.01)+1)]',...
% %     [8*ones(1,(16/0.01)+1),4*ones(1,(4/0.01)+1),-8:0.01:4]'];
% % % Plotting the boundaries %
% % plot(bdry(:,1),bdry(:,2),'kx')
% % if (K == 1)
% %     axis([0,8,0,8]);
% % end
% % hold on