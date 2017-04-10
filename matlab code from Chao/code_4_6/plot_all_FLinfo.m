function []=plot_all_FLinfo(ii,jj,index,xm,ym)

xx=(ii-1)*0.5+1;
yy=(jj-1)*0.5+2;
filename_ = ['260w',num2str(ii),'v20q8r','.mat'];
load(filename_)

switch index
    case 0 
%% ---------- outflow at bottleneck (local) ----------------- %%
        x = 0; y = 6; % define region x=4,y=5;
        Ro = 2; % 3
        pres_loc = [];
        dens_loc = [];
        var_loc = [];
        
        for t = 1:size(sampleSt,1)
            dens = 0;
            cnt = 0;
            v_avg = 0;
            v_diff = [];
            v_var = 0;
            v_num = 0;
            v_deno = 0;
            fn_all = 0;
            % calculate density
            for n = 1:2:2*m
                dist = norm([sampleSt(t,2*m+n)-x, sampleSt(t,2*m+n+1)-y]);
                if dist < Ro
                    fn = (pi*Ro^2)^(-1)*exp(-dist^2/Ro^2);
                    v_numn = sampleSt(t,n)*fn;
                    cnt = cnt + 1;
                else
                    fn = 0;
                    v_numn = 0;
                end
                dens = dens + fn;
                v_num = v_num + v_numn;
                v_deno = v_deno + fn;
            end
            if cnt ~= 0;
                v_avg = v_num/v_deno;
                for n = 1:2:2*m
                    dist = norm([sampleSt(t,2*m+n)-x, sampleSt(t,2*m+n+1)-y]);
                    if dist < Ro
                        fn = (pi*Ro^2)^(-1)*exp(-dist^2/Ro^2);
                        v_diff = [v_diff, fn*(sampleSt(t,n)-v_avg)^2];
                        fn_all = fn_all + fn;
                    end
                end
                v_var = sum(v_diff)/fn_all;
            else
                v_avg = 0;
                v_var = 0;
            end
            pres_loc(t) = dens*v_var;
            dens_loc(t) = dens;
            var_loc(t) = v_avg(1);
            flowrate(t) = dens*v_avg(1);
        end
        % plot velocity vs. density
        figure(1)
        hold on
        plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,dens_loc,'b','LineWidth',1)
        xlabel('Time (sec)','FontSize',12)
        ylabel('Crowd density (1/m^2)','FontSize',12)
        grid on
        %axis([0.2 2.2 0.2 1.6])
        set(gca,'FontSize',12);
        
        figure(2)
        hold on
        plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,var_loc,'b','LineWidth',1)
        xlabel('Time (sec)','FontSize',12)
        ylabel('Velocity (m/s)','FontSize',12)
        grid on
        %axis([0.2 2.2 0.2 1.6])
        set(gca,'FontSize',12);
        
        figure(3)
        hold on
        plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate,'b','LineWidth',1)
        xlabel('Time (sec)','FontSize',12)
        ylabel('Outflow (1/m.s)','FontSize',12)
        grid on
        %axis([0.2 2.2 0.2 1.6])
        set(gca,'FontSize',12);
        
    case 1
        %% ------------------- outflow at bottleneck (avg.) ----------------- %%
        flowrate_q = [];
        flowrate_qsum = [];
        flowrate_d = [];
        flowrate_dsum = [];
        for i = 1:10:size(sampleSt,1)
            dens = 0;
            cnt = 0;
            v_num = 0;
            v_avg = 0;
            % calculate density
            for n = 1:2:2*m
                if (sampleSt(i,2*m+n)>4 && sampleSt(i,2*m+n)<7 && ...
                        sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<8)
                    v_num = v_num + sampleSt(i,n);
                    cnt = cnt + 1;
                end
            end
            if cnt ~= 0;
                dens = cnt/12;
                v_avg = v_num/cnt;
            end
            flowrate_q = [flowrate_q; v_avg*dens];
            flowrate_qsum = [flowrate_qsum; sum(flowrate_q)];
            flowrate_d = [flowrate_d; (dens*v_avg-5)^2];
            flowrate_dsum = [flowrate_dsum; sum(flowrate_d)];
        end
        
        %if flowaccum_temp(300)>95.22
            figure(4)
            subplot(2,1,1)
            plot(0:0.1*length(sampleSt(:,1))-0.1,flowrate_q,'b','LineWidth',1)            
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (1/m.s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Accumulated Outflow (1/m)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            
            figure(5)
            subplot(2,1,1)
            plot(0:0.1*length(sampleSt(:,1))-0.1,flowrate_d,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Reward r(t)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1*length(sampleSt(:,1))-0.1,flowrate_dsum,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Cost function J(t)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
       % end
        
    case 2
        %% outflow counted at x=8 %%
        figure(6)
        outflow_sec = 0;
        for i = 1:size(outflow,2)/10
            outflow_sec = [outflow_sec; sum(outflow((i-1)*10+1:(i-1)*10+10))];
        end
        subplot(2,1,1)
        plot(0:0.1*length(sampleSt(:,1)),outflow_sec,'k','LineWidth',1)
        xlabel('Time (sec)','FontSize',12)
        ylabel('Outflow (1/m.s)','FontSize',12)
        grid on
        hold on
        %axis([0.2 2.2 0.2 1.6])
        set(gca,'FontSize',12);
        subplot(2,1,2)
        plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,outflow_sum,'k','LineWidth',1)
        xlabel('Time (sec)','FontSize',12)
        ylabel('Accumulated Outflow (1/m)','FontSize',12)
        grid on
        hold on
        %axis([0.2 2.2 0.2 1.6])
        set(gca,'FontSize',12);
        
    case 3
        %% outflow counted at x=4 %%
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
%                 if (sampleSt(t,2*m+i+1)>3)
%                     cnt1 = cnt1 + 1;
%                 end
%                 if (sampleSt(t,2*m+i+1)>3 && sampleSt(t,2*m+i)<8)
%                     cnt2 = cnt2 + 1;
%                 end
%                 if (sampleSt(t,2*m+i)>0 && sampleSt(t,2*m+i+1)>4)
%                     cnt1 = cnt1 + 1;
%                 end
%                 if (sampleSt(t,2*m+i)>0 && sampleSt(t,2*m+i)<8 && sampleSt(t,2*m+i+1)>4)
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
        
        
      %  if flowaccum_temp(300)>95.22
            figure(7)
            subplot(2,1,1)
            plot(0:0.1*length(sampleSt(:,1)),outflow_sec,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (1/m.s)','FontSize',12)
            %title(['x=',num2str(xx),',','y=',num2str(yy)],'FontSize',12)
            set(gca,'FontSize',12);
            grid on
           % hold on
            
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',1)                
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (1/m)','FontSize',12)
            set(gca,'FontSize',12);
            grid on
           % hold on
            
            figure(8)
            subplot(2,1,1)
            plot(flowrate_qsmth,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (1/m.s)','FontSize',12)
            %title(['x=',num2str(xx),',','y=',num2str(yy)],'FontSize',12)
            set(gca,'FontSize',12);
            grid on
           % hold on
            
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (1/m)','FontSize',12)
            set(gca,'FontSize',12);
            grid on
           % hold on
      %  end
    case 4
        %% ------------------- Average velocity and density in specific area (after bottleneck, smoothed)----------------- %%
        vel_avg = [];
        rho_avg = [];
        flowrate_q = [];
        flowrate_qsum = [];
        flowrate_qsmth = [];
        
        for i = 1:size(sampleSt,1)
            dens = 0;
            cnt = 0;
            v_num = 0;
            v_avg = 0;
            % calculate density
            for n = 1:2:2*m
                if (sampleSt(i,2*m+n)>4 && sampleSt(i,2*m+n)<7 && ...
                        sampleSt(i,2*m+n+1)>4 && sampleSt(i,2*m+n+1)<8)
                    v_num = v_num + sampleSt(i,n);
                    cnt = cnt + 1;
                end
            end
            if cnt ~= 0;
                dens = cnt/12;
                v_avg = v_num/cnt;
            end
            vel_avg = [vel_avg; v_avg];
            rho_avg = [rho_avg; dens];
            flowrate_q = [flowrate_q; v_avg*dens];
            win = 50; % moving average window size
            if length(flowrate_q)<win
                flowrate_qsmth = [flowrate_qsmth; sum(flowrate_q)/length(flowrate_q)];
            else
                flowrate_qsmth = [flowrate_qsmth; sum(flowrate_q(length(flowrate_q)-(win-1):length(flowrate_q)))/win];
            end
            flowrate_qsum = [flowrate_qsum; 0.1*sum(flowrate_q)];
        end
        
        if flowaccum_temp(300)>95.22
            figure(9)
            subplot(2,1,1)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,vel_avg,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Average velocity (m/s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,rho_avg,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Density (1/m^2)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            
            figure(10)
            subplot(2,1,1)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsmth,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (1/m.s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Accumulated outflow (1/m)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
        end

    case 5
        %% ------------------- Average velocity and density in specific area (before bottleneck)----------------- %%
        vel_avg = [];
        rho_avg = [];
        flowrate_q = [];
        flowrate_qsum = [];
        flowrate_qsmth = [];
        
        for i = 1:size(sampleSt,1)
            dens = 0;
            cnt = 0;
            v_num = 0;
            v_avg = 0;
            % calculate density
            for n = 1:2:2*m
                if (sampleSt(i,2*m+n)>2 && sampleSt(i,2*m+n)<4 && ...
                        sampleSt(i,2*m+n+1)>5 && sampleSt(i,2*m+n+1)<7)
                    v_num = v_num + sampleSt(i,n);
                    cnt = cnt + 1;
                end
            end
            if cnt ~= 0;
                dens = cnt/4;
                v_avg = v_num/cnt;
            end
            vel_avg = [vel_avg; v_avg];
            rho_avg = [rho_avg; dens];
            flowrate_q = [flowrate_q; v_avg*dens];
            win = 50; % moving average window size
            if length(flowrate_q)<win
                flowrate_qsmth = [flowrate_qsmth; sum(flowrate_q)/length(flowrate_q)];
            else
                flowrate_qsmth = [flowrate_qsmth; sum(flowrate_q(length(flowrate_q)-(win-1):length(flowrate_q)))/win];
            end
            flowrate_qsum = [flowrate_qsum; 0.1*sum(flowrate_q)];
        end
        
%        if flowaccum_temp(300)>95.22
            figure(100)
            subplot(2,1,1)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_q,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (m/s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (m/s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            
            figure(101)
            subplot(2,1,1)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsmth,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (m/s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,flowrate_qsum,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Outflow (m/s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            
            figure(102)
            subplot(2,1,1)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,vel_avg,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Average velocity (m/s)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
            subplot(2,1,2)
            plot(0:0.1:0.1*length(sampleSt(:,1))-0.1,rho_avg,'b','LineWidth',1)
            xlabel('Time (sec)','FontSize',12)
            ylabel('Density (1/m^2)','FontSize',12)
            grid on
            hold on
            %axis([0.2 2.2 0.2 1.6])
            set(gca,'FontSize',12);
%        end
end
