%% plot pressure distribution at a given time instance ------------------%%
% ----------------  version 1 (spatial average of vel.) ------------------%
t = 170; % Define the timestep for which the figures are plotted
ds = 0.02; % spatial resolution
ti = 20;
xl = 1; xh = 5;
yl = 3; yh = 8;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
pres_loc = zeros(M,N);
flow_loc = zeros(M,N);
v_var = zeros(M,N);
R = 1;

% calculate pressure distribution %
for i = 1:M
    for j = 1:N
        dens = 0;
        cnt = 0;
        v_avg = 0;
        v_diff = [];
        v_num = 0;
        v_deno = 0;
        fn_all = 0;
        % calculate density
        for n = 1:2:2*m
            dist = norm([sampleSt(t,2*m+n)-(xl+ds*(j-1)), sampleSt(t,2*m+n+1)-(yl+ds*(i-1))]);
            if dist < R
                fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                v_numn = [sampleSt(t,n),sampleSt(t,n+1)]*fn;
                %v_numn = sampleSt(t,n)*fn;
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
                dist = norm([sampleSt(t,2*m+n)-(xl+ds*(j-1)), sampleSt(t,2*m+n+1)-(yl+ds*(i-1))]);
                if dist < R
                    fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                    v_diff = [v_diff, fn*(norm([sampleSt(t,n),sampleSt(t,n+1)] - v_avg))^2];
                    %v_diff = [v_diff, fn*(sampleSt(t,n)-v_avg)^2];
                    fn_all = fn_all + fn;
                end
            end
            v_var(i,j) = sum(v_diff)/fn_all;
        else
            v_var(i,j) = 0;
        end
        pres_loc(i,j) = dens*v_var(i,j);
        flow_loc(i,j) = dens*norm(v_avg);
    end
end
   
% plot pressure distribution
figure(3)
pres_loc = medfilt2(pres_loc,[30,30]);
surf([xl:ds:xh],[yl:ds:yh],pres_loc,'Linestyle','None')
alpha(0.8)
colorbar;
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
colormap(gca,'jet')
%caxis([0,0.191]) % Define a uniform scale for comparison purpose

% Plot velocity field on top of the pressure figure
ds = 0.2;
R = 0.8;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
[xv,yv] = meshgrid(xl:ds:xh,yl:ds:yh);
ux = [];
uy = [];
for i = 1:M
    for j = 1:N
        dens = 0;
        v_num = 0;
        cnt = 0;
        for n = 1:2:2*m
            dist = norm([sampleSt(t,2*m+n)-(xl+ds*(j-1)), sampleSt(t,2*m+n+1)-(yl+ds*(i-1))]);
            if dist < R
                fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                v_numn = [sampleSt(t,n),sampleSt(t,n+1)]*fn;
                cnt = cnt + 1;
            else
                fn = 0;
                v_numn = 0;
            end
            dens = dens + fn;
            v_num = v_num + v_numn;            
        end
        if cnt ~= 0
            ux(i,j) = v_num(1)/dens;
            uy(i,j) = v_num(2)/dens;
        else
            ux(i,j) = 0;
            uy(i,j) = 0;
        end
    end
end

figure(3)
hold on
quiver(xv,yv,ux,uy,'k','LineWidth',1,'AutoScaleFactor',0.6)
axis([1,5,3,8]);
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
view(0,90)  

% figure(5)
% flow_loc = medfilt2(flow_loc,[50,50]);
% % for i = 1:M
% %     for j = 1:N
% %         if (i <= (8-6)/ds && j >= (5-2.5)/ds)
% %             v_loc(i,j) = 0;
% %         end
% %     end
% % end
% surf([xl:ds:xh],[yl:ds:yh],flow_loc,'Linestyle','None')
% colorbar;
% %caxis([0,0.58])
% 
% xlabel('Position x (m)','FontSize',12)
% ylabel('Position y (m)','FontSize',12)
% set(gca,'FontSize',12);

%% ---------------- version 2 (temporal average of vel.) -----------------%
t = 96; % Define the timestep for which the figures are plotted
tw = 10; % Define time slot 
ds = 0.02; % spatial resolution
xl = 1; xh = 5;
yl = 3; yh = 7;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
pres_loc = zeros(M,N);
flow_loc = zeros(M,N);
v_var = zeros(M,N);
R = 0.8;

% calculate pressure distribution %
for i = 1:M
    for j = 1:N
        dens_all = [];
        v_all = [];
        v_diff = [];
        % calculate average density
        for tt = t-tw:t+tw
            dens = 0;
            cnt = 0;
            v_avg = 0;
            v_num = 0;
            v_deno = 0;
            for n = 1:2:2*m
                dist = norm([sampleSt(tt,2*m+n)-(xl+ds*(j-1)), sampleSt(tt,2*m+n+1)-(yl+ds*(i-1))]);
                if dist < R
                    fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                    v_numn = [sampleSt(tt,n),sampleSt(tt,n+1)]*fn;
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
            else
                v_avg = [0,0];
            end
            dens_all = [dens_all; dens];
            v_all = [v_all; v_avg];
        end
        dens_mean = mean(dens_all);
        v_mean = [mean(v_all(:,1)), mean(v_all(:,2))];
        for tt=1:length(t-tw:t+tw)
            v_diff = [v_diff, norm(v_all(tt,:)-v_mean)^2];
        end
        v_var(i,j) = mean(v_diff);
        pres_loc(i,j) = dens_mean*v_var(i,j);
        flow_loc(i,j) = dens*norm(v_avg);
    end
end
   
% plot pressure distribution
figure(22)
pres_loc = medfilt2(pres_loc,[50,50]);
surf([xl:ds:xh],[yl:ds:yh],pres_loc,'Linestyle','None')
alpha(0.8)
colorbar;
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
%caxis([0,0.191]) % Define a uniform scale for comparison purpose

% Plot velocity field on top of the pressure figure
ds = 0.2;
R = 0.8;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
[xv,yv] = meshgrid(xl:ds:xh,yl:ds:yh);
ux = [];
uy = [];
for i = 1:M
    for j = 1:N
        dens = 0;
        v_num = 0;
        cnt = 0;
        for n = 1:2:2*m
            dist = norm([sampleSt(t,2*m+n)-(xl+ds*(j-1)), sampleSt(t,2*m+n+1)-(yl+ds*(i-1))]);
            if dist < R
                fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                v_numn = [sampleSt(t,n),sampleSt(t,n+1)]*fn;
                cnt = cnt + 1;
            else
                fn = 0;
                v_numn = 0;
            end
            dens = dens + fn;
            v_num = v_num + v_numn;            
        end
        if cnt ~= 0
            ux(i,j) = v_num(1)/dens;
            uy(i,j) = v_num(2)/dens;
        else
            ux(i,j) = 0;
            uy(i,j) = 0;
        end
    end
end

figure(22)
hold on
quiver(xv,yv,ux,uy,'k','LineWidth',1,'AutoScaleFactor',0.6)
axis([1,5,3,7]);
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
view(0,90)  

%% plot snapshot ------------------------------------------------------%%
figure(7)
hold off
plot(bdry(:,1),bdry(:,2),'kx')
if (K == 1)
    axis([0,8,0,8]);
end
hold on
% Plotting the pedestrians %
for n = (2*m)+1:2:(4*m)
    if (n<=3*m)
        plot(sampleSt(t,n),sampleSt(t,n+1),'bo','MarkerSize',14,'LineWidth',2)
        quiver(sampleSt(t,n),sampleSt(t,n+1),sampleSt(t,n-2*m),sampleSt(t,n+1-2*m),'b','MaxHeadSize',0.8,'LineWidth',2,'AutoScaleFactor',0.4)
        if (K == 1)
            axis([0,8,0,8]);
        end
        hold on
    else
        plot(sampleSt(t,n),sampleSt(t,n+1),'ro','MarkerSize',14,'LineWidth',2)
        quiver(sampleSt(t,n),sampleSt(t,n+1),sampleSt(t,n-2*m),sampleSt(t,n+1-2*m),'r','MaxHeadSize',0.8,'LineWidth',2,'AutoScaleFactor',0.4)
        if (K == 1)
            axis([0,8,0,8]);
        end
        hold on
    end
end
% plotting the robot position  %
if (nr ~=0);
    for i = (2*nr)+1:2:(4*nr)
        plot(sampleSt(t,4*m+i),sampleSt(t,4*m+i+1),'mp','MarkerSize',14,'LineWidth',2)
        quiver(sampleSt(t,4*m+2*nr+1),sampleSt(t,4*m+2*nr+2),sampleSt(t,4*m+1),sampleSt(t,4*m+2),'m','MaxHeadSize',0.8,'LineWidth',2,'AutoScaleFactor',0.4)
        hold on
    end
end

% plot observed region
rectangle('position',[1,4,3,3],'LineWidth',2,'LineStyle','--')

xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
grid on
%% velocity distribution at a given time instance %%
% t = 60;
% ds = 0.02;
% ti = 20;
% N = length(2:ds:5);
% M = length(6:ds:10);
% dens_loc = zeros(M,N);
% v_loc = zeros(M,N);
% pres_loc = zeros(M,N);
% R = 0.8;
% % calculate density distribution %
% for i = 1:M
%     for j = 1:N
%         dens = 0;
%         cnt = 0;      
%         v_num = 0;
%         v_deno = 0;
%         fn_all = 0;
%         % calculate density 
%         for n = 1:2:2*m
%             dist = norm([sampleSt(t,2*m+n)-(2+ds*(j-1)), sampleSt(t,2*m+n+1)-(6+ds*(i-1))]);
%             if dist < R
%                 fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
%                 v_numn = [sampleSt(t,n),sampleSt(t,n+1)]*fn;
%                 %v_numn = norm(sampleSt(t,n),sampleSt(t,n+1))*fn;
%                 cnt = cnt + 1;
%             else
%                 fn = 0;
%                 v_numn = 0;
%             end
%             dens = dens + fn;
%             v_num = v_num + v_numn;
%             v_deno = v_deno + fn;
%         end
%         if cnt ~= 0;
%             dens_loc(i,j) = dens;
%             v_loc(i,j) = norm(v_num/v_deno);
%             pres_loc(i,j) = dens_loc(i,j)*v_loc(i,j);
%         else
%             dens_loc(i,j) = 0;
%             v_loc(i,j) = 0;
%             pres_loc(i,j) = 0;
%         end        
%     end
% end
% 
% % plot density distribution
% figure(1)
% v_loc = medfilt2(v_loc,[15,15]);
% surf([2:ds:5],[6:ds:10],v_loc,'Linestyle','None')
% colorbar;
% 
% figure(2)
% dens_loc = medfilt2(dens_loc,[15,15]);
% surf([2:ds:5],[6:ds:10],dens_loc,'Linestyle','None')
% colorbar;  
% 
% figure(3)
% pres_loc = medfilt2(pres_loc,[15,15]);
% surf([2:ds:5],[6:ds:10],pres_loc,'Linestyle','None')
% colorbar; 
% 
% % ---------------------------------------------%
% figure(4)
% hold off
% plot(bdry(:,1),bdry(:,2),'rx')
% if (K == 1)
%     axis([0,10,0,10]);
% end
% hold on
% % Plotting the pedestrians %
% for n = (2*m)+1:2:(4*m)
%     if (n<=3*m)
%         plot(sampleSt(t,n),sampleSt(t,n+1),'bo','MarkerSize',8)
%         if (K == 1)
%             axis([0,10,0,10]);
%         end
%         hold on
%     else
%         plot(sampleSt(t,n),sampleSt(t,n+1),'ro','MarkerSize',8)
%         if (K == 1)
%             axis([0,10,0,10]);
%         end
%         hold on
%     end
% end
% % plotting the robot position  %
% if (nr ~=0);
%     for i = (2*nr)+1:2:(4*nr)
%         plot(sampleSt(t,4*m+i),sampleSt(t,4*m+i+1),'mp','MarkerSize',8)
%         hold on
%     end
% end
    