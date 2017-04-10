%% --------------- Flow fundamental diagram v1 (local)------------------------------%
x = 2.5; y = 5; % define region
Ro = 1.5;
tf = 150; % define final timestep
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
figure(3)
vel_fit = polyfit(dens_loc(5:tf),var_loc(5:tf),2);
z = polyval(vel_fit,dens_loc(5:tf));
hold on
plot(dens_loc(5:tf),var_loc(5:tf),'r.','MarkerSize',15)
plot(dens_loc(5:tf),z,'r-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Average velocity (m/s)','FontSize',12)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
flow_fit = polyfit(dens_loc(1:tf),flowrate(1:tf),2);
z = polyval(flow_fit,dens_loc(1:tf));
hold on
plot(dens_loc(1:tf),flowrate(1:tf),'b.','MarkerSize',15)
plot(dens_loc(1:tf),z,'b-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Flow rate (1/s)','FontSize',12)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

%% ------------- Flow fundamental diagram v2 (global)--------------------------- %%
xl = 1; xh = 4;
yl = 4; yh = 7;
tf = 100; % define final timestep
dens_glob = [];
var_glob = [];
for t = 1:size(sampleSt,1)
    dens = 0;
    cnt = 0;
    v_avg = 0;
    v_num = 0;
    % calculate density
    for n = 1:2:2*m
        if (sampleSt(t,2*m+n)<=xh && sampleSt(t,2*m+n)>=xl && sampleSt(t,2*m+n+1)<=yh && sampleSt(t,2*m+n+1)>=yl)
%            fn = (pi*Ro^2)^(-1)*exp(-dist^2/Ro^2);
%            v_numn = sampleSt(t,n)*fn;
            v_num = v_num + sampleSt(t,n);
            cnt = cnt + 1;
%        else
%             fn = 0;
%             v_numn = 0;
        end
%         dens = dens + fn;
%         v_num = v_num + v_numn;
%         v_deno = v_deno + fn;
    end
    if cnt ~= 0;
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
        dens = cnt/9;
        v_avg = v_num/cnt;
    else
        v_avg = 0;
        dens = 0;
%        v_var = 0;
    end
%     pres_loc(t) = dens*v_var;
%     dens_loc(t) = dens;
%     var_loc(t) = v_avg(1);
%     flowrate(t) = 3*dens*v_avg(1);
      dens_loc(t) = dens;
      var_loc(t) = v_avg;
      flowrate(t) = dens*v_avg;
end
% plot velocity vs. density
figure(3)
vel_fit = polyfit(dens_loc(5:tf),var_loc(5:tf),2);
z = polyval(vel_fit,dens_loc(5:tf));
hold on
plot(dens_loc(5:tf),var_loc(5:tf),'r.','MarkerSize',15)
plot(dens_loc(5:tf),z,'r-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Average velocity (m/s)','FontSize',12)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
flow_fit = polyfit(dens_loc(1:tf),flowrate(1:tf),2);
z = polyval(flow_fit,dens_loc(1:tf));
hold on
plot(dens_loc(1:tf),flowrate(1:tf),'b.','MarkerSize',15)
plot(dens_loc(1:tf),z,'b-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Flow rate (1/s)','FontSize',12)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

%%
% plot flowrate vs. time
figure(3)
hold on
plot([0:0.1:T],flowrate,'k','LineWidth',2)
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('Flow rate (persons/s)','FontSize',12)
%legend('Average flow velocity','Density','Flow rate')
set(gca,'FontSize',12);

% plot density vs. time
figure(4)
hold on
plot([0:0.1:T],dens_loc,'b','LineWidth',2)
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('Density (1/m^{2})','FontSize',12)
%legend('Average flow velocity','Density','Flow rate')
set(gca,'FontSize',12);

% accumulated number of pedestrians passed through %
figure(5)
hold on
flow_accum = [];
flow_sum = 0;
for i = 1:tf
    flow_sum = flow_sum + flowrate(i);
    flow_accum(i) = flow_sum;
end
plot([0.1:0.1:tf/10],flow_accum*0.1,'LineWidth',2)
xlabel('Time (sec)','FontSize',12)
ylabel('Number of pedestrians passed through (persons)','FontSize',12)
grid on
set(gca,'FontSize',12);
% add close-up window
axes('position',[0.20,0.60,0.35,0.30])
plot([0.1:0.1:tf/10],flow_accum*0.1,'LineWidth',2)
axis([6 7 14 20]);
grid on
set(gca,'FontSize',12);


