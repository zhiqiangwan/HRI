%% version 3 (spatial average of pressure in an observed region)%%
x = 2.5; y = 5; %define location to be measured
pres_loc = [];
dens_loc = [];
var_loc = [];
R = 0.8;

% calculate local pressure %
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
            dist = norm([sampleSt(t,2*m+n)-x, sampleSt(t,2*m+n+1)-y]);
            if dist < R
                fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                v_diff = [v_diff, fn*(norm([sampleSt(t,n),sampleSt(t,n+1)] - v_avg))^2];
                %v_diff = [v_diff, fn*(sampleSt(t,n)-v_avg)^2];
                fn_all = fn_all + fn;
            end
        end
        v_var = sum(v_diff)/fn_all;
    else
        v_var = 0;
    end
    pres_loc(t) = dens*v_var;
    dens_loc(t) = dens;
    var_loc(t) = v_var;
    %flowrate(t) = dens*v_avg(1);
end

figure(2)
hold on
plot([0:0.1:T],pres_loc,'k')
plot([0:0.1:T],dens_loc,'b')
plot([0:0.1:T],var_loc,'r')
legend('local pressure','local density','local velocity variance')
set(gca,'FontSize',12);
grid on

% figure(3)
% hold on
% plot([0:0.1:T],flowrate,'k')
% plot([0:0.1:T],dens_loc,'b')
% plot([0:0.1:T],var_loc,'r')
% grid on