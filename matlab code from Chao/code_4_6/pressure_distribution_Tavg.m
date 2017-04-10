% pressure distribution %
t = 30;
ds = 0.01;
ti = 10;
xl = 2; xh = 6;
yl = 3; yh = 6;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
pre_dis = zeros(M,N);
v_var = zeros(M,N);
dens_all = zeros(M,N);
R = 0.8;
% calculate density distribution %
for i = 1:M
    for j = 1:N
        dens_avg = 0;
        v_avg = 0;             
        v_tau = [];
        dens_tau = [];
        % calculate temporal average velocity at current potision
        for tau = t-ti:t+ti
            dens = 0;
            cnt = 0;
            v_num = 0;
            v_deno = 0;
            for n = 1:2:2*m
                dist = sqrt((sampleSt(tau,2*m+n)-(xl+ds*(j-1)))^2 + (sampleSt(tau,2*m+n+1)-(yl+ds*(i-1)))^2);
                if dist < R
                    fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                    v_numn = [sampleSt(tau,n),sampleSt(tau,n+1)]*fn;
                    cnt = cnt + 1;
                else
                    fn = 0;
                    v_numn = [0,0];
                end
                dens = dens + fn;
                v_num = v_num + v_numn;
                v_deno = v_deno + fn;
            end
            if cnt ~= 0
                dens_tau = [dens_tau; dens];
                v_tau = [v_tau; v_num/v_deno];
            else
                dens_tau = [dens_tau; 0];
                v_tau = [v_tau; [0,0]];
            end
        end
        dens_avg = sum(dens_tau)/length(t-ti:t+ti);
        v_avg = sum(v_tau)/length(t-ti:t+ti);
        v_diff = [];
        for dt = 1:2*ti+1
            v_diff = [v_diff, (norm(v_tau(dt,:) - v_avg))^2];
        end
        v_var(i,j) = sum(v_diff)/length(t-ti:t+ti);
        pre_dis(i,j) = dens_avg*v_var(i,j);
    end
end

% plot density distribution
figure(1)
pre_dis = medfilt2(pre_dis,[10,10]);
surf([xl:ds:xh],[yl:ds:yh],pre_dis,'Linestyle','None')
colorbar;

% figure(2)
% dens_all = medfilt2(dens_all,[5,5]);
% surf([3:ds:6],[7:ds:10],dens_all,'Linestyle','None')
% colorbar;    

% ---------------------------------------------%
figure(3)
hold off
plot(bdry(:,1),bdry(:,2),'kx')
if (K == 1)
    axis([0,8,0,8]);
end
hold on
% Plotting the pedestrians %
for n = (2*m)+1:2:(4*m)
    if (n<=3*m)
        plot(sampleSt(t,n),sampleSt(t,n+1),'bo','MarkerSize',8)
        if (K == 1)
            axis([0,8,0,8]);
        end
        hold on
    else
        plot(sampleSt(t,n),sampleSt(t,n+1),'ro','MarkerSize',8)
        if (K == 1)
            axis([0,8,0,8]);
        end
        hold on
    end
end
% plotting the robot position  %
if (nr ~=0);
    for i = (2*nr)+1:2:(4*nr)
        plot(sampleSt(t,4*m+i),sampleSt(t,4*m+i+1),'mp','MarkerSize',8)
        hold on
    end
end