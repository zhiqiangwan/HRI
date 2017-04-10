%% plot density distribution at a given time instance  ------------------%% 
t = 277; % Define the timestep for which the figures are plotted
ds = 0.02;
xl = 1; xh = 5;
yl = 3; yh = 7;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
pre_dis = zeros(M,N);
v_avg = zeros(M,N);
R = 0.8;
% calculate density distribution %
for i = 1:M
    for j = 1:N
        dens = 0;
        v_num = 0;
        v_deno = 0;
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
            v_deno = v_deno + fn;
        end
        if cnt ~= 0
            pre_dis(i,j) = dens;%*(v_num/v_deno);
            v_avg(i,j) = norm(v_num/v_deno);
        else
            pre_dis(i,j) = 0;
            v_avg(i,j) = 0;
        end
    end
end

% plot density distribution
figure(3)
pre_dis = medfilt2(pre_dis,[50,50]);
surf([xl:ds:xh],[yl:ds:yh],pre_dis,'Linestyle','None')
alpha(0.8)
colorbar;
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
%caxis([0,2.4]) % Define a uniform scale for comparison purpose

% Plot velocity field on top of the density figure
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
axis([1,5,3,7]);
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
view(0,90)

%% plot snapshot ------------------------------------------------------%
figure(4)
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
rectangle('position',[1,4,3,3],'LineWidth',2,'LineStyle','--');

xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
grid on
    