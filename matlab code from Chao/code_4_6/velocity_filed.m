%% plot velocity field at a given time instance ------------------------%%
t = 50;
xl = 1; xh = 5;
yl = 3; yh = 7;
ds = 0.2;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
[xv,yv] = meshgrid(xl:ds:xh,yl:ds:yh);
ux = [];
uy = [];
R = 0.6;
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

figure(2)
hold on
quiver(xv,yv,ux,uy,'k','LineWidth',1,'AutoScaleFactor',0.6)
axis([1,5,3,7]);
xlabel('Position x (m)','FontSize',12)
ylabel('Position y (m)','FontSize',12)
set(gca,'FontSize',12);
