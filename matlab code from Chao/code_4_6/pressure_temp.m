%% version 1 (spatial average of pressure in an observed region)%%
xl = 1; xh = 5;
yl = 3; yh = 8;
ds = 0.1;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
pres_loc = zeros(M,N);
v_var = zeros(M,N);
pres_mean_hist = [];
R = 1;
% calculate density distribution %
for t = 1:10:size(sampleSt,1)
    pres_mean = 0;
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
            pres_mean = pres_mean + pres_loc(i,j)/(N*M);
        end
    end
    pres_mean_hist = [pres_mean_hist; pres_mean];
end
figure(3)
hold on
plot(pres_mean_hist,'k')
xlabel('Time (sec)','FontSize',12)
ylabel('Crowd pressure (1/s^{2})','FontSize',12)
grid on
set(gca, 'FontSize',12)

%% version 2 (formular in figure 3)%%
ds = 0.1;
xl=1; xh=5;
yl=3; yh=8;
N = length(xl:ds:xh);
M = length(yl:ds:yh);
prs = [];
dens = [];
R = 1;
for t=1:10:size(sampleSt,1)    
    v = zeros(M,N);
    v_avg = 0;
    v_diff = [];
    v_var = 0;
    count = 0;
    % number of pedestrians
    for i=1:2:2*m
        if (sampleSt(t,2*m+i)>=xl && sampleSt(t,2*m+i)<=xh && sampleSt(t,2*m+i+1)>=yl && sampleSt(t,2*m+i+1)<=yh) % range of the specific section (or from 14 to 16)
            count = count + 1;
        end
    end
    dens(t) = count/[(xh-xl)*(yh-yl)];
    % local velocity
    for i = 1:M
        for j = 1:N          
            v_num = 0;
            v_deno = 0;
            cnt = 0;
            for n=1:2:2*m
                dist = norm([sampleSt(t,2*m+n)-(xl+ds*(j-1)), sampleSt(t,2*m+n+1)-(yl+ds*(i-1))]);
                if dist < R
                    fn = (pi*R^2)^(-1)*exp(-dist^2/R^2);
                    v_numn = [sampleSt(t,n),sampleSt(t,n+1)]*fn;
                    cnt = cnt + 1;
                else
                    fn = 0;
                    v_numn = 0;
                end
                v_num = v_num + v_numn;
                v_deno = v_deno + fn;
            end
            
            if cnt ~= 0;
                v(i,j) = norm(v_num/v_deno);
            else
                v(i,j) = 0;
                %Jk = 0;
            end
        end
    end
    % velocity variance
    v_avg = sum(v(:))/(N*M);
    for i = 1:M
        for j = 1:N
            v_diff = [v_diff, (v(i,j) - v_avg)^2];
        end
    end
    v_var = sum(v_diff)/(N*M);
    prs = [prs; dens(t)*v_var];
    
end
figure(2)
hold on
%plot([0:0.1:T],v_var,'r')
plot(prs,'k')
%plot([0:0.1:T],J,'k')
grid on

%% version 3 %
% % define region
% xl = 1; xh = 4;
% yl = 4; yh = 7;
% prs = [];
% v_var = [];
% J = [];
% for j=1:size(sampleSt,1)
%     count = 0;
%     v_sum = 0;
%    % v_sum2 = 0;
%     v_diff = [];
%     for i=1:2:2*m
%        if (sampleSt(j,2*m+i)>=xl && sampleSt(j,2*m+i)<=xh && sampleSt(j,2*m+i+1)>=yl && sampleSt(j,2*m+i+1)<=yh) % range of the specific section (or from 14 to 16)
%             v_sum = v_sum + [sampleSt(j,i),sampleSt(j,i+1)];
%             %v_sum = v_sum + sampleSt(j,i);
%             count = count + 1;
%        end
%     end
%     if count ~= 0;
%         %v_ave = sqrt(v_sum1^2+v_sum2^2)/count;
%         v_ave = v_sum/count;
%         % calculate velocity variance
%         for i=1:2:2*m
%            if (sampleSt(j,2*m+i)>=xl && sampleSt(j,2*m+i)<=xh && sampleSt(j,2*m+i+1)>=yl && sampleSt(j,2*m+i+1)<=yh)
%                 v_diff = [v_diff,(norm([sampleSt(j,i),sampleSt(j,i+1)]-v_ave))^2];
%                 %v_diff = [v_diff,(sampleSt(j,i)-v_ave)^2];
%            end
%         end
%         var = sum(v_diff)/count;
%         % cost function
%         Jk = count/((xh-xl)*(yh-yl)); % cost at this step, flow efficiency
%     else
%         var = 0;
%         v_ave = 0;
%         Jk = 0;
%     end    
%     %prs = [prs; v_ave(1)*count/((xh-xl)*(yh-yl))];
%     prs = [prs; var*count/((xh-xl)*(yh-yl))];
%     v_var = [v_var; var];
%     J = [J; Jk];
% end
% figure(2)
% hold on
% plot([0:0.1:T],v_var,'r')
% plot([0:0.1:T],prs,'b')
% plot([0:0.1:T],J,'k')
% set(gca,'FontSize',12);
% grid on
 
