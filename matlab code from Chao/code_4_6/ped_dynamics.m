% Social Force Model Based Pedestrian Dynamics %
%-------------------------------------------------------------------------%
function f0a = ped_dynamics(t,xi,ur,q)
global m bdry p p1 p2 D BDRY x0
global tau v0a0 v0ai vmax r A1a A1aa A2a A1r B1a B2a B1r lambda A1 B1 VS dist 
global nr rr 

x = zeros(m,2);
v = zeros(m,2);
xr = zeros(nr,2);
nrmpx = zeros(1,m);
f = zeros(1,2*m);
X = zeros(1,2*m);
% Putting pedestrian positions in a separate vector %
for i = 1:m
    x(i,1) = xi((2*i)-1+(2*m));
    x(i,2) = xi((2*i)+(2*m));
    v(i,1) = xi(2*i-1);
    v(i,2) = xi(2*i);
end

% robot velocity vector vr and position vector xr %
for j=1:nr
%     vr(j,1) = xi(4*m+(2*j)-1);
%     vr(j,2) = xi(4*m+(2*j));
    xr(j,1) = xi(4*m+(2*j)-1);
    xr(j,2) = xi(4*m+(2*j));
end

% Setting inital position for average speed calculation %
if (t == 0)
    x0 = x;
end

%% -------------------- Driving Force component -------------------------%%
% Desired Velocity %
for i = 1:m
    if x(i,1) <= -2 || x(i,2) <= -10
        v0a = 5*ones(1,m);
    else
        v0a = v0ai;
    end
%    if t == 0        
%         v0a(i) = v0a0;
%     else
%         V (i,:) = x(i,:) - x0(i,:);
%         Vbar = norm(V(i,:)/t);
%         na = 1 - (Vbar/v0a0);
%         v0a(i) = (1 - na)*v0a0 + na*vmax;
%     end
   
    % Desired destination p %
%    if (way == 1)
        % Pedestrians in area A %
        if (i <= m*q)
%             nrmpx(i) = norm(p(i,:) - x(i,:));
%             if (nrmpx(i) < dist)
%                 D(i) = 1;
%             end
%             if (D(i) == 1)
                p(i,:) = p1(1,:);
                nrmpx(i) = norm(p(i,:) - x(i,:));
%                 for j = 1:length(p1)
%                     if (norm(p1(j,:) - x(i,:)) < nrmpx(i))
%                         p(i,:) = p1(j,:);
%                         nrmpx(i) = norm(p(i,:) - x(i,:));
%                     end
%                 end
           % end
            % Pedestrians in area B %
        elseif (i > m*q)
            nrmpx(i) = norm(p(i,:) - x(i,:));
            %if (nrmpx(i) < dist)
            if x(i,2) > 4
                D(i) = 1;
            end
            if (D(i) == 1)
                p(i,:) = p2(1,:);
                nrmpx(i) = norm(p(i,:) - x(i,:));
%                 for j = 1:length(p2)
%                     if (norm(p2(j,:) - x(i,:)) < nrmpx(i))
%                         p(i,:) = p2(j,:);
%                         nrmpx(i) = norm(p(i,:) - x(i,:));
%                     end
%                 end
            end
        end
%     elseif (way == 0)
%         % Pedestrians in area A %
%         if (i <= m/2)
%             p(i,:) = p1(1,:);
%             nrmpx(i) = norm(p(i,:) - x(i,:));
%             for j = 1:length(p1)
%                 if (norm(p1(j,:) - x(i,:)) < nrmpx(i))
%                     p(i,:) = p1(j,:);
%                     nrmpx(i) = norm(p(i,:) - x(i,:));
%                 end
%             end
%             % Pedestrians in area B %
%         elseif (i > m/2)
%             p(i,:) = p2(1,:);
%             nrmpx(i) = norm(p(i,:) - x(i,:));
%             for j = 1:length(p2)
%                 if (norm(p2(j,:) - x(i,:)) < nrmpx(i))
%                     p(i,:) = p2(j,:);
%                     nrmpx(i) = norm(p(i,:) - x(i,:));
%                 end
%             end
%         end
%    end
    
    % Desired Direction vector  %    
    if (nrmpx(i) == 0)
        e = [0,0];
    else
        e = (p(i,:) - x(i,:))/(nrmpx(i));
    end
    
    % Driving Force component %
    f((2*i)-1) = (((v0a(i)/tau) * e(1)) - (xi((2*i)-1)/tau));
    f((2*i)) = (((v0a(i)/tau) * e(2)) - (xi((2*i))/tau));
    
%% ------------------ Pedestrian Interactions Force--------------------- %%
    fab = [0,0];
    for j = 1:m
        if (i ~= j)
            dab = norm(x(i,:) - x(j,:));
            if (dab < VS)
                rab = r(i) + r(j);
                nab = ((x(i,:) - x(j,:))/dab);
                fab = fab + ((A1a*exp((rab - dab)/B1a))*nab)...
                    *(lambda + (1 - lambda)*((1+(-nab*e'))/2))...
                    + ((A2a*exp((rab - dab)/B2a))*nab);
            end
        end
    end
    f((2*i)-1) = f((2*i)-1) + fab(1,1);
    f((2*i)) = f((2*i)) + fab(1,2);

    %------------------ interaction force model 2-------------------------%
%         fab = [0,0];
%         %k = 1.2*10^2; K = 2.4*10^2;
%         k = 20; K = 100;
%         for j = 1:m
%             if (i ~= j)
%                 dab = norm(x(i,:) - x(j,:));
%                 if (dab < VS)
%                     rab = r(i) + r(j);
%                     nab = ((x(i,:) - x(j,:))/dab);
%                     tab = [-nab(2),nab(1)];
%                     if (dab > rab)
%                         gab = 0;
%                     else 
%                         gab = rab - dab;
%                     end
%                     fab = fab + (A1aa*exp((rab - dab)/B1a)*...
%                          (lambda + (1 - lambda)*((1+(-nab*e'))/2))+...
%                          k*gab)*nab + K*gab*(v(j,:) - v(i,:))*tab'*tab;
%     
%                 end
%             end
%         end
%         f((2*i)-1) = f((2*i)-1) + fab(1,1);
%         f((2*i)) = f((2*i)) + fab(1,2);
%         
% ------------------- interaction force model 3 --------------------------%
%       fab = [0,0];
%       k = 2; D0 = 0.31; D1 = 0.45;
%       for j = 1:m
%           if (i~=j)
%               dab = norm(x(i,:) - x(j,:));
%               rab = r(i) + r(j);
%               if (dab < VS)
%                   nab = ((x(i,:) - x(j,:))/dab);
%                   fab = fab + A1aa*exp((rab - dab)/D0 + (D1/dab)^k)*nab*...
%                         (lambda + (1 - lambda)*((1+(-nab*e(i,:)'))/2));
%               end
%           end
%       end
%       f((2*i)-1) = f((2*i)-1) + fab(1,1);
%       f((2*i)) = f((2*i)) + fab(1,2);      

    
%% ---------------- Boundary Interaction Component ----------------------%%
    
%     % nearest boundary interactions %
%     if BDRY == 1
%         for j = 1:length(bdry)
%             if (j == 1)
%                 daB = norm(x(i,:) - bdry(j,:));
%                 BDY = bdry(j,:);
%                 rd = r(i) - daB;
%             elseif ((norm(x(i,:) - bdry(j,:))) < daB)
%                 daB = norm(x(i,:) - bdry(j,:));
%                 BDY = bdry(j,:);
%                 rd = r(i) - daB;
%             end
%         end
%         naB = (x(i,:) - BDY)/daB;
%         faB = ((A1*exp(rd/B1))*naB);
%         f((2*i)-1) = f((2*i)-1) + faB(1,1);
%         f((2*i)) = f((2*i)) + faB(1,2);
%     end

% ----------- optimized ---------- %
    if (x(i,2)>6)
        daB = abs(8 - x(i,2));
        rd = r(i) - daB;
        naB = [0,x(i,2)-8]/daB;
        faB = ((A1*exp(rd/B1))*naB);
    elseif (x(i,1)>=4 && x(i,2)<6)
        daB = abs(x(i,2) - 4);
        rd = r(i) - daB;
        naB = [0,x(i,2)-4]/daB;
        faB = ((A1*exp(rd/B1))*naB);
    elseif (x(i,1)<4 && x(i,2)<=4)
        daB = abs(x(i,1) - 4);
        rd = r(i) - daB;
        naB = [x(i,1)-4,0]/daB;
        faB = ((A1*exp(rd/B1))*naB);
    elseif (x(i,1)<4 && x(i,2)>4 && x(i,2)<6 && norm(x(i,:)-[4,4])<2)
        daB = norm(x(i,:)-[4,4]);
        rd = r(i) - daB;
        naB = (x(i,:)-[4,4])/daB;
        faB = ((A1*exp(rd/B1))*naB);        
    else        
        faB = [0,0]; 
    end
        f((2*i)-1) = f((2*i)-1) + faB(1,1);
        f((2*i)) = f((2*i)) + faB(1,2);
    
%% ---------------------- Human Robot Interation Force -------------------%%
%    lambda1 = 0;
    if (nr ~=0)
        far = [0,0];
        for j = 1:nr
            dar = norm(x(i,:) - xr(j,:));
            if (dar < VS)
                rar = r(i) + rr(j);
                nar = ((x(i,:) - xr(j,:))/dar);
                far = far + ((A1r*exp((rar - dar)/B1r))*nar);%...
 %                   *(lambda1 + (1 - lambda1)*((1+(-nar*e(i,:)'))/2));
 %                   + ((A2a*exp((rar - dar)/(1*B2a)))*nar);
            end
        end
        f((2*i)-1) = f((2*i)-1) + far(1,1);
        f((2*i)) = f((2*i)) + far(1,2);
    end
    
    % Displacement update %
    X((2*i)-1)= xi((2*i)-1);
    X(2*i) = xi(2*i);
end

%% ------------------------ Robot Dynamics ------------------------------%%
% if (nr ~=0)
%     ur(1) = 0;   % d2x/dt2 = -w*x + w*(C+A) -- C is peak position; A is magnitute
%     ur(2) = -w*xr(1,2)+w*(3.6+0.9);
%     %
%     Vr(1) = vr(1,1);
%     Vr(2) = vr(1,2);
% else
%     ur =[];
%     Vr =[];
% end

% ----case 1----- %
if (nr ~=0)
    Xr(1) = ur(1);
    Xr(2) = ur(2);
else
    Xr = [];
end

%% ----------------- Return Function Value ------------------------------%%
f0a = [f,X,Xr]';

