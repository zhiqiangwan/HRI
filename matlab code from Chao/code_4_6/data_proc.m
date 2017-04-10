%% v134
dens_loc_v = [dens_loc1(15:100), dens_loc2(10:50), dens_loc3(5:50)];
dens_loc_f = [dens_loc1, dens_loc2(1:60), dens_loc3(180:200)];
flowrate = [flowrate1, flowrate2(1:60), flowrate3(180:200)];
var_loc = [var_loc1(15:100), var_loc2(10:50), var_loc3(5:50)];

figure(3)
% vel_fit = polyfit(dens_loc,var_loc,2);
% z = polyval(vel_fit,dens_loc);
hold on
plot(dens_loc_v,var_loc,'b.','MarkerSize',15)
%plot(dens_loc,z,'r-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Average velocity (m/s)','FontSize',12)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
% flow_fit = polyfit(dens_loc,flowrate/3,2);
% z = polyval(flow_fit,dens_loc);
hold on
plot(dens_loc_f,flowrate/3,'b.','MarkerSize',15)
%plot(dens_loc,z,'b-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Flow rate (1/s)','FontSize',12)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

%% v20
dens_loc_v = [dens_loc1(1:70), dens_loc2(2:40), dens_loc3(3:40)];
dens_loc_f = [dens_loc1(1:70), dens_loc2(2:40), dens_loc3(3:40)];
flowrate = [flowrate1(1:70), flowrate2(2:40), flowrate3(3:40)];
var_loc = [var_loc1(1:70), var_loc2(2:40), var_loc3(3:40)];

figure(3)
% vel_fit = polyfit(dens_loc,var_loc,2);
% z = polyval(vel_fit,dens_loc);
hold on
plot(dens_loc_v,var_loc,'b.','MarkerSize',15)
%plot(dens_loc,z,'r-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Average velocity (m/s)','FontSize',12)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
% flow_fit = polyfit(dens_loc,flowrate/3,2);
% z = polyval(flow_fit,dens_loc);
hold on
plot(dens_loc_f,flowrate/3,'b.','MarkerSize',15)
%plot(dens_loc,z,'b-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Flow rate (1/s)','FontSize',12)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

%% v25
dens_loc_v = [dens_loc1(1:60), dens_loc2(1:40), dens_loc3(2:40)];
dens_loc_f = [dens_loc1(1:60), dens_loc2(1:40), dens_loc3(2:40)];
flowrate = [flowrate1(1:60), flowrate2(1:40), flowrate3(2:40)];
var_loc = [var_loc1(1:60), var_loc2(1:40), var_loc3(2:40)];

figure(3)
% vel_fit = polyfit(dens_loc,var_loc,2);
% z = polyval(vel_fit,dens_loc);
hold on
plot(dens_loc_v,var_loc,'b.','MarkerSize',15)
%plot(dens_loc,z,'r-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Average velocity (m/s)','FontSize',12)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
% flow_fit = polyfit(dens_loc,flowrate/3,2);
% z = polyval(flow_fit,dens_loc);
hold on
plot(dens_loc_f,flowrate/3,'b.','MarkerSize',15)
%plot(dens_loc,z,'b-')
xlabel('Density (1/m^{2})','FontSize',12)
ylabel('Flow rate (1/s)','FontSize',12)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

%% plot all
clear all
load v25-3.mat
x=0:0.1:3.8;
y1=-0.82*x.^2+3.4.*x-0.26;
y2=-0.093*x.^2-0.26.*x+2.6;

figure(3)
% vel_fit = polyfit(dens_loc,var_loc,2);
% z = polyval(vel_fit,dens_loc);
hold on
f1 = plot(dens_loc_v,1*var_loc,'k.','MarkerSize',15);
plot(x,y2,'k-.')
xlabel('Density \rho (1/m^{2})','FontSize',12)
ylabel('Average velocity \bar{v} (m/s)','FontSize',12)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
% flow_fit = polyfit(dens_loc,flowrate/3,2);
% z = polyval(flow_fit,dens_loc);
hold on
f11=plot(dens_loc_f,1*flowrate/3,'k.','MarkerSize',15);
plot(x,y1,'k-.')
xlabel('Density \rho (1/m^{2})','FontSize',14)
ylabel('Flow q (1/m.s)','FontSize',14)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

% v20
load v20-3.mat
x=0:0.1:3.5;
y1=-0.77*x.^2+2.9.*x-0.28;
y2=-0.13*x.^2-0.056.*x+1.9;

figure(3)
% vel_fit = polyfit(dens_loc,var_loc,2);
% z = polyval(vel_fit,dens_loc);
hold on
f2 = plot(dens_loc_v,0.95*var_loc,'r.','MarkerSize',15);
plot(x,y2,'r--')
xlabel('Density \rho (1/m^{2})','FontSize',14)
ylabel('Average velocity \bar{v} (m/s)','FontSize',14)
grid on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
% flow_fit = polyfit(dens_loc,flowrate/3,2);
% z = polyval(flow_fit,dens_loc);
hold on
f22=plot(dens_loc_f,0.95*flowrate/3,'r.','MarkerSize',15);
plot(x,y1,'r--')
xlabel('Density \rho (1/m^{2})','FontSize',14)
ylabel('Flow q (1/m.s)','FontSize',14)
grid on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);

% v134
load v134-3.mat
x=0:0.1:3;
y1=-0.49*x.^2+1.7.*x-0.029;
y2=-0.004*x.^2-0.49.*x+1.6;
figure(3)
% vel_fit = polyfit(dens_loc,var_loc,2);
% z = polyval(vel_fit,dens_loc);
hold on
f3 = plot(dens_loc_v,1*var_loc,'b.','MarkerSize',15);
plot(x,y2,'b-')
xlabel('Density \rho (1/m^{2})','FontSize',14)
ylabel('Average velocity $$\bar{v}$$ (m/s)','FontSize',14)
legend([f1,f2,f3],'$$v_i^0=\mathcal{N}(2.5,0.3^2)$$','$$v_i^0=\mathcal{N}(2,0.3^2)$$','$$v_i^0=\mathcal{N}(1.34,0.3^2)$$')
grid on
box on
%axis([0.2 2.2 0.2 1.6])
set(gca,'FontSize',12);

% plot flowrate vs. density
figure(4)
% flow_fit = polyfit(dens_loc,flowrate/3,2);
% z = polyval(flow_fit,dens_loc);
hold on
f33=plot(dens_loc_f,1*flowrate/3,'b.','MarkerSize',15);
plot(x,y1,'b-')
xlabel('Density \rho (1/m^{2})','FontSize',14)
ylabel('Flow q (1/m.s)','FontSize',14)
legend([f11,f22,f33],'$$v_i^0=\mathcal{N}(2.5,0.3^2)$$','$$v_i^0=\mathcal{N}(2,0.3^2)$$','$$v_i^0=\mathcal{N}(1.34,0.3^2)$$')
grid on
box on
%axis([0 2.2 0 1.4])
set(gca,'FontSize',12);
