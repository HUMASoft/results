clear; close all;

fig=figure; hold on;grid on;
leg=[];

for i=0
data=csvread("../data/dist/ada"+num2str(i)+"00response.csv");
t=data(:,1);
d1=data(:,2);
d2=data(:,3);



plot(t,d1);
plot(t,d2,'--');

leg = [leg ;"Neck inc. "+num2str(i)+"00 g";"Motor pos. "+num2str(i)+"00 g"];

end

N=size(t,1);
ylabel(' Position (rad)          Inclination (deg)      ');
xlabel('time (sec)');
legend(leg,'Location','northwest');
saveas(fig,'../adadistimeResponse','epsc');





fig=figure; hold on;grid on;
leg=[];
for i=0
    data=csvread("../data/dist/adasysden"+num2str(i)+"00.csv");
    data2=csvread("../data/dist/adasysnum"+num2str(i)+"00.csv");
    t=data(:,1);
    d1=data(:,2);
    d2=data(:,3);
    d3=data(:,4);

    plot(t,d1);
    plot(t,d2);
    plot(t,d3);
    n1=data2(:,2);
    plot(t,n1);

    leg = [leg ;"Neck inc. "+num2str(i)+"00 g";"Motor pos. "+num2str(i)+"00 g"];

end

ylabel(' Estimated model parameters');
xlabel('time (sec)');
ylim([-1.5,1.5]);

% legend(leg,'Location','northwest');
saveas(fig,'../adadisparameters','epsc');





fig=figure; hold on;grid on;
leg=[];
for i=0
data=csvread("../data/dist/adasensor"+num2str(i)+"00response.csv");
    t=data(:,1);
    phi=data(:,3);
    mag=data(:,2);

    plot(t,phi,'--');
    plot(t,mag);


    leg = [leg ;"Phase "+num2str(i)+"00 g";"Magnitude "+num2str(i)+"00 g"];

end

ylabel(' Phase (rad)          Magnitude	      ');
xlabel('time (sec)');
ylim([-3,3]);
legend(leg,'Location','west');
saveas(fig,'../adadisphimag','epsc');




fig=figure; hold on;grid on;
leg=[];
for i=0
data=csvread("../data/dist/adacon"+num2str(i)+"00.csv");
%     t=data(:,1);
    kp=data(:,2);
    ka=data(:,3);
    exp=data(:,4);

    plot(t,kp,'--');
    plot(t,ka);
    plot(t,exp);


    leg = [leg ;"Kp "+num2str(i)+"00 g";"Ka "+num2str(i)+"00 g";"alpha "+num2str(i)+"00 g"];

end

ylabel(' Controller parameters');
xlabel('time (sec)');
legend(leg,'Location','northwest');
saveas(fig,'../adadiscon','epsc');

skp=mean(kp(800:N));
ska=mean(ka(800:N));
sexp=-1;



