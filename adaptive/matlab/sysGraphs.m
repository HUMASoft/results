clear; close all;

dts=0.02;
z=tf('z',0.02);
s=tf('s');

datan=csvread("../data/final/adasysnum000.csv");
datad=csvread("../data/final/adasysden000.csv");

N=size(datan,2);
M=size(datad,2);


t=datan(:,1);
SZ=size(datan,1);


figure(1); hold on;
plot(t,datan(:,3:N));
figure; hold on;
plot(t,datad(:,4:M));

poles=[];
for i=1:SZ
poles=[poles, roots(datad(i,3:M))]; %#ok<*AGROW>
end



fig=figure;hold on;
plot(t,real(poles)');
plot(t,datan(:,3:N));

ylabel('Roots of model denominator');
xlabel('time (sec)');
legend('Pole 1', 'Pole 2', 'Gain','Location','best');
saveas(fig,'../sysConverge000','epsc');

figure;
plot(imag(poles)');

sys=tf(datan(fix(SZ/2),3:N),datad(fix(SZ/2),3:M),dts);
figure;
bode(sys);
