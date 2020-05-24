data=csvread("../data/final/adacon600.csv")
t=data(:,1);
kp=data(:,2);
kv=data(:,3);
exp=data(:,4);

figure; hold on;grid on;

plot(t,kp);
plot(t,kv);
plot(t,exp);

legend('kp','kv','exp')