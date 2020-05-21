data=csvread("/home/humasoft/Escritorio/time-response.csv")
t=data(:,1);
incli=data(:,2);
motor=data(:,3);


figure; hold on;grid on;

plot(t,incli);
plot(t,motor);
legend('incli deg','motor pos rad')