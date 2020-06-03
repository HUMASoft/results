
data=csvread("../data/rev2/ada000response.csv")
t=data(:,1);
incli=data(:,2);
motor=data(:,3);


fig=figure; hold on;grid on;

plot(t,incli);
plot(t,motor);

ylabel(' Position (rad)          Inclination (deg)      ');
xlabel('time (sec)');
legend('Neck inclination', 'Motor position','Location','northwest');
saveas(fig,'../timeResponse000','epsc');



