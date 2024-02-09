a = load('E:\points.txt');
p = polyfit(a(:,1),a(:,2),3);
x = min(a(:,1)):0.01:max(a(:,1));
y = polyval(p,x);
figure
plot(x,y,'g.')
hold on
plot(a(:,1),a(:,2),'r.')
