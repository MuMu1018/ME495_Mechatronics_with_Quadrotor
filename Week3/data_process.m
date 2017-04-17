data=csvread('data.csv');
figure
hold on
for k=1:3
    plot(data(:,k));
end

figure
hold on
for k=4:6
    plot(data(:,k));
end