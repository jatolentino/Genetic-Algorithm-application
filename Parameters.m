%% Primera parte
clear all; close all; clc
filename = 'valve10050.txt';
filename1 = 'valve5050.txt';
%%
data = load(filename, '-ascii');
data1 = load(filename1, '-ascii');
t= data(:,1); % column 1 of the data text file is assigned the variable x
y = data(:,2); % column 2 is assigned the variable y
ta=data1(:,1);
ya=data1(:,2);
t1=t(10:211,1)-9;
y1=y(10:211,1)*0.01;
ta1=ta(12:998,1)-11;
ya1=ya(12:998,1)*0.01;
%&986
datos= [t1 y1];
datosa=[ta1 ya1];
%%
syms x
V1=[];
V2=[];
Q=[];
Q2=[];
involu=int(0.15*sqrt(1-(x/0.52)^2));
for i=1:201
V1(i)=vpa(subs(involu,0.52-y1(i+1))-subs(involu,0.52-y1(1)));
%vpa(int(0.15*sqrt(1-(x/0.52)^2),0.52-y1(i),0.52-y1(i+1)));
Q(i)=V1(i)/i;
end
%%
for k=1:986
V2(k)=vpa(subs(involu,0.52-ya1(k+1))-subs(involu,0.52-ya1(1)));
%vpa(int(0.15*sqrt(1-(x/0.52)^2),0.52-y1(i),0.52-y1(i+1)));
Q2(k)=V2(k)/k;
end
%Q=V1*
t2=t(2:202,1);
t2a=ta(2:987,1);
plot(t2,Q')
hold on
plot(t2a,Q2')
hold off
%%

