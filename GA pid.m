clear all; close all; clc
syms h1 h2 u
He=0.52;
beta1=0.15;
yref=0.2;
h1op=yref;
udis=100;
Ts=0.1;
uop=(0.0000000702626295*h1op^3 - 0.000000123003337*h1op^2 + ...
0.0000000734410693*h1op + 0.0000000060014657)*udis^1.82800065;
Fu1=(u-(0.0000000702626295*h1^3 - 0.000000123003337*h1^2 +...
+0.0000000734410693*h1+ 0.0000000060014657)*udis^1.82800065)/(beta1*sqrt(1-((He-h1)/(He))^2));
%Qs=((0.0000000702626295*h1^3 - 0.000000123003337*h1^2 +...
%+0.0000000734410693*h1+ 0.0000000060014657)*udis^1.82800065)/(beta1*sqrt(1-((He-h1)/(He))^2))
%Fu1=(u-alpha1*sqrt(h1))/(beta1*sqrt(1-((He-h1)/(He))^2));
G=h1;
A11=double(subs(diff(Fu1,h1),{h1 u},{h1op uop}));
B11=double(subs(diff(Fu1,u),{h1 u},{h1op uop}));
C11=double(subs(diff(G,h1),{h1 u},{h1op uop}));
D11=double(subs(diff(G,u),{h1 u},{h1op uop}));
A=[A11];
B=[B11];
C=[C11];
D=[D11];
%% Ecuacion de estados linealizada 
syms h1d ud
htd=[h1d];
utd=[ud];
%Hprima=vpa((A*htd) + (B*utd),10);
%Ysalida=vpa((C*htd) + (D*utd),10);
Hprima=(A*htd) + (B*utd);
Ysalida=(C*htd) + (D*utd);
[n,d]=ss2tf(A,B,C,D);
mySys_tf=tf(n,d)
[C_pid_fast,info] = pidtune(mySys_tf,'PID')
[Kp,Ti,Td] = pidstddata(C_pid_fast)
yz=c2d(mySys_tf,Ts,'zoh')
[numdxx,dendxx]=tfdata(yz,'v');
[A1,B1,C1,D1]=tf2ss(numdxx,dendxx);
%step(feedback(C_pid_fast*mySys_tf,1))
%%

dt = 0.000001;
PopSize = 25;
MaxGenerations = 10;
s = tf('s');
%G = 1/(s*(s*s+s+1));
Gs=n(2)/(s + d(2));
options = optimoptions(@ga,'PopulationSize',PopSize,'MaxGenerations',MaxGenerations,'OutputFcn',@myfun);
%[x,fval] = ga(@(K)pidtest(Gs,dt,K),3,-eye(3),zeros(3,1),[],[],[],[],[],options);
[x,fval] = ga(@(K)pidtest(Gs,dt,K),2,-eye(2),zeros(2,1),[],[],[],[],[],options);
%% 
Kpag=x(1)
Kiag=x(2);
%Kdag=x(3);
Tix=Kpag/Kiag
%Tdx=Kdag/Kpag
%C_pid_fastx=pid(Kpag, Kiag, Kdag);
C_pid_fastx=pid(Kpag, Kiag);%Kp, ki, kd
T_pid_fastx = feedback(C_pid_fastx*mySys_tf,1);
%figure
%step(T_pid_fastx)
%axis([0 100 0 5])
