clear all; close all; clc
syms h1 h2 u
He=0.52;
beta1=0.15;
yref=0.2;
h1op=yref;
%udis=100;
Ts=1;
%uop=(0.0000000702626295*h1op^3 - 0.000000123003337*h1op^2 + ...
%0.0000000734410693*h1op + 0.00000000600146569)*udis^1.82800065;
% Fu1=(u-(0.0000000702626295*h1^3 - 0.000000123003337*h1^2 +...
% +0.0000000734410693*h1+ 0.00000000600146569)*udis^1.82800065)/(beta1*sqrt(1-((He-h1)/(He))^2));
G=h1;
Ax=[];
Bx=[];
Cx=[];
Dx=[];
h1opx=[0.2 0.4 0.2 0.4];% 0.4 0.2 0.4 0.2 0.4];
udisx=[100 100 50 50];% 75 75 25 25];
uopx=[];
for k=1:4
    Fu1=(u-(0.0000000702626295*h1^3 - 0.000000123003337*h1^2 +...
+0.0000000734410693*h1+ 0.00000000600146569)*udisx(k)^1.82800065)/(beta1*sqrt(1-((He-h1)/(He))^2));
    uopx(k)=(0.0000000702626295*h1opx(k).^3 - 0.000000123003337*h1opx(k).^2 + ...
    0.0000000734410693*h1opx(k) + 0.0000000060014657)*udisx(k).^1.82800065;
    Ax(k)=double(subs(diff(Fu1,h1),{h1 u},{h1opx(k) uopx(k)}));
    Bx(k)=double(subs(diff(Fu1,u),{h1 u},{h1opx(k) uopx(k)}));
    Cx(k)=double(subs(diff(G,h1),{h1 u},{h1opx(k) uopx(k)}));
    Dx(k)=double(subs(diff(G,u),{h1 u},{h1opx(k) uopx(k)}));
end
%% Ecuacion de estados linealizada 
% syms h1d ud
% htd=[h1d];
% utd=[ud];
% Hprima=(A*htd) + (B*utd);
% Ysalida=(C*htd) + (D*utd);
% [n,d]=ss2tf(A,B,C,D);
% mySys_tf=tf(n,d);
mySystfx={};
Kpx=[];
Tix=[];
Tdx=[];
Al=[];
Bl=[];
Cl=[];
Dl=[];
for k=1:4
    sys_lti(:,:,k) = ss(Ax(k),Bx(k),Cx(k),Dx(k));   % Gs
    sys_pid(:,:,k) = pidtune(sys_lti(:,:,k),'PID');  % Gpid
    [Kpx(k),Tix(k),Tdx(k)]=pidstddata(sys_pid(:,:,k));
    Gspid(:,:,k)=sys_lti(:,:,k)*sys_pid(:,:,k); %Gspid
    %[numdxx,dendxx]=tfdata(yz,'v');
    %[A1,B1,C1,D1]=tf2ss(numdxx,dendxx);
    sys_loop(:,:,k)=feedback(sys_lti(:,:,k)*sys_pid(:,:,k),1);  %TF LAZO CEERRADO
end
%%
% res={};
% Gsxx={'Gs1x','Gs2x','Gs3x','Gs4x'};
% Gpidxx={'Gpid1','Gpid2','Gpid3','Gpid4'};

% for k=1:1
%     Fz2=c2d(sys_loop(:,:,k),Ts,'zoh');
%     [numz2,denz2]=tfdata(Fz2,'v');
%     sdz2(k, :)=denz2;
%     tf(numz2,denz2,Ts)
%     vpa(sdz2,9)
%     juryC(denz2)
% end

for k=3:3
    %sys_lti(:,:,k); %Gs
    [numgs,dengs]=tfdata(sys_lti(:,:,k),'v');
    G_s=tf(numgs,dengs)
    %sys_pid(:,:,k); %Gpid
    [numpi,denpi]=tfdata(sys_pid(:,:,k),'v');
    G_pid=tf(numpi,denpi)
    [numgspi,dengspi]=tfdata(Gspid(:,:,k),'v');
    G_spid=tf(numgspi,dengspi)
    sysz=c2d(Gspid(:,:,k),Ts,'zoh');
    [numzGspid,denzGspid]=tfdata(sysz,'v');
    G_zpid=tf(numzGspid,denzGspid,Ts)
    Fz1=feedback(sysz,1);
    %[numdz(k),dendz(k)]=tfdata(sys_loop(:,:,k),'v')
    %[numk,denk]=tfdata(sys_loop(:,:,k),'v');
    [numz,denz]=tfdata(Fz1,'v');
    %sd(k, :)=denk;
    sdz(k, :)=denz;
   % field = strcat( 'p',num2str(k) );
    %polcar.(field)=tf(numk,denk);
    %tf(numk,denk)
    G_loopz=tf(numz,denz,Ts)
    Polcar_z=vpa(sdz,9)
    juryC(denz)
end
%vpa(sd,9)