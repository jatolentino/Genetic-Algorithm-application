%clear all; close all; clc
syms h1 h2 u
He=0.52;
beta1=0.15;
yref=0.2;
h1op=yref;
udis=100;
Ts=0.1;
%uop=(0.0000000702626295*h1op^3 - 0.000000123003337*h1op^2 + ...
%0.0000000734410693*h1op + 0.0000000060014657)*udis^1.82800065;
%Fu1=(u-(0.0000000702626295*h1^3 - 0.000000123003337*h1^2 +...
%+0.0000000734410693*h1+ 0.0000000060014657)*udis^1.82800065)/(beta1*sqrt(1-((He-h1)/(He))^2));
G=h1;
Ax=[];
Bx=[];
Cx=[];
Dx=[];
h1opx=[0.2 0.4 0.2 0.4];
udisx=[100 100 50 50];
uopx=[];
for k=1:4
    Fu1=(u-(0.0000000702626295*h1^3 - 0.000000123003337*h1^2 +...
+0.0000000734410693*h1+ 0.0000000060014657)*udisx(k)^1.82800065)/(beta1*sqrt(1-((He-h1)/(He))^2));
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
    sys_lti(:,:,k) = ss(Ax(k),Bx(k),Cx(k),Dx(k));
    sys_pid(:,:,k) = pidtune(sys_lti(:,:,k),'PID');
    [Kpx(k),Tix(k),Tdx(k)]=pidstddata(sys_pid(:,:,k));
    sys_loop(:,:,k)=feedback(sys_lti(:,:,k)*sys_pid(:,:,k),1);  %TF LAZO CEERRADO
%     [Al(k) Bl(k) Cl(k) Dl(k)]=ssdata(sys_loop(:,:,k))
%     A=ss(A,B,C,D)
end
%[C_pid_fast,info] = pidtune(mySys_tf,'PID')
%[Kp,Ti,Td] = pidstddata(C_pid_fast)
%yz=c2d(mySys_tf,Ts,'zoh')
%[numdxx,dendxx]=tfdata(yz,'v');
%[A1,B1,C1,D1]=tf2ss(numdxx,dendxx);
%%
%step(feedback(C_pid_fast*mySys_tf,1))
syms fig1 fig2 fig3 fig4
figuras={fig1, fig2, fig3, fig4};
kpti={};
tit={};
%h=[];
figure() ;
for k=1:4
    %figure
    %subplot(2,2,k);
    subplot(2,2,k);
    %kpti(k)=sprintf('Kp = %.5f, Ti = %.5f,Td = %.5f', Kpx(1),Tix(1));
    figur=stepplot(h1opx(k)*sys_loop(:,:,k,1));
     pfig = getoptions(figur);
     pfig.TimeUnits = 'segundos'; 
     pfig.YLabel.String = 'h_{1}(m)';
     pfig.XLabel.String = 'Tiempo';
     setoptions(figur,pfig);
%      rect = [0.25, 0.25, .25, .25];
%      set(h, 'Position', rect)
%     setoptions(figur,pfig)
    %hgsave(h,sprintf('fig%d',k))
    %hold on
    grid on
    %opt.Normalize = 'on'
    %sprintf('Setpoint: %f cm con valvula de disturbio al %f%',h1opx(k),udisx(k));
    %title(tit(k));
    title(sprintf('Setpoint: %.1fm y válvula de disturbio al %d%%',h1opx(k),udisx(k)));
    %legend(kpti(k));
    %lg=sprintf('Kp = %.4e, Ti = %.2f', Kpx(k),Tix(k))
    leg=legend(sprintf('Kp = %.4e, Ti = %.2f', Kpx(k),Tix(k)));
    switch k
    case 1
       newPosition = [0.36 0.6 0.04 0.04];
    case 2
       newPosition = [0.8 0.6 0.04 0.04];
    case 3
       newPosition = [0.36 0.125 0.04 0.04];
    otherwise
       newPosition = [0.8 0.125 0.04 0.04];
    end
    
    newUnits = 'normalized';
    set(leg,'Position', newPosition,'Units', newUnits);
    set(gca,'FontName','Times New Roman','FontSize',12);
    %set(allaxes,'FontName','Times New Roman','FontWeight','Bold','LineWidth',2,'FontSize',12)
    set(findall(gcf,'type','text'),'FontName','Times New Roman','FontSize',12);%'fontWeight','bold')
    %set(findall(gcf,'-property','FontSize'),'FontSize',12)
    %set(findall(gcf,'-property','FontName'),'Times New Roman')
    
%     rect = [0.25, 0.25, .25, .25];
%     set(lgg, 'Position', rect)
end
%%
% for k=1:4
%     Polz=sys_loop(:,:,k) 
%     [numdz,dendz]=tfdata(Polz,'v')
%     juryC(dendz)
% end

