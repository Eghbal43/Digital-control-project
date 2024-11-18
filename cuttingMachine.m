close all;
%------------------------- DC_MOTOR ----------------------------%

% Power 30 Kw           
% Max speed 3000 RPM    
% Max Voltage 1400     

%Parameter

Ra = 1;        %Armature Resistance -> (ohm)
La = 0.001;    %Armature Inductance -> (H)
Ke = 0.66;                                         % -> (N*m/A) 
Kt = 0.66;     %Motor Torque Constant -> (Nm/A)
Km = 0.66;                                         % -> (V*s/rad)  
J  = 0.1;      %Load and Armature Inertia 
B  = 0.029;    %Motor Friction Coefficient

Ge = tf(1, [La Ra]);
Gm = tf(1, [J B]);
G  = zpk(feedback(Ge*Gm*Kt,Ke));
G1=zpk (Ge*Gm*Km);
pzmap (G,G1)



                                            Xf = 1000;
                                            Tf = 100;

%************************* DC_MOTOR ***************************%

%------------------------- Work area ----------------------------%

figure('Name','Wood sheet','NumberTitle','off')
rectangle('Position',[0 0 5 3],'EdgeColor','black','LineWidth',1)
xlabel('x-axis (m)');
ylabel('y-axis (m)');
xlim([-0.5 5.5]);
ylim([-0.5 3.5]);
hold on
grid on

%************************* Work area ***************************%

%-------------------- Travelling Time (TT) ---------------------%
%TT: Org -> P1
T1 = 2; 
T2 = 22;

%TT: P1 -> P2
T3 = 23;                  
T4 = 43;

%TT: P2 -> P3
T5 = 44; 
T6 = 64;

%TT: P3 -> P4
T7 = 65; 
T8 = 85;

%TT: P4 -> P1
T9  = 86; 
T10 = 96;

%TT: P1 -> P5 (Initial Point of the Circle)
T11 = 97;
T12 = 127;

%Travelling time for the circle
T13=128;
T14=158;

%TT: P5 -> Org
T15 = 159;
T16 = 189;



%******************* Travelling Time (TT) *********************%

%------------------ Stopping Laser 1 (SL1) --------------------%
%S: -> P1 (Primer Punto del cuadrado)
Sx1 = 1;
Sy1 = 1;
sigma = 0:0.01:1; % Adimensional Time

orgX = 0; % (Orgin) 
OrgY = 0;

lambda = poly5(sigma);
lambda_d = poly5d(sigma);
lambda_dd = poly5dd(sigma);

                        %figure;
                        %plot(sigma,lambda);
                        %figure;
                        %plot(sigma,lambda_d);
                        %figure;
                        %plot(sigma,lambda_dd);

t1 = T1+sigma*(T2-T1);
SL1X = orgX+lambda*(Sx1-orgX); 
SL1Y = OrgY+lambda*(Sy1-OrgY);

%p = plot(SL1X,SL1Y,'g');
%p(1).LineWidth = 2;

% Velocity 
SL1VX=(Sx1-orgX)/(T2-T1)*(lambda_d);
SL1VY=(Sy1-OrgY)/(T2-T1)*(lambda_d);

%****************** Stopping Laser 1 (SL1) ********************%


%------------------------- Square(S)---------------------------%

%S: -> P2
Sx2 = 2;
Sy2 = 1;

%S: -> P3
Sx3 = 2;
Sy3 = 2;

%S: -> P4
Sx4 = 1;
Sy4 = 2;


%First Segment of the Square (P1 -> P2)
t2 = T3+sigma*(T4-T3);
SX1 = Sx1+lambda*(Sx2-Sx1);
SY1 = Sy1+lambda*(Sy2-Sy1);


% Velocity 
SVX1 = (Sx2-Sx1)/(T4-T3)*(lambda_d);
SVY1 = (Sy2-Sy1)/(T4-T3)*(lambda_d);


%Second Segment of the Square (P2 -> P3)
t3 = T5+sigma*(T6-T5);
SX2 = Sx2+lambda*(Sx3-Sx2);
SY2 = Sy2+lambda*(Sy3-Sy2);
%p = plot(SX2,SY2,'k');
%p(1).LineWidth = 2;

% Velocity 
SVX2 = (Sx3-Sx2)/(T6-T5)*(lambda_d);
SVY2 = (Sy3-Sy2)/(T6-T5)*(lambda_d);


%Third Segment of the Square (P3 -> P4)
t4 = T7+sigma*(T8-T7);
SX3 = Sx3+lambda*(Sx4-Sx3);
SY3 = Sy3+lambda*(Sy4-Sy3);
%p = plot(SX3,SY3,'k');
%p(1).LineWidth = 2;

% Velocity 
SVX3 = (Sx4-Sx3)/(T8-T7)*(lambda_d);
SVY3 = (Sy4-Sy3)/(T8-T7)*(lambda_d);


%Fourth Segment of the Square (P4 -> P3)
t5 = T9+sigma*(T10-T9);
SX4 = Sx4+lambda*(Sx1-Sx4);
SY4 = Sy4+lambda*(Sy1-Sy4);
%p = plot(SX4,SY4,'k');
%p(1).LineWidth = 2;

% Velocity 
SVX4 = (Sx1-Sx4)/(T10-T9)*(lambda_d);
SVY4 = (Sy1-Sy4)/(T10-T9)*(lambda_d);



%************************* Square(S) **************************%

%------------------ Stopping Laser 2 (SL2) --------------------%
% Center of the circle
CCx = 3;
CCy = 1.5;

%Point of contact with the circle
cContactX = 2.5;
cContactY = 1.5;

t6 = T11+sigma*(T12-T11);

SL2X = Sx1+lambda*(cContactX-Sx1); 
SL2Y = Sy1+lambda*(cContactY-Sy1);

%p = plot(SL2X,SL2Y,'g');
%p(1).LineWidth = 2;

% Velocity 

SL2VX=(cContactX-Sx1)/(T12-T11)*(lambda_d);
SL2VY=(cContactY-Sy1)/(T12-T11)*(lambda_d);


%****************** Stopping Laser 2 (SL2) ********************%



%------------------------- Circle(C)---------------------------%

t7 = T13:0.1:T14; % para trabajar con sigmaC

sigmaC = (t7-T13)/(T14-T13);
lambdaC = poly5(sigmaC);
lambdaC_d = poly5d(sigmaC);
lambdaC_dd = poly5dd(sigmaC);

%circle radius
circleR = 0.5;

Theta = 2*pi*lambdaC;
P = -circleR*[cos(Theta);sin(Theta)];
CX5 = CCx+P(1,:);
CY5 = CCy+P(2,:);

%p = plot(CX5,CY5,'k');
%p(1).LineWidth = 2;


%Velocity
Theta_d = (2*pi/(T14-T13))*lambdaC_d;

AV = -circleR*[-sin(Theta);cos(Theta)].*Theta_d; % velocidad del angulo
CVX5 = AV(1,:);
CVY5 = AV(2,:);


%************************* Circle(C) **************************%




%------------------ Stopping Laser 3 (SL3) --------------------%

t8 = T15+sigma*(T16-T15);

SL3X = cContactX+lambda*(orgX-cContactX); 
SL3Y = cContactY+lambda*(OrgY-cContactY);

%p = plot(SL3X,SL3Y,'g');
%p(1).LineWidth = 2;

% Velocity 

SL3VX=(orgX-cContactX)/(T16-T15)*(lambda_d);
SL3VY=(OrgY-cContactY)/(T16-T15)*(lambda_d);


%****************** Stopping Laser 3 (SL3) ********************%



%plotting SL 1 
for i=1:length(SL1X)
   pause(0.02)
    plot(SL1X(i),SL1Y(i),'g.');
end

%plotting Square
squareX=[SX1,SX2,SX3,SX4];
squareY=[SY1,SY2,SY3,SY4];

for i=1:length(squareX)
    pause(0.02)
    plot(squareX(i),squareY(i),'k.');
end

%plotting SL 2
for i=1:length(SL2X)
    pause(0.02)
    plot(SL2X(i),SL2Y(i),'g.');
end

%plotting Circle

for i=1:length(CX5)
    pause(0.02)
    plot(CX5(i),CY5(i),'k.');
end


%plotting SL 3
for i=1:length(SL3X)
   pause(0.02)
   plot(SL3X(i),SL3Y(i),'g.');
end

PX=([SL1X,SX1,SX2,SX3,SX4,SL2X,CX5,SL3X]);
PY=[SL1Y,SY1,SY2,SY3,SY4,SL2Y,CY5,SL3Y];
Time=([t1,t2,t3,t4,t5,t6,t7,t8]);

figure;

ax = subplot(3,1,1); 
plot(ax,sigma,lambda)
title(ax,'Position')
grid

ax1 = subplot(3,1,2); 
plot(ax1,sigma,lambda_d)
title(ax1,'Speed')
grid

ax2 = subplot(3,1,3); 
plot(ax2,sigma,lambda_dd)
title(ax2,'Acceleration')
grid


%Time series represent the time-evolution of a dynamic population or process
Px = timeseries(PX*4000*pi);  
Py = timeseries(PY*4000*pi);

figure;
plot(PX,PY);
grid

dX = diff(PX*4000*pi ); 
dY = diff(PY *4000*pi);

Vx = timeseries(dX);
Vy = timeseries(dY);

Ax = timeseries(diff(dX));
Ay = timeseries(diff(dY));



cutting_Machine

