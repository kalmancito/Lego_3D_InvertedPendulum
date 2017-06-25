% marcelino Lineal para LQR

X0=[pi,0,0,0];
U0=[0,0];
A=Jacobian_marcelino(X0,U0);
B=Jacobian_marcelinoB(X0,U0);
C=eye(4);
D=zeros(4,2);

manolito= ss(A,B,C,D);

Q = eye(4)*1e-4;
Q(1,1) = 100e0;
Q(3,3) = 10e0;
R = eye(2)*1e2;


K_lqr = lqr(A,B,Q,R)
% -K_lqr*X0'
CI = [pi+0.01;0;0;0];
disp('end')
P=[-0.2 -0.45 -0.5 -0.4];
K_pole=place(A,B,P)

radioRueda=0.056;

K_lqr(1,end-1)*radioRueda;
K_lqr(1,end)*radioRueda;

K_lqr

%%
Q = eye(8)*1e-1;
Q(1,1) = 1e2;
Q(2,2)= 1e1;
Q(3,3) = 1e2;
Q(5,5) = 1e1;
Q(7,7) = 1e2;
R = eye(2)*1e2;
N =ones(8,2)*1e1;
%N(1:4,1)=[10 10 1 10]';
%N(1:4,2)=[10 10 1 10]';
%N(5:8,1)=[-100 100 100 10]';
%N(5:8,2)=[-100 100 100 10]';
minreal(manolito)
eig([Q N;N' R] )
K_lqi=lqi(manolito,Q,R,N)
%%

A

B