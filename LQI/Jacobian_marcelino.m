function J  = Jacobian_marcelino(X,u)
%%
%  X es el estado, de dimensiones 4x1

        % X(1): Angulo del robot con respecto a la vertical
        % X(2): Derivada del angulo del robot con respecto a la vertical
        % X(3): Angulo del motor con respecto a la referencia inercial
        % X(4): Derivada del angulo del motor con respecto a la referencia inercial

%  u es la accion de control, o el termino forzante. de dimensiones 2x1
        % U(1):  voltaje motor izquierdo
        % U(2):  voltaje motor derecho
%%
g = 9.81; % gravedad
m = 0.03; % masa ruedas 
R = 0.028; % radio ruedas (0.056)
M = 60; % masa robot (con o sin ruedas, revisar) (0.63)
%Hs = 0.26; % (0.315) HAY QUE MEDIR SIN RUEDA
Rm = 6.69;
Kb = 0.468;
Kt = 0.317;
% x=X(3);
%%
l=0.13;
vl=u(1);
vr=u(2);
%%
 
J21=(2*l*m*cos(X(1))*...
    sin(X(1))*((Kt*(vl + vr + Kb*(X(2) - X(4)/R))*cos(X(1)))/(R*Rm) -  g*(m + M)*sin(X(1)) + l*m*X(2)^2*cos(X(1))*sin(X(1))))...
        /((-l)*(m + M) + l*m*cos(X(1))^2)^2 + ((-g)*(m + M)*cos(X(1)) + ...
    l*m*X(2)^2*cos(X(1))^2 - (Kt*(vl + vr + Kb*(X(2) - X(4)/R))*sin(X(1)))/(R*Rm) - ...
    l*m*X(2)^2*sin(X(1))^2)/((-l)*(m + M) + l*m*cos(X(1))^2);
    
J22= ((Kb*Kt*cos(X(1)))/(R*Rm) + 2*l*m*X(2)*cos(X(1))*sin(X(1)))/((-l)*(m + M) + ...
   l*m*cos(X(1))^2);
J24= -((Kb*Kt*cos(X(1)))/(R^2*Rm*((-l)*(m + M) + l*m*cos(X(1))^2)));


J41=(l*m*(X(2)^2*cos(X(1)) - g*m*cos(X(1))^2 + g*m*sin(X(1))^2))/(m + M -... 
    m*cos(X(1))^2) - (2*m*cos(X(1))*sin(X(1))*((Kt*(vl + vr + Kb*(X(2) - X(4)/R)))/(R*Rm) + ...
      l*m*(X(2)^2*sin(X(1)) - g*m*cos(X(1))*sin(X(1)))))/(m + M - m*cos(X(1))^2)^2;


J42= ((Kb*Kt)/(R*Rm) + 2*l*m*X(2)*sin(X(1)))/(m + M - m*cos(X(1))^2);

J44=-((Kb*Kt)/(R^2*Rm*(m + M - m*cos(X(1))^2)));


J(1,:)=[0   1   0 0];
J(2,:)=[J21 J22 0 J24];
J(3,:)=[0   0   0 1];
J(4,:)=[J41 J42 0 J44];



end