function B  = Jacobian_marcelinoB(X,U)


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

m = 0.03; % masa ruedas 
R = 0.028; % radio ruedas (0.056)
M = 60; % masa robot (con o sin ruedas, revisar) (0.63)
%Hs = 0.26; % (0.315) HAY QUE MEDIR SIN RUEDA
Rm = 6.69;

Kt = 0.317;
% x=X(3);
%%
l=0.13;
%%
 
B21=(Kt*cos(X(1)))/(R*Rm*((-l)*(m + M) + l*m*cos(X(1))^2));
    
B22=(Kt*cos(X(1)))/(R*Rm*((-l)*(m + M) + l*m*cos(X(1))^2)) ;


B41=Kt/(R*Rm*(m + M - m*cos(X(1))^2));


B42=Kt/(R*Rm*(m + M - m*cos(X(1))^2));


B(1,:)=[0   0   ];
B(2,:)=[B21 B22 ];
B(3,:)=[0   0   ];
B(4,:)=[B41 B42 ];

end