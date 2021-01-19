clear
phi_1=1;

t=3600;
phi=3600;

k_1=1e-9;
k_2=1e-9;
D_1=1e-13;
D_2=1e-13;

t_2=(D_2*k_1^2)/(D_1^2)*(t-phi);

k_star=(D_2*k_1)/(D_1*k_2);

% x=1e-6*[0:2];
syms x u

Dawson= @(t) exp(-t^2*int(exp(x^2),0,t));

C=...
    -2/pi*int(exp(-u^2*(phi_1+t_2))/(1+u^2)*(sin(u*x)/u+cos(u*x)),u,0,inf)...
    +2/pi*int(exp(-u^2*t_2)/(1+k_star^2*u^2)*(sin(u*x)/u+k_star*cos(u*x)),u,0,inf)...
    +2/pi*(1-k_star)*int(exp(-u^2*(phi_1+t_2))/((1+u^2)*(1+k_star^2*u^2))*(cos(u*x)-k_star*u*sin(u*x)),u,0,inf)...
    -2/pi*(1-k_star)*exp(phi_1)*erfc(sqrt(phi_1))*int(exp(-u^2*t_2)*u/((1+u^2)*(1+k_star^2*u^2))*(sin(u*x)+k_star*u*cos(u*x)),u,0,inf)...
    -4/pi^2*(1-k_star^2)*int(exp(-u^2*t_2)/((1+u^2)*(1+k_star^2*u^2))*(sin(u*x)+k_star*u*cos(u*x))*sqrt(pi)*Dawson(u*sqrt(phi_1)),u,0,inf);

eval(subs(C,x,0))