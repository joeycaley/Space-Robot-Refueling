function dx = dynFunc(x,u)

m1 = 1.0;% kg
m2 = 1.0;% kg
I1 = 0.;
I2 = 0.;
l1 = 0.5;
l2 = 0.5;
r1 = l1/2;
r2 = l2/2;
g = 9.81;%


q1 = x(1);
q2 = x(2);
dq1 = x(3);
dq2 = x(4);

a = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
b = m2*l1*r2;
c = I2 + m2*r2^2;

M = [a + 2*b*cos(q2), c + b*cos(q2);
    c + b*cos(q2), c];

C = [-b*sin(q2)*dq1*dq2 - b*sin(q2)*(dq1+dq2)*dq2;
    b*sin(q2)*dq1^2];

G = [m1*r1*g*cos(q1) + m2*g*(l1*cos(q1) + r2*cos(q1+q2));
    m2*g*r2*cos(q1+q2)];




U = [0;u];

ddq = M\(U - C - G);

dx = [dq1;dq2;ddq];

end

