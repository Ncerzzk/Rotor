syms Fz F1 F2 theta1 theta2 Mx My Mz;
syms r d;
Q1=F1*cos(theta1)+F2*cos(theta2)-Fz;
Q2=F1*cos(theta1)*d-F2*cos(theta2)*d-Mx;
Q3=F1*sin(theta1)*d-F2*sin(theta2)*d-Mz;
Q4=F1*r*sin(theta1)+F2*r*sin(theta2)-My;

[a,b,c,d]=solve(Q1,Q2,Q3,Q4,F1,F2,theta1,theta2);

result1=[a(1);b(1);c(1);d(1)];
result2=[a(3);b(3);c(3);d(3)];
pretty(simplify(result2))