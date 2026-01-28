syms sp Lb d2 cp rw La
T03 = desp([sp*(Lb+d2),-cp*(Lb+d2),0]);
T03 = vpa(T03,2);
TWO = desp([0 0 rw])*rotx(-pi/2);
TWO = vpa(TWO,2);
T3TCP = rotx(pi/2)*desp([La 0 0]);
T3TCP = vpa(T3TCP,2);
T_total =TWO * T03 * T3TCP;
T_total = vpa(T_total,2)


syms theta1 theta3 Lb d2

T1 = dh_trans(0,0,0,theta1)
T2 = dh_trans(0,pi/2,Lb+d2,0)
T3 = dh_trans(0,-pi/2,0,theta3)

T = TWO*T1*T2*T3*T3TCP;
 vpa(T(3,4),2)