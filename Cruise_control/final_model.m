%%%%%%%%System Modeling%%%%%%%%%%%



%State-space model

m1 = 1624; %%vehicle mass(in kg) for 2019 BMW3 Series
b1 = 17.91;   %% damping coefficient(in Newton*sec/meters)
f1 = 480;  %% nominal control force(in Newton)(the exact price)
m2 = 1645; %%vehicle mass for 2018 Audi A4(in kg)
b2 = 17.98;   %% damping coefficient(in Newton*sec/meters)
f2 = 482;  %% nominal control force(in Newton)
r= 26.8;    %% reference speed(m/s),it's the same for our two models

A1 = -b1/m1;
B1 = 1/m1;
A2 = -b2/m2;
B2 = 1/m2;
C = 1;
D = 0;

cruise_model_ss1 = ss(A1,B1,C,D);%%creates a state-space model object representing the continuous-time state-space model
cruise_model_ss2 = ss(A2,B2,C,D);


%Transfer function model

s = tf('s');
P_transfer_cruise1 = 1/(m1*s+b1)%1/(m1*s+b1);
P_transfer_cruise2 = 1/(m2*s+b2)

%
%%%%%%%%System Analysis%%%%%%%%%%%


%The open-loop step response
open_step_response1 = f1*P_transfer_cruise1;
open_step_response2 = f2*P_transfer_cruise2;
figure(1)
%%compare the step-responses of the open-loop system and specify plot colors and styles for each response
step(open_step_response1,'g-')
legend("open-loop step 1")
figure(2)
step(open_step_response2,'r-')
legend("open-loop step2")


%{
%Open-loop poles/zeros

figure(3)
pzmap(P_transfer_cruise1)%%if we want to get the values p,z--->poles,zeros
axis([-1 1 -1 1])
legend("poles/zeros of transfer function1")
figure(4)
pzmap(P_transfer_cruise2)%%if we want to get the values p,z--->poles,zeros
axis([-1 1 -1 1])
legend("poles/zeros of transfer function2")

%Open-loop Bode plot
figure(5)
bode(P_transfer_cruise1)
legend("bode plot for first model")
figure(6)
bode(P_transfer_cruise2,'r')
legend("bode plot for second model")
%}
%%%%%%%%%%%%%PID Controller Design%%%%%%%%%%%%%%%%%

%
Kp1 = 9484.9;%rise time at 0.375 and 26.8m/s in 5.57 secs which is acceptable(for 5,6 secs) and peak response in 20 secs
Ki1 = 1;
Kd1 = 1;

Kp2 = 9520;%26.8 m/s in 7.09(in 7,1 secs) secs which is accepatble and rise time 0.378 secs and peak resposnse 20 secs
Ki2 = 1;
Kd2 = 1;

s = tf('s');
C_PID_1  = Kp1 + Ki1/s + Kd1*s
C_PID_2  = Kp2 + Ki2/s + Kd2*s
%%%%%%%Alternatively PID controller object%%%%%%%%%%

C_PID_altern1 = pid(Kp1,Ki1,Kd1)
C_PID_altern2 = pid(Kp2,Ki2,Kd2)
%%%%%%Proportional Control----->in order to get other values we can put in
%%%%%%in another .m file    9582
%}
%
Kp_proportional1 = 9582;%%26.8 m/s at 2.77 with rise time 0.37 and peak response in 6.2 secs
C_proportional1 = pid(Kp_proportional1);
Kp_proportional2 = 9520;%%peak response at 6.3 seconds 26.8 m/s and rise time 0.372 secs and 26.8m/s in 2.68 secs
C_proportional2 = pid(Kp_proportional2);

T1_proportional = feedback(C_proportional1*P_transfer_cruise1,1)
T2_proportional = feedback(C_proportional2*P_transfer_cruise2,1)
step1_proportional = r*T1_proportional
step2_proportional = r*T2_proportional


t = 0:0.1:20;

figure(7)
step(step1_proportional,t)
axis([0 20 0 26.8])%%we use the following axes for our desired reference velocity
legend("step1proportional")

figure(8)
step(step2_proportional,t)
axis([0 20 0 26.8])
legend("step2proportional")

%}
%
%%%%%%changing Kp to 5000
%%Kp = 5000;
Kp_proportional_new1 = 10550;%%peak response at 5.6 secs and rise time 0.341 secs 
Kp_proportional_new2 = 9650;%%peak response 26.8 m/s at 6.1 secs and rise time at 0.371 ssecs

C_proportional_new1 = pid(Kp_proportional_new1);
C_proportional_new2 = pid(Kp_proportional_new2);

T1_proportional_new = feedback(C_proportional_new1*P_transfer_cruise1,1)
T2_proportional_new = feedback(C_proportional_new2*P_transfer_cruise2,1)
step1_proportional_new = r*T1_proportional_new
step2_proportional_new = r*T2_proportional_new



figure(9)
step(step1_proportional_new,t)
axis([0 20 0 26.8])%%we use the following axes for our desired reference velocity
legend("step1proportional-new")

figure(10)
step(step2_proportional_new,t)
axis([0 20 0 26.8])
legend("step2proportional-new")

%}
%
%%%%%%PI Control

Kp_pi1 = 9484.9;%rise time at 0.375 secs and 5.57 secs for 26.8 m/s
Ki_pi1 = 10%11.3691;
C_pi1 = pid(Kp_pi1,Ki_pi1);

Kp_pi2 = 9520%1100%;%rise time at 0.377 secs and peak response at 20 secs and 26.8 m/s in 7.09 secs
Ki_pi2 = 20%12.0611%1;
C_pi2 = pid(Kp_pi2,Ki_pi2);

T1_pi = feedback(C_pi1*P_transfer_cruise1,1)
T2_pi = feedback(C_pi2*P_transfer_cruise2,1)
step1_pi = r*T1_pi
step2_pi = r*T2_pi


t = 0:0.1:20;

%%%%%otherwise we get at the same plot
figure(11)
step(step1_pi,t)
axis([0 20 0 26.8])
legend("step1-pi")

figure(12)
step(step2_pi,t)
axis([0 20 0 26.8])
legend("step2-pi")
%}




%
%%%%%changing Kp tp 700 and Ki to 7 to see the changes of response
Kp_pi_new1 = 9485;%rise time at 0.375 secs and peak response at 20 secs and 26.8 m/s in 5.47 secs(as we increase the value of Kp the speed 26.8m/s is reached faster) 
Ki_pi_new1 = 100;
C_pi_new1 = pid(Kp_pi_new1,Ki_pi_new1);

Kp_pi_new2 = 9521;%rise time at 0.377 secs and 26.8m /s in 6.09 secs
Ki_pi_new2 = 100;
C_pi_new2 = pid(Kp_pi_new2,Ki_pi_new2);

T1_pi_new = feedback(C_pi_new1*P_transfer_cruise1,1)
T2_pi_new = feedback(C_pi_new2*P_transfer_cruise2,1)
step1_pi_new = r*T1_pi_new
step2_pi_new = r*T2_pi_new



figure(13)
step(step1_pi_new,t)
axis([0 20 0 26.8])
legend("step1-pi-new")

figure(14)
step(step2_pi_new,t)
axis([0 20 0 26.8])
legend("step2-pi-new")
%}


%%%%%%PID Control
%
C_pid1 = pid(Kp1,Ki1,Kd1);
C_pid2 = pid(Kp2,Ki2,Kd2);


T1_pid = feedback(C_pid1*P_transfer_cruise1,1)
T2_pid = feedback(C_pid2*P_transfer_cruise2,1)

step1_pid = r*T1_pid
step2_pid = r*T2_pid


figure(15)
step(step1_pid,t)
axis([0 20 0 26.8])
legend("step1-pid")

figure(16)
step(step2_pid,t)
axis([0 20 0 26.8])
legend("step2-pid")


%}

%%this method is used for auto computation of gains
[C_prop1,info] = pidtune(P_transfer_cruise1,'P')
[C_prop2,info] = pidtune(P_transfer_cruise2,'P')

[C_PI1,info] = pidtune(P_transfer_cruise1,'PI')
[C_PI2,info] = pidtune(P_transfer_cruise2,'PI')
%%we increase the crossover frequency for results optimization
[C_PI1_fast,inf0] = pidtune(P_transfer_cruise1,'PI',1.0)
[C_PI2_fast,info] = pidtune(P_transfer_cruise2,'PI',1.0)

[C_PID1,info] = pidtune(P_transfer_cruise1,'PID')
[C_PID2,info] = pidtune(P_transfer_cruise2,'PID')

[C_PID1_fast,info] = pidtune(P_transfer_cruise1,'PID',1.0)
[C_PID2_fast,info] = pidtune(P_transfer_cruise2,'PID',1.0)


T_PI1_pre = feedback(C_PI1*P_transfer_cruise1, 1);
T_PI1 =r*T_PI1_pre
T_PI2_pre = feedback(C_PI2*P_transfer_cruise2, 1);
T_PI2 =r*T_PI2_pre

T_PI1_fast_pre = feedback(C_PI1_fast*P_transfer_cruise1,1);
T_PI2_fast_pre = feedback(C_PI2_fast*P_transfer_cruise2,1);
T_PI1_fast= r*T_PI1_fast_pre
T_PI2_fast= r*T_PI2_fast_pre

T_PID1_pre = feedback(C_PID1*P_transfer_cruise1,1);
T_PID2_pre = feedback(C_PID2*P_transfer_cruise2,1);
T_PID1= r*T_PID1_pre
T_PID2= r*T_PID2_pre


T_PID1_fast_pre = feedback(C_PID1_fast*P_transfer_cruise1,1);
T_PID2_fast_pre = feedback(C_PID2_fast*P_transfer_cruise2,1);
T_PID1_fast= r*T_PID1_fast_pre
T_PID2_fast= r*T_PID2_fast_pre

R=26.8; 
[y,t]=step(R*T_PI1_pre); 
sserror_pi1=abs(R-y(end)) 

[y,t]=step(R*T_PI2_pre); 
sserror_pi2=abs(R-y(end)) 


[y,t]=step(R*T_PI1_fast_pre); 
sserror_pi1_fast=abs(R-y(end)) 

[y,t]=step(R*T_PI2_fast_pre); 
sserror_pi2_fast=abs(R-y(end))

[y,t]=step(R*T_PID1_pre); 
sserror_PID1=abs(R-y(end)) 

[y,t]=step(R*T_PID2_pre); 
sserror_PID2=abs(R-y(end))


[y,t]=step(R*T_PID1_fast_pre); 
sserror_PID1_fast=abs(R-y(end)) 

[y,t]=step(R*T_PID2_fast_pre); 
sserror_PID2_fast=abs(R-y(end)) 


figure(17)
step(T_PI1,T_PI1_fast)
legend('PI1','PI1,fast')

figure(18)
step(T_PI2,T_PI2_fast)
legend('PI2','PI2,fast')

figure(19)
step(T_PI1_fast, T_PID1_fast);
legend('PI1,fast','PID1,fast');

figure(20)
step(T_PI2_fast, T_PID2_fast);
legend('PI2,fast','PID2,fast');

figure(21)
step(T_PID1, T_PID1_fast);
legend('PID1','PID1,fast')

figure(22)
step(T_PID2, T_PID2_fast);
legend('PID2','PID2,fast')





