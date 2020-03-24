% vtiger_demo demonstrates V-Tiger.
    % Parameter setting of Plant G(s), sampling time ts[s], and so on.
    ts=0.01;    % sampling time [s]
    s=tf('s');  % complex variable of Laplace transform
    z=tf('z',ts);   % shift operator
    p=(1-1/z)/ts;   % differential operator based on backward Euler's rule
    Gs = 5/(0.01*s^2+0.2*s+10)*exp(-0.1*s);   % plant G(s) to be controlled
    G = c2d(Gs,ts); % G(z) is derived by discretizing G(s) with zero order holder

    % Design initial controller K0 using  The Ziegler-Nichols rule
    [Ku,Pm,Wu,Wcp] = margin(G);    % get Gaim margin Ku at Wu[rad/s]
    Tu = 1/(Wu/2/pi);   % When K=Ku, self-excited vibration with period Tu[s] will occur.
    kp0=0.6*Ku; ki0=kp0/(0.5*Tu);    kd0=kp0*0.125*Tu;  % ZN classical parameters
    K0 = kp0 + ki0/p + kd0*p;   % ZN PID controller K0(z)
    
    % Measuring one-shot data y00(t) and u00(t)
    u00=ones(300,1);    % input u00(t) is a step function
    u00(1)=0;   % initial value must be different from the other values <-- IMPORTANT!
    y00=lsim(G,u00);    % y00 is simulated
    r=u00;  % reference input to feedback system
    
    % Step 1) Make step responses to cyclic, and get frequency data.
    freq.y0jw = fft4step(y00);  % y0(j w) from y00(t)
    freq.u0jw = fft4step(u00);  % u0(j w) from u00(t)
    freq.r0jw = fft4step(r);    % r0(j w) from r(t)
    freq.p = fft4tf(p,length(u00)*2);   % p(j w) from differential operator p
    freq.r = r;   % r(t) is reference input to feedback system
    freq.wST=0.02;% Error band of settling time for cost function
    freq.OVr=2;   % Overshoot [%] for constraints
    freq.GMr=3;   % Gain margin [dB] for constraints:  Regulator 3-10dB, 20-inf deg
    freq.PMr=20;  % Phase margin [deg] for constraints:Servo    10-20dB, 40-60  deg
    
    % Step 2) Optimize PID gains by evaluating overshoot, settling time,
    %         and stability margins using virtual time response.
    [kp,ki,kd] = vtigerPID(freq,[kp0 ki0 kd0]);   % Get optimum PID gains. [kp0 ki0 kd0] is initial value for optimization
    K = kp + ki/p + kd*p;  % PID controller by V-Tiger
    
    disp('-----------------------------------------------------')
    % Verify the controller of V-Tiger and ZN by simulations
    [y,u] = freq2yu(freq,K); % Virtual time responses predicted by V-Tiger when K is used
    Gcl = feedback(ss(G*K),1);  % closed loop transfer function. feedback(a,b)=a/(1+a*b). ss(G) is ss(Gcl);  % State space representation from G
    yt=lsim(Gcl,r); % yt is true y(t) simulated using true plant model G(z)
    yZN=lsim(feedback(ss(G*K0),1),r);% y(t) by K0 (ZN) is simulated using true plant model G(z)
    figure(1),
    p1=plot([yt yZN y-yt y00]);
    hold on
    vtigerPID(freq,[kp ki kd],1);
    hold off; grid; xlabel('sample number k (0.01k [sec])')
    legend(p1,'y (V-Tiger)','y (ZN)','error of true/virtual y','y_{00}','Location','southeast');
    title('PID control result. V-Tiger is better than Ziegler-Nichols rule')
    disp('V-Tiger has completed controller design using y00 instead of the plant model.')
    disp(['Plant G(s) to be discretized with sampling time ts=' num2str(ts) '[s] is as follows:']), 
    [yZN,iZN]=max(yZN); text(iZN,yZN,'\leftarrow y (ZN)','Color','red','FontSize',14);
    [yvt,ivt]=max(yt); text(ivt,yvt+0.06,['y (V-Tiger)';'\downarrow '],'Color','blue','FontSize',14);
    text(length(y)*0.16,y00(end)*0.8,['\uparrow                                      '; ...
                                      'y_{00} (used by V-Tiger instead of model G(z))'],'Color','magenta','FontSize',14);
    text(length(y)*0.25,max(y)*0.58,[ ...
    'V-Tiger optimization is as follows:                       '; ...
    '  Cost function: Settling time (error band is \pm 3%)     '; ...
    '  Constraints: Overshoot<3%, Stability margins> 3dB, 20deg'])
    Gs, disp('');
    disp('Fig.1 shows step resonses. V-Tiger is better than ZN method.'),
    disp('press any key to type code of "vtiger_demo.m".'), 
    pause
    disp('-----------------------------------------------------')
    figure(2), margin(G*K);    text(10,-1000,'Open-loop by V-Tiger')
    dbtype vtiger_demo 1:37
    disp('-----------------------------------------------------')
