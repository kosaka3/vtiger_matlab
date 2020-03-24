% Get optimum PID gains using V-Tiger.
%   [kp,ki,kd] = vtigerPID(freq,[kp0 ki0 kd0]);
%       freq: y0(j w), u0(j w), r0(j w), d0(j w), du0(j w), p(j w)
%       [kp0,ki0,kd0]: initial PID gains for optimization
%       [kp,ki,kd]: optimum PID gains using V-Tiger
%		see also:
%			vtiger.m
%					by Kosaka Lab., 190531-200323
function	    [kp,ki,kd] = vtigerPID(freq,th0,f)
	global gl       % global valuable used in this m-file
    gl= freq;       % y0(j w), u0(j w), r0(j w), d0(j w), du0(j w), p(j w)

    % Calculate G(j w)
    G= gl.y0jw./gl.u0jw;
    N = length(gl.u0jw);
    ts=1;   % ts is dummy sampling time, but ts never affect anything.
	gl.w= 2*pi./(N./(0:(N-1))'*ts);%'
    del=sum(abs(gl.u0jw))/length(gl.u0jw)*1e-10;  % countermeasure for numerical error
    G(abs(gl.u0jw)<=del)=[];
    gl.w(abs(gl.u0jw)<=del)=[];
    gl.p2=gl.p;
    gl.p2(abs(gl.u0jw)<=del)=[];
    gl.Gjw= G;  % G(j w): frequency response of the plant
    
    if nargin>2
        J_cost(th0,f),   % for debug
        return;
    end

    % PID gains are optimized evaluating overshoot, settling time, and stability margins.
    [th, fval, exitflag, output] = fminsearch(@J_cost2,th0);
%    [th, fval, exitflag, output] = fmincon(@J_cost,th0,[],[],[],[],[1 1 1]'*(eps),[1 1 1]'*inf,@constraints);
%    [th, fval, exitflag, output] = ga(@J_cost,length(th0),[],[],[],[],[1 1 1]'*(eps),th0*5,@constraints);
%    [th, fval, exitflag, output] = pso(@J_cost,length(th0),[],[],[],[],[1 1 1]'*(eps),th0*2,@constraints);
    kp= th(1);
    ki= th(2);
    kd= th(3);    
end


%;////////////////////////////////////////////////////////////////////
%;// Cost function with constraints by penalty for fminsearch
%;////////////////////////////////////////////////////////////////////
function J = J_cost2(th)
    J = J_cost(th) + 1e10*max([0,constraints(th)]);
end

%;////////////////////////////////////////////////////////////////////
%;// Cost function for optimization
%;////////////////////////////////////////////////////////////////////
function J = J_cost(th,f)
	global gl       % global valuable used in this m-file

  if max(isnan(th))==1
    J=1e99;
  else
	Kp=th(1);	Ki=th(2);	Kd=th(3); % Ki/Kp > 10*Kp/Kd is desired
	K=Kp + Ki./gl.p + Kd*gl.p;	% Set s=(1-1/z)/ts in the PID controller K(s)=Kp+Ki/s+Kd*s

    %%%%%  Virtual time responses y(t) and u(t) is predicted when K(th) is in loop.
    [y,u] = freq2yu(gl,th); % Virtual time responses predicted by V-Tiger when K is used
    k=((1:length(y))-1);

    %%%%%  Measure settling time
    y(y<0)=0;   % counter measure for bug of stepinfo when undershoot occurs.
    si=stepinfo(y(1:end-2),k(1:end-2),sum(gl.r(end+1-20:end))/20,'SettlingTimeThreshold',gl.wST); % wST: Error band of settling time
    J = si.SettlingTime;%   if isnan(ey)==1, ey=inf;end

    % for debug
    if nargin>1,
        plot([ones(length(y),1)*sum(gl.r(end+1-20:end))/20*[1+gl.wST 1-gl.wST 1+gl.OVr/100]],'k-.'),
    %    plot(k,[y ones(length(y),1)*sum(gl.r(end+1-20:end))/20*[1+gl.wST 1-gl.wST 1+gl.OVr/100]]),
    %    constraints(th),si,
        return;
    end
  end
    if isnan(J),    J=1e99;end,% pso does not accept NaN
    disp(sprintf('J= %g',J)),   % display J: cost function
end


%;////////////////////////////////////////////////////////////////////
%;// Constraints for optimization
%;////////////////////////////////////////////////////////////////////
function [c,ceq] = constraints(th)   % c<0, ceq=0, if no constraints, then c=[] or ceq=[]
	global gl  % global valuable used in this m-file
    p=gl.p;    % 

if max(isnan(th))==1
    c=1e99*[1 1 1 1];
else
	Kp=th(1);	Ki=th(2);	Kd=th(3); % Ki/Kp > 10*Kp/Kd is desired
	K=Kp + Ki./gl.p + Kd*gl.p;	% Set s=(1-1/z)/ts in the PID controller K(s)=Kp+Ki/s+Kd*s

    c=[];	% 	c is a vector of inequality constraints.
    Kjw=Kp+Ki./gl.p2+Kd*gl.p2;
    GKjw= gl.Gjw .* Kjw;
    GKjw= GKjw(1:round(end/2));    % ignore over Nyquist freq.
    [Gm,Pm,Wcg,Wcp] = margin(abs(GKjw),angle(GKjw)/pi*180,gl.w((1:round(end/2))));
    c = [c, gl.GMr-20*log10(Gm)];   % constraint: Gain margin
    c = [c, gl.PMr-Pm];    			% constraint: Phase margin
  if isnan([Kd+Kp+Ki])
    c = [c, inf, inf];
  else
    c = [c, max(real(roots([Kd Kp Ki])))+0.001]; % constraint: 1/K is stable and Re[pole]<-0.001
%    c = [c, -Kp, -Ki, -Kd];        % constraints: Kp>0, Ki>0, Kd>0 (same as 1/K to be stable)

    %%%%%  Virtual time responses y(t) and u(t) is predicted when K(th) is in loop.
    [y,u] = freq2yu(gl,th); % Virtual time responses predicted by V-Tiger when K is used
    k=((1:length(y))-1);

    si=stepinfo(y,k,sum(gl.r(end+1-20:end))/20);
    c = [c, si.Overshoot-gl.OVr];  % constraint: overshoot < OVr
  end
end
%[20*log10(Gm) Pm max(real(roots([Kd Kp Ki]))) si.Overshoot], gl.PMr,c
%margin(abs(GKjw),angle(GKjw)/pi*180,gl.w);pause
    ceq = [];   % 	Equality constraint is not exist.
end
