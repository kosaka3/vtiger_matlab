% 	[y,u,r2] = freq2yu(freq,K,ts)
%       freq.y0jw: Frequency response of cyclic output data of the plant
%       freq.u0jw: Frequency response of cyclic input data of the plant
%       freq.r0jw: Frequency response of cyclic reference signal into the feedback system with K
%       freq.d0jw: Frequency response of cyclic output disturbance into the feedback system with K
%       freq.du0jw: Frequency response of cyclic input disturbance into the feedback system with K
%       K: Controller to be evaluated. ex. K=1-0.1/z, z=tf('z',ts)
%          or PID gains. ex. K=[kp ki kd]
%       ts: Sampling time [s]
%       y, u: Predicted time responses of the closed loop in which K is inserted.
%       r2: r(t) in which set rjw=0 when u0jw=0.
%           see also vtiger_demo.m
%                  by Kosaka Lab., 190515-200322
%
%   Try by copy & paste the following:
%
%         u00=ones(100,1);
%         u00(1)=0; %<-- IMPORTANT!
%         s=tf('s');  G=c2d((-s^2+1)/(s+1)^2,1);    y00=lsim(G,u00);
%         K=c2d(-0.1+0.1/s,1);    r=u00;    yt=lsim(feedback(G*K,1),r);
%         freq.y0jw = fft4step(y00);  freq.u0jw = fft4step(u00);
%         freq.r0jw = fft4step(r);
%         [y,u,r2] = freq2yu(freq,K);
%         plot([y yt y-yt]), legend('y by vtiger','yt=Gcl r','e=y-yhat')
%         err=[norm(yt-y), norm(r2-r)],
function [y,u,r2] = freq2yu(freq,K,ts)
    if nargin==3,
        K = K+0*tf('z',ts); % set K.ts used in fft4tf
    end
    if isnumeric(K)
        Kjw = K(1) + K(2)./freq.p + K(3)*freq.p;  % K is PID gains
        invKjw = 1./Kjw;
%        disp('Error: Set sampling time ts in freq2yu(freq,K,ts).');
%        return;
    else
        invKjw = fft4tf(1/K,length(freq.u0jw));
    end
    y0jw = freq.y0jw;
    u0jw = freq.u0jw;
    if isfield(freq,'r0jw')==1
        r0jw = freq.r0jw;
    else
        r0jw = y0jw*0;
    end
    if isfield(freq,'d0jw')==1
        d0jw = freq.d0jw;
    else
        d0jw = y0jw*0;
    end
    if isfield(freq,'du0jw')==1
        du0jw = freq.du0jw;
    else
        du0jw = u0jw*0;
    end
    
    % r1(jw)を計算
    r1jw = invKjw.*u0jw + y0jw;
    
    % Calculate virtual frequency responses of the feedback system with K 
    yjw= (r0jw.*y0jw + invKjw.*(d0jw.*u0jw + du0jw.*y0jw))./r1jw;
    ujw= (r0jw-yjw)./invKjw;
    
    % Set zero when r1jw=0
    yjw(r1jw==0)=0;
    ujw(r1jw==0)=0;
    ujw(invKjw==0)=0;

    % 逆フーリエ変換(IFFT)してy,uを求める。長さをy00と同じにする
    % Calculate virtual time responses y(t) and u(t) using IFFT
    y = ifft(yjw);    y = y(1:end/2);
    u = ifft(ujw);    u = u(1:end/2);
    r2= ifft(r0jw);   r2= r2(1:end/2);

    if norm(imag(y))~=0
        tmp=norm(imag(y))/norm(real(y));
        if tmp>1e-5,
            disp(['warning in vtiger: yに虚部(im/re>1e-5)が存在します。虚部÷実部= ',num2str(tmp)]);
        end
        y=real(y);  u=real(u);  r2=real(r2);
    end

    r00 = ifft(r0jw);   r00=r00(1:end/2);
    if norm(r00-r2),
        disp('err at vtiger.m: u0が含まない周波数成分をrが含むため、rのその周波数成分を除去しました')
    end
