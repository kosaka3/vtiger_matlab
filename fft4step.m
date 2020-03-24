% V-Tiger (Vertual-Time response based Iterative Gain Evaluation and Redesign)
% 	[yjw,w_ts] = fft4step(y)
%       y  : Step response like data. y(1) and y(end) must be stational.
%       yjw: amplitude = abs(yjw), angle = angle(yjw) [rad]
%           see also vtiger.m
%                  by Kosaka Lab., 190515-200322
%
%   Try by copy & paste the following:
%
%         u00=ones(100,1);
%         u00(1)=0; %<-- IMPORTANT!
%         s=tf('s');  G=c2d((-s^2+1)/(s+1)^2,1);    y00=lsim(G,u00);
%         K=c2d(-0.1+0.1/s,1);    r=u00;    yt=lsim(feedback(G*K,1),r);
%         yjw = fft4step(y00);  y = ifft(yjw);  y = y(1:length(y00));
%         plot([y y00 y-y00]), legend('y by fft4step','y00','e=y-y00')
%         err=norm(y-y00),
function [y0jw,w_ts] = fft4step(y00)
    % ステップ応答など定常値が得られる信号を、周期信号にする
    if size(y00)*[1;0]==1,
        y00=y00';
    end,%'
    if norm(y00 - y00(1)*ones(length(y00),1))==0,
        disp('Error in fft4step: Data in which all elements are the same cannot be cycled.');
        disp('      Set initial value different from others. Ex. u00(1)=0, r00(1)=0;')
        return;
    end
    n=min(20,round(length(y00)/10)+1);  % ノイズ対策で平均値を使用
    y0=[y00;-y00+sum(y00(end+1-n:end))/n+y00(1)];   % step response to cyclic

    % フーリエ変換(FFT)する
    y0jw=fft(y0);

    % Normalized freq. w_ts*ts [rad/s], ts: sampling time [s]
    N = length(y0jw);
	w_ts=2*pi./(N./(0:(N-1))');
