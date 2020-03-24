%   h = fft4tf(G,N)
%   h is same freq. data as fft(u). h is freq. response of G(z).
%   N is length(u). u is periodic data.
%                                                   Kosaka Lab. 190526
%   Ex:
%        u=[ones(10,1);zeros(10,1)];
%        s=tf('s');  G = c2d(1/(s+1),1);
%        h=fft4tf(G,length(u));
%        yfhat=h.*fft(u);   yhat=ifft(yfhat);
%        y=lsim(G,u);
%        plot([y yhat y-yhat]), legend('y','yhat by fft4tf','y-yhat')
function h = fft4tf(G,N)
    
    [g,p] = bode(G,2*pi./(N./(0:N/2)'*G.ts));
    h = squeeze(g).*exp(1i*squeeze(p)/180*pi);
    h = [h;conj(flipud(h(2:end-1)))];
%    h = [h;conj(flipud(h(2:end)))];
