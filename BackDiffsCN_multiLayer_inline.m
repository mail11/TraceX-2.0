function [X,pro,DepthFlag]=BackDiffsCN_multiLayer_inline(C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProfileLength,PixelNo)
DepthFlag=0;
dx=ProfileLength*1e-6/(PixelNo-1); %spacial step
X=0:dx:ProfileLength*1e-6; %domain
% dx=X(2)-X(1);
t_i=t1*3600; %s %initial exchange duration
t_b=t2*3600; %s %back exchange duration
t_tot=t_i+t_b; %s %total time
h1=k1/D1;
h2=k2/D2;
C_gas1=C_gas; % gas concentration for period 1
C_gas2=C_bg; % gas concentration for period 2

gamma=ones(1,length(X)+1);
gamma(100)=0.05; %interfacial
gamma(101:end)=.5; % second material

dt=round(dx^2./(2*max(D1*gamma)),3,'significant'); %time step
% dt=round(dx^2/(2*max(D1,D2)),3,'significant');
Nt=round((t_tot)/dt); % Number of time steps
I=diag(ones(1,PixelNo));

sigma1=D1*dt/(2*dx^2);
sigma2=D2*dt/(2*dx^2);

%%% CN Diffusion matrix
% A1=    full(gallery('tridiag',PixelNo, sigma1,1-2*sigma1, sigma1));
% A2=    full(gallery('tridiag',PixelNo, sigma2,1-2*sigma2, sigma2));
% A1_new=full(gallery('tridiag',PixelNo,-sigma1,1+2*sigma1,-sigma1));
% A2_new=full(gallery('tridiag',PixelNo,-sigma2,1+2*sigma2,-sigma2));

A1=    zeros(length(X));
A2=    zeros(length(X));
A1_new=zeros(length(X));
A2_new=zeros(length(X));
for i=1:length(X)
    if i>1
        A1(i,i-1)=    sigma1*gamma(i);
        A2(i,i-1)=    sigma2*gamma(i);
        A1_new(i,i-1)=-sigma1*gamma(i);
        A2_new(i,i-1)=-sigma2*gamma(i);
    end
    
    A1(i,i)=    1-sigma1*(gamma(i)+gamma(i+1));
    A2(i,i)=    1-sigma2*(gamma(i)+gamma(i+1));
    A1_new(i,i)=1+sigma1*(gamma(i)+gamma(i+1));
    A2_new(i,i)=1+sigma2*(gamma(i)+gamma(i+1));
    
    if i<length(X)
        A1(i,i+1)=    sigma1*gamma(i+1);
        A2(i,i+1)=    sigma2*gamma(i+1);
        A1_new(i,i+1)=-sigma1*gamma(i+1);
        A2_new(i,i+1)=-sigma2*gamma(i+1);
    end
end
% keyboard
%%% Exchange surface condition
A1(1,1:2)=    [1-2*dx*h1*sigma1-2*sigma1, 2*sigma1];
A2(1,1:2)=    [1-2*dx*h2*sigma2-2*sigma2, 2*sigma2];
A1_new(1,1:2)=[1+2*dx*h1*sigma1+2*sigma1,-2*sigma1];
A2_new(1,1:2)=[1+2*dx*h2*sigma2+2*sigma2,-2*sigma2];
beta1=-h1*C_gas1;
beta2=-h2*C_gas2;
G1=zeros(PixelNo,1); G1(1)=-4*dx*beta1*sigma1;
G2=zeros(PixelNo,1); G2(1)=-4*dx*beta2*sigma2;
%%% Mirror oundary condition
A1(end,end-2:end)=    [sigma1*gamma(end) -2*sigma1*gamma(end) 1+sigma1*gamma(end)]; %same second der
A2(end,end-2:end)=    [sigma2*gamma(end) -2*sigma2*gamma(end) 1+sigma2*gamma(end)];
A1_new(end,end-2:end)=[-sigma1*gamma(end) 2*sigma1*gamma(end) 1-sigma1*gamma(end)];
A2_new(end,end-2:end)=[-sigma2*gamma(end) 2*sigma2*gamma(end) 1-sigma2*gamma(end)];
A1_newI=A1_new\I;
A2_newI=A2_new\I;
% Time stepping
% C=zeros(PixelNo,Nt+1);
C=    C_bg*ones(length(X),1);
C_new=C;
% 
GIFhand.im=1;ii=1;
for t_idx=1:Nt
    if t_idx<=ceil(t_i/dt)
        C_new=A1_newI*(A1*C+G1);
    else
        C_new=A2_newI*(A2*C+G2);
    end
    if rem(t_idx,400)==0 || t_idx==1
        
        plot(X,C_new)
        ylim([0 1])
        pause(0.01)
        [GIFhand]=GifWriter('BackExchangeWithInterface', ii, round(Nt/400),GIFhand);
        ii=ii+1;
    end
    C=C_new;
end
pro=C(:,end); pro=pro';
keyboard
% pro
% combine with linear grad if profile extends outsde domain
if pro(end)>C_bg*1.1 %max(pro)-pro(end)>pro(end)*1.01
    C=zeros(PixelNo,Nt+1);
    C(:,1)=C_bg;
    %%% Linear grad.
    A1(end,end-2:end)=    [-sigma1 2*sigma1 1-sigma1]; %central dif
    A2(end,end-2:end)=    [-sigma2 2*sigma2 1-sigma2];
    A1_new(end,end-2:end)=[sigma1 -2*sigma1 1+sigma1];
    A2_new(end,end-2:end)=[sigma2 -2*sigma2 1+sigma2];
    A1_newI=A1_new\I;
    A2_newI=A2_new\I;
    % Time stepping
    for t_idx=1:Nt
        if t_idx<=ceil(t_i/dt)
            C(:,t_idx+1)=A1_newI*(A1*C(:,t_idx)+G1);
        else
            C(:,t_idx+1)=A2_newI*(A2*C(:,t_idx)+G2);
        end
    end
    
    pro=(pro'+C(:,end))./2; pro=pro';%pro=C(:,end); pro=pro';
    
end