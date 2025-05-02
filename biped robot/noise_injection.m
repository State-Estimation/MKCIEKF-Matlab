function noise_injection()
%
clear all

addpath(genpath('data'))
load('D:\Bit_study\IEKF\My-Contact-Aided-Invariant-EKF-master\Contact-Aided-Invariant-EKF-master\matlab_example\data\measurements\contact.mat')
load('D:\Bit_study\IEKF\My-Contact-Aided-Invariant-EKF-master\Contact-Aided-Invariant-EKF-master\matlab_example\data\measurements\encoders.mat')

t=contact.time;
Data=contact.Data;
rData=Data(:,1);
lData=Data(:,2);

% encoder=zeros(14,1);
% encoder1=[randn(7,1);zeros(7,1)];
% p_VectorNav_to_RightToeBottom(encoder1)-p_VectorNav_to_RightToeBottom(encoder); % 1-7 is for the left leg and 8-14 is for the right
% obtain the contact moment
len=length(t);
for i=1:len
    encoder_noise_std(i) = 0*ones(1,1); 
    if(lData(i)>0.1&&lData(i)<0.9)
        encoder_noise_std(i)=2*deg2rad(0.5)*ones(1,1);
    end
     
    if(rData(i)>0.1&&rData(i)<0.9)
        encoder_noise_std(i)=2*deg2rad(0.5)*ones(1,1);
    end

    
end

%%
figure
hold on
plot(t,rData)
plot(t,lData)
plot(t,encoder_noise_std,'Linewidth',2)
legend('rleg','lleg','noise std')

%%
enc=encoders.Data;
enc1=enc;
enc2=enc;

for i=1:len
    enc1(i,:)=enc(i,:)+encoder_noise_std(i).*randn(1,14);
    enc2(i,:)=enc(i,:)-encoder_noise_std(i).*randn(1,14);
end
%%
figure
hold on
plot(t,enc1)
plot(t,enc2)
legend on

%% average iteration number
figure
plot(imkckf.signals.values)


end
