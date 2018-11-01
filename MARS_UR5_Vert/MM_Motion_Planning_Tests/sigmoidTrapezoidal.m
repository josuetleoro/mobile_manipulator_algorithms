function y=sigmoidTrapezoidal(time,blend_perc)
tf=time(end);

%Incresing time
t1=tf*blend_perc;

%Decreasing time
t3=tf-t1;

N=length(time);
ts=time(end)/(N-1);

%Increasing segment
t=0:ts:t1;
trans1=sigmoid(t,t1/2,8);
y=trans1;

%Linear segment
t=t1:ts:t3-ts;
trans2=ones(size(t));
y=[y(1:end-1) trans2];

%Decreasing segment
%Using sigmoid
% t=t3:ts:tf;
% trans3=1-trans1;
% y=[y trans3];

%Using 5th order polynomial
[trans3,~,~]=Traj5(1,0,ts,0,t1);
trans3=1-trans1;
y=[y trans3];
% 
% plot(trans3)
% pause()

end
