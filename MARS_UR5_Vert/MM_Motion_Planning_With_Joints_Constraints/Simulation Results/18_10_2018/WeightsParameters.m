%% Set the step size for the gradient descent method and error weight.
%A higher error weight might decrease the manipulability because of its
%influence on the motion.

% Use Fs=20Hz
ts=1/20;   
alpha=6;
kappa=10;  %Position error weight
lambda=0.1;   %Orientation error weigth

% Use Fs=100Hz
% ts=0.005;  %Overwrite ts
% alpha=8;   %Best alpha=8
% kappa=10;  %Position error weight
% lambda=0.1;   %Orientation error weigth
