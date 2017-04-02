# Example to check udacity results.
clear

%P =[0.0043, -.0013; -0.0013, 0.0077];
%x = [5.7441, 1.38];
%lambda = 3;
%sqrt_p = sqrt(lambda) * chol(P)';
%sigma =  x' + sqrt_p


%%% Example with udacity coeffs

P = [  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020;
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060;
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008;
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100;
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123];
x =  [5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528];
         
        
lambda = 3;
sqrt_p = sqrt(lambda) * chol(P)';
chol(P)'
sigma =  x - sqrt_p



