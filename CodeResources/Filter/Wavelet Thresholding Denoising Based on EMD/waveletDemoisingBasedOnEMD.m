function [new_signal,Ks] = waveletDemoisingBasedOnEMD(x)
%% First Step: Implement Empirical Mode Decomposition (EMD) to original signal,x(t)
%  By using EMD method, dividing the signal into n IMF components and one
%  residual part. s.t. original signal, x(t) = ∑ ai + rn, where i is from 1
%  to n and rn is the residual part
%  There is a build in function in MATLAB which used for implementing EMD:
%  [imf,residual,info] = emd(x)

% Implement EMD to original signal and collect information:
[imf,residual] = emd(x,'Interpolation','pchip');
% Plot the IMF components and Residual part in the figure window (not necessary)
% emd(x);

%% Second Step: Define the cutoff point, Aks
% the component Ak if the kth IMF part from the first step. This index of
% the IMF will separate the signal IMF parts into two sets of high
% frequency band and low frequency band
% According to the continuous root mean square error of the signal:
% σCMSE( ˆxi, xˆi+1) = 1/N * (sum(ˆxi(t) − ˆxi+1(t))) = 1/N * sum(Ai(t))^2

N = length(x);
Ai = imf;
[r_imf,c_imf] = size(imf);
sita_CMSE = zeros(1,c_imf);
for k = 1:c_imf
    sita_CMSE(k) = 1/N * sum(Ai(:,k));
end
Ks = find(sita_CMSE == min(sita_CMSE));


%% Third Step: Implement wavelet denoising to high frequency band IMF
% SURE — Stein's Unbiased Risk Estimate
% This method uses a threshold selection rule based on Stein's Unbiased Estimate of Risk (quadratic loss function).
% One gets an estimate of the risk for a particular threshold value (t). Minimizing the risks in (t) gives a selection of the threshold value.
Imf_high_frequency = imf(:,1:Ks);
Imf_low_frequency = imf(:,Ks+1:end);
Imf_high_frequency_denoise = zeros(size(Imf_high_frequency));
for k = 1:Ks
    Imf_high_frequency_denoise(:,k) = wdenoise(Imf_high_frequency(:,k),4,'Wavelet','db4',...
    'DenoisingMethod','SURE',...
    'ThresholdRule','Soft');
end

imf_denoise = [Imf_high_frequency_denoise Imf_low_frequency residual]';
new_signal = sum(imf_denoise);
end