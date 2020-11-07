%% Calibration Data
% Ch1 = Data20(:,3);
% n = 1:length(Ch1);
% [new_signal_Ch1,Ks_Ch1] = waveletDemoisingBasedOnEMD(Ch1);
% plot(n,Ch1,'b:');
% hold on
% plot(n,new_signal_Ch1,'r');
% title('Original Signal vs. Denoised Signal');
% xlabel('time(n)')
% ylabel('Ch1 value(V)')
% legend('Original Signal','Denoised Signal');


%% Test Data
% load('AccData');
% X_axis = AccData(:,1);
% Y_axis = AccData(:,2);
% Z_axis = AccData(:,3);
% n = 1:length(X_axis);
% [new_signal_X,Ks_X] = waveletDemoisingBasedOnEMD(X_axis);
% [new_signal_Y,Ks_Y] = waveletDemoisingBasedOnEMD(Y_axis);
% [new_signal_Z,Ks_Z] = waveletDemoisingBasedOnEMD(Z_axis);
% subplot(3,1,1);
% title('X-axis Denoised Signal vs. Original Signal')
% xlabel('sample')
% ylabel('Acceleration Value(g)')
% plot(n,X_axis,'b:');
% hold on
% plot(n,new_signal_X,'r');
% 
% subplot(3,1,2);
% title('Y-axis Denoised Signal vs. Original Signal')
% xlabel('sample')
% ylabel('Acceleration Value(g)')
% plot(n,Y_axis,'b:');
% hold on
% plot(n,new_signal_Y,'r');
% 
% subplot(3,1,3);
% title('Z-axis Denoised Signal vs. Original Signal')
% xlabel('sample')
% ylabel('Acceleration Value(g)')
% plot(n,Z_axis,'b:');
% hold on
% plot(n,new_signal_Z,'r');

%% Test MPU6050 Data
load('MPUData');
Yaw = MPUData(:,1);
Pitch = MPUData(:,2);
Roll = MPUData(:,3);
n = 1:length(Yaw);
[new_Yaw,Ks_X] = waveletDemoisingBasedOnEMD(Yaw);
[new_Pitch,Ks_Y] = waveletDemoisingBasedOnEMD(Pitch);
[new_Roll,Ks_Z] = waveletDemoisingBasedOnEMD(Roll);
subplot(3,1,1);
title('Yaw Denoised Signal vs. Original Signal')
xlabel('sample')
ylabel('Angle(degree)')
plot(n,Yaw,'b');
hold on
plot(n,new_Yaw,'r--');

subplot(3,1,2);
title('Pitch Denoised Signal vs. Original Signal')
xlabel('sample')
ylabel('Angle(degree)')
plot(n,Pitch,'b');
hold on
plot(n,new_Pitch,'r--');

subplot(3,1,3);
title('Roll Denoised Signal vs. Original Signal')
xlabel('sample')
ylabel('Angle(degree)')
plot(n,Roll,'b');
hold on
plot(n,new_Roll,'r--');