clear
close all
clc

% OPTICAL CAMERA FOR SPACE RELATIVE NAVIGATION

% this script is connected with the simulink model 'camera_model.slx'
% Matlab code imports data from txt files, initializes variables, launches the Simulink model and plot final results
% Simulink model computes relative distance


%% SPECIFICATIONS OF OPTICAL CAMERA

focal_length=0.05; % [m]
N=4096; % pixels in x direction (crosstrack)
M=1850; % pixels in y direction (alongtrack)
FOV_N=30; % [deg] fiel of view in x direction (crosstrack)
FOV_M=15; % [deg] field of view in y direction (alongtrack)
gsd_factor=40/300000; % the payload is able to distinguish objects 40 m long from a distance of 300 km

xfp=focal_length*tand(FOV_N); % [m]
yfp=focal_length*tand(FOV_M); % [m]

%% IMAGE PROCESSING CAPABILITIES

% minimum number of pixel needed in a marker to compute coordinates from photos (less px means better image processing capabilities)
pxPerMarker_min_v=[1,2,3]; % TRY DIFFERENT VALUES

%% PATTERN SPECIFICATIONS
pattern_dim_v=[5,6,7,8]; % [m] dimensions of pattern on the leader satellite (length of major axis)--> TRY DIFFERENT VALUES
marker_dim_v=0.01:0.02:0.1; % [m] diameter of markers that compose the pattern --> TRY DIFFERENT VALUES


maxRangeResults=zeros(length(pxPerMarker_min_v), length(pattern_dim_v), length(marker_dim_v)); % max operating range
minRangeResults=zeros(length(pxPerMarker_min_v), length(pattern_dim_v), length(marker_dim_v)); % min operating range

for cont_px=1:length(pxPerMarker_min_v)
    pxPerMarker_min=pxPerMarker_min_v(cont_px);
    
    for cont_pd=1:length(pattern_dim_v)
        pattern_dim=pattern_dim_v(cont_pd);
        
        for cont_md=1:length(marker_dim_v)
            marker_dim=marker_dim_v(cont_md);
    
%% TRAJECTORY DATA
dt=1; % simulation time step [s]

ds=0.1; % simulation space step [m]
reld=0.01:ds:1000; % relative distance [m]: what ranges do you want to test? SET THEM HERE!

min_reld=min(reld);
max_reld=max(reld);

simulation_time=[0:dt:length(reld)-1]';

%% DETERMINATION OF MARKER COORDINATES BY TRAJECTORY DATA
% (in real world they are computed  from photos of the pattern through image processing)

% major axis in the focal plane (inverse relation of camera agorithm)
amax=pattern_dim*focal_length./reld; % [m]

pos_reflectors=zeros(length(reld), 9); % [simulation_time, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y]
pos_reflectors(:,1)=simulation_time;

% marker coordinates in the focal plane. null LOS angles are imposed for semplicity
fp=amax/2;
pos_reflectors(:,2)=fp;
pos_reflectors(:,3)=0;
pos_reflectors(:,4)=0;
pos_reflectors(:,5)=-fp;
pos_reflectors(:,6)=-fp;
pos_reflectors(:,7)=0;
pos_reflectors(:,8)=0;
pos_reflectors(:,9)=fp;

%% SIMULATION OF CAMERA ALGORITHM
sim('camera_model') % launch Simulink model
range_camera(:,1)=[]; % from Simulink

%% TEST THE OPERATING RANGE OF THE OPTICAL CAMERA

% min operating range
range_min=-10;
contmin=0;
for cont_test=1:1:length(amax)
    if amax(cont_test)>2*yfp || amax(cont_test)>2*xfp
        contmin=contmin+1;

        if range_camera(cont_test)>range_min
            range_min=range_camera(cont_test);
        end
    end
end

% max operating range
range_max=10e50;
contmax=0;

for cont_test=1:1:length(amax)
    gsd=gsd_factor*range_camera(cont_test); % ground sampling distance
    pxPerMarker=marker_dim/gsd;
        
    if pxPerMarker<pxPerMarker_min
        contmax=contmax+1;
        if range_camera(cont_test)<range_max
            range_max=range_camera(cont_test);
        end
    end
end

maxRangeResults(cont_px, cont_pd, cont_md)=range_max;
minRangeResults(cont_px, cont_pd, cont_md)=range_min;


        end
        
    end

end

%% VISUALIZATION OF RESULTS

% min operating range
figure
p=plot(squeeze(minRangeResults(1,:,1)), pattern_dim_v);
p.Marker='*';
xlabel('min operating range [m]')
ylabel('pattern size [m]')
grid on
title('effect of pattern size on min operating range')


% max operating range
str=strings(length(pxPerMarker),1);
figure
for i=1:length(pxPerMarker_min_v)
    hold on
    title('effect of marker size on max operating range')
    p=plot(squeeze(maxRangeResults(i,1,:)),marker_dim_v);
    p.Marker='*';
    xlabel('max operating range [m]')
    ylabel('marker size [m]')
    grid on
    
    str(i)=strcat('pxPerMarker= ',num2str(pxPerMarker_min_v(i)));
end
legend(str, 'location', 'southeast')


str=strings(length(marker_dim_v),1);
figure
for i=1:length(marker_dim_v)
    hold on
    title('effect of image processing on max operating range')
    p=plot(squeeze(maxRangeResults(:,1,i)), pxPerMarker_min_v);
    p.Marker='*';
    xlabel('max operating range [m]')
    ylabel('px per marker [m]')
    grid on
    
    str(i)=strcat('marker size= ',num2str(marker_dim_v(i)), ' [m]');
end
legend(str)


