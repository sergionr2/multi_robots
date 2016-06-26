% This program was used to code the sim_Point.py
% This program simulates the dinamics of the robot
close all
clear all

f = 30; %Hz
dt = 1/f;
maxTime = 15; % seconds, time of simulation

t = ( 0:dt:maxTime );

x = zeros( size(t) );
y = zeros( size(t) );

distance = zeros( size(t) );
acum = zeros( size(t) );

VEL_MAX = 0.200; % m/s
VEL_MIN = 0;
% CONTROLLER GAINS HERE
D = 0.400;
kp_v = VEL_MAX/D; % if error larger than D, use max speed
ki_v = 0; 
%---
x_goal = 1.00; %m
y_goal = 2.00;

x(1) = 0.200; %initial Pose
y(1) = 0.200;
ang(1) = 0;

MAX_POS_ERROR = 0.001; %m

for i = 2:length(t)
    % controller here
    d_x = x_goal - x(i-1); %distance in x
    d_y = y_goal - y(i-1); %distance in y
    %
    distance(i) = sqrt( d_x*d_x + d_y*d_y );
    acum(i) = acum(i-1) + distance(i)*dt;

    s(i) = ( kp_v * distance(i) + ki_v * acum(i) );
    if( s(i) > VEL_MAX )
        s(i) = VEL_MAX;
    elseif( s(i) < -1*VEL_MAX )
        s(i) = -1*VEL_MAX;
    elseif( s(i) < VEL_MIN && s(i) > -1*VEL_MIN )
        if( s(i) > 0 )
            s(i) = VEL_MIN;
        else
            s(i) = -1*VEL_MIN;
        end
    end
    if( distance(i) <= MAX_POS_ERROR )    
        s(i) = 0;
    end
    ang = atan2( d_y, d_x );
    x(i) = x(i-1) + s(i)*cos( ang )*dt;
    y(i) = y(i-1) + s(i)*sin( ang )*dt;
end

figure
scatter( x,y )
grid on 
title('Position( m )')
legend('trajectory')
xlabel('Position in X (m)')
ylabel('Position in Y (m)')
figure
title('Distance( m )')
plot( t, distance )
legend('Distance Error')
xlabel('time( t )')
title('distance to the goal')
figure
hold on
plot( t, s )
title('Speed( m/s ) ')
xlabel('time( t )')
