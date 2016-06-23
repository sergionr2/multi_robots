% This program was used to code the simulation.py
% see simulator.py for explication
% This program simulates the dinamics of the robot
close all
clear all

L = 0.100;
R = 0.030; %
f = 30; %Hz
dt = 1/f;
maxTime = 10; % seconds

t = ( 0:dt:maxTime );

x = zeros( size(t) );
y = zeros( size(t) );
ang = zeros( size(t) );

w_r = zeros( size(t) );
w_l = zeros( size(t) );
angle_error = zeros( size(t) );
norm = zeros( size(t) );
acum = zeros( size(t) );
s = zeros( size(t) );
% Actuador limits
W_MAX = 5;  % rads/s
W_MIN = 3;
VEL_MAX = 5; % rads/s  *R to m/s
VEL_MIN = 2;
% CONTROLLER GAINS HERE
kp = 5;
D = 0.400;
kp_v = VEL_MAX/D; % if error larger than D, use max speed
ki_v = 0;
%---
x_goal = 1.400; %m
y_goal = 0.200;

x(1) = 0.200; %initial Pose
y(1) = 0.200;
ang(1) = 0;

MAX_POS_ERROR = 0.020; %m
MAX_ANGLE_ERROR = 5; %Degrees

for i = 2:length(t)
    % controller here
    d_x = x_goal - x(i-1); %distance in x
    d_y = y_goal - y(i-1); %distance in y
    %
    norm(i) = sqrt( d_x*d_x + d_y*d_y );
    dotProduct = d_x*-1*sin(ang(i-1)) + d_y*cos(ang(i-1));
    angle_error(i) = acos( dotProduct/norm(i) );
    
    dotProduct2 = d_x*-1*sin(ang(i-1)+0.01) + d_y*cos(ang(i-1)+0.01);
    nextError =  acos( dotProduct2/norm(i) );
    if( nextError > angle_error(i) )
        angle_error(i) = -1*angle_error(i);
    end
    acum(i) = acum(i-1) + norm(i);

    w_r(i) = angle_error(i)*kp;
    if( w_r(i) > W_MAX )
        w_r(i) = W_MAX;
    elseif( w_r(i) < -1*W_MAX )
        w_r(i) = -1*W_MAX;
    elseif( w_r(i) < W_MIN && w_r(i) > -1*W_MIN )
        if( w_r(i) > 0)
            w_r(i) = W_MIN;
        else
            w_r(i) = -1*W_MIN;
        end
    end

    if( angle_error(i)*180/pi > -1*MAX_ANGLE_ERROR && angle_error(i)*180/pi < MAX_ANGLE_ERROR )
        w_r(i) = 0;
    end
    w_l(i) = -1*w_r(i);
    s(i) = ( kp_v * norm(i) + ki_v * acum(i) );
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
    w_r(i) = w_r(i) + s(i);
    w_l(i) = w_l(i) + s(i);

    if( norm(i) < MAX_POS_ERROR && norm(i) > -1*MAX_POS_ERROR )    
        w_r(i) = 0;
        w_l(i) = 0;
    end
    w = R*(w_r(i)-w_l(i))/L;
    v = R*(w_r(i)+w_l(i))/2;
    ang(i) = ang(i-1) + w*dt;
    if( ang(i) > pi )
        ang(i) = ang(i) - 2*pi;
    elseif( ang(i) < -1*pi )
        ang(i) = ang(i) + 2*pi;
    end
    x(i) = x(i-1) - v*sin( ang(i) )*dt;
    y(i) = y(i-1) + v*cos( ang(i) )*dt;
end

figure
scatter( x,y )
grid on 
title('Position( m )')
legend('trajectory')
xlabel('Position in X (m)')
ylabel('Position in Y (m)')
axis([ min(x_goal, y_goal) max(x_goal, y_goal) min(x_goal, y_goal) max(x_goal, y_goal)])
figure
plot( t, ang*180/pi )
grid on 
title('Angle(°)')
hold on 
plot( t, angle_error*180/pi )
legend('Angle', 'Angle Error')
xlabel('time( s )')
ylabel('Angle')
figure
title('Distance( m )')
plot( t, norm )
legend('Distance Error')
xlabel('time( t )')
title('distance to the goal')
figure
title('Angular Velocity( rads/s )')
hold on
plot( t, w_r )
plot( t, w_l )
xlabel('time( s )')
legend('w_r', 'w_l')
figure
hold on
plot( t, s )
title('Speed( m/s ) ')
xlabel('time( t )')
