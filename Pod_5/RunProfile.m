% Pod 5 Run Profile Code
clear 
clc
close all

%Inputs to the function (include quotes,case sensitive):
%"Hyperloop" - indicates run profile will be calculated for a full hyperloop run
%"External_subtrack" - indicates the run profile will be calculated for an external subtrack run
%"Open_air" - indicates the run profile will be calcuated for an open air run
%Considering adding in braking pressure
%Put breakpoint before plotting section to see variables from RunProfile function

prompt = "Enter run profile type -> Hyperloop, External_subtrack, or Open_air:";

[table,percent_error,End_mode] = P5RunProfile((input(prompt,'s')))

function [T,percent_error,End_mode] = P5RunProfile(run_type)
%% Constants
wD = 10.45; % wheel diameter in inches
pM =348.2/2.204; %kg - updated 11/3/19 from steamfitters trip in fall 2019

secondary_delay=1;%time mesaured in seconds from when primary brakes are commanded takes into account waiting time on the microcontroller plus the time to actuate secondary
primary_delay=0.5;%time mesaured in seconds from when primary brakes are commanded takes into account lag within the braking system itself
rolling_drag=50;%N  
CD=0.19; %pod 4 (10/14) 
Area_pod=0.28;%m^2 pod 4 (10/14) - try and make sure this is updated
gear_ratio=1;   %set to 1 as the system is now direct drive
num_vert= 2 ; % # of vertical wheels in the stability system of the same size
num_lat= 4 ;%# of lateral wheels of the same size
num_prop= 1 ;% # of propulsion wheels
minertia_lat=8.92e-5 ; % mass moment of inertia derived value from CAD in kg-m^2 
minertia_vert=8.92e-5  ; % same as above
minertia_prop = 0.05084258; %mass moment of inertia for propulsion wheel AND coating from CAD in kg-m^2
r_lat=convlength(1.5,'in','m');   %pod 4 wheel size convlength(radius,'unit input','unit desired')
r_vert=convlength(1.5,'in','m'); %pod 4
r_prop=convlength(wD/2,'in','m'); %meters

%% Function Logic
if run_type == "Hyperloop"
    trackLength = 1250; %meters
    safetyDistance = 31+90+20; %meters 31 for the 100ft rule, 90 for the max braking delay of 1 second between primary and secondary, and 20 as an additional margin 
    RPM_limit = 6750; %Max speed of motor
    air_rho=0.00738; %kg/m^3 density of air at 0.135psi 77F 50%RH 
elseif run_type == "External_subtrack"
    trackLength = convlength(150,'ft','m'); % external subrack length - meters
    safetyDistance = convlength(50,'ft','m'); %somewhat arbritrarily picked as 1/3 of the total track 
    speed_limit = convvel(25,'mph','m/s'); %max allowable speed for external subtrack run
    RPM_limit = 60*speed_limit/(r_prop*2*pi()); %convert speed limit to RPMs
    air_rho = 1.1807; %kg/m^3 density of air at 14.7psi 77F 50%RH 
elseif run_type == "Open_air"
    trackLength = 1250; %meters
    safetyDistance = 31+90+20; %meters 31 for the 100ft rule, 90 for the max braking delay of 1 second between primary and secondary, and 20 as an additional margin 
    speed_limit = convvel(50,'mph','m/s'); %max allowable speed for open air run
    RPM_limit = 60*speed_limit/(r_prop*2*pi()); %convert speed limit to RPMs
    air_rho = 1.1807; %kg/m^3 density of air at 14.7psi 77F 50%RH 
end

%% Braking deceleration calculations
%Constants
pressure = 250; %psig P4 MEOP = 140 psig
spring_losses = 2*30; % 2 actuators per side and 30 lbf per spring
F_a=convforce(2*pi()*pressure,'lbf','N') - convforce(spring_losses,'lbf','N'); %F_a is the applied normal force put out by the actuator. pi*psi
cof=0.3; %coefficient of friction depending on the brake pad used
theta = 40; %[deg] - angle made when actuated

%Force balance 
braking_coeff = [-1,cof,0;-1,0,2*cosd(theta);0,-1,-2*sind(theta)]; %coefficient matrix
braking_rhs = [0;0;-F_a]; %right hand side vector
braking_ans = braking_coeff\braking_rhs; %[F_f;F_R;F_t] F_f = friction force, F_R = reaction force from track, F_a = applied force, F_t = force in swingarms

%Answers
F_f=braking_ans(1); %F_f is friction force per side of 1 system
F_t = braking_ans(3); %F_t is the force in the linkage. x direction balance
F_R = braking_ans(2); %y direction balance
F_f_tot = 2*F_f; %total friction force in 1 braking system
decel = -F_f_tot/pM; 
G_s_decel = decel/9.806; %g force calc
%decel = 2*-9.806; % g's of braking

%% Run Calculations    
m_rot=num_lat*(minertia_lat/r_lat^2)+num_vert*(minertia_vert/r_vert^2)+num_prop*(minertia_prop/r_prop^2);%converting over the wheel interia the an equivalent mass
pM=pM+m_rot; % pM is equivalent with wheel inertias now

dt = 1e-3;  %time step size
v = 0;  %velocity array initialize/reset
x = 0;  %position array initialize/reset
t = 0;  %time array initialize/reset
a=0;    %acceleration array initialize/reset
c = 1;  %step counter initialize/reset
r=0;    %boolean for propulsion loop initialize/reset

current_voltage(c) = 290;  %starting battery voltage
accumulated_power = 0;
current_capacity(c) = 8; % 8 aH

torque=10*gear_ratio;   %torque on the wheel initialize - estimate around 10 Nm for initial torque due to scaling of MC
force_prop = (torque/r_prop) - rolling_drag; %no need for air losses at initialization
a(1)= force_prop/pM;%N for first time step of the propulsion phase

%% Propulsion phase loop
while r==0
    c = c + 1;  %loop index incriment
    v(c) = a(c-1)*dt + v(c-1);  %forward euler to determine next velocity value
    x(c) = v(c-1)*dt + x(c-1);    %forward euler to determine next position value
    t(c) = (c-1)*dt;    %time array incriment

    RPM = (v(c)*60)/(pi()*2*r_prop); %RPM calculated
    torque = ((1e-07*RPM^2) + (-0.0019*RPM) + 90);%torque at the motor - same as torque at the wheel
    force_prop = (torque/r_prop) - (0.5 *CD*air_rho*Area_pod*(v(c))^2) - rolling_drag; %assume power limited, not friction limited
    a(c) = force_prop/pM; %next acceleration value

    PowerLoss(c) = (17.5 + (0.0499*RPM) + (1.73E-05*RPM^2)); % internal (free run) losses in watts
    CurrentPower(c) = ((torque * RPM*2*pi()/60) - PowerLoss(c)); % Watts - added in losses from rolling drag and air reseitance in the form of a negative torque on the wheel/motor
    load_amps(c) = CurrentPower(c-1)/ current_voltage(c-1); % amps required for desired power
    charge_used = load_amps(c) * (dt / 3600); %amp hrs used in single iteration
    current_capacity(c) =  current_capacity(c-1) - charge_used; %subtracts charge used from current capacity
    current_voltage(c) = get_voltage(load_amps(c), current_capacity(c)); % new current voltage
    accumulated_power = accumulated_power + ((current_voltage(c) * load_amps(c))* dt / 1000); %for each iteration, calculates power, sums up - this is actually energy in KJ
    power(c) = accumulated_power; %accumulated energy in KJ
    Max_RPM = 24 * current_voltage(c); % 24 is slope of line of (Power/RPM), pretty linear
    mech_KE1(c) = (torque * RPM*2*pi()/60)*dt / 1000; %another energy check
    mech_KE2(c) = CurrentPower(c)*dt/1000; %power from torque and angular velocity

    if (((v(c))^2/(2*(-decel)))+(primary_delay+secondary_delay)*v(c))>=(trackLength - safetyDistance - x(c)) %calculating if the pod can stop in time at the current velocity
        %Incuding primary and secondary delays because we need to stop in time for off-nominal runs
        vMax=v(c);  %max velocity to display in the results
        vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
        r=1;    %to move out of the propulsion phase
        End_mode={'No more track'}; %Reason the run ended
    end

    if RPM>=Max_RPM   %checks if the Max RPM at the given battery voltage has been reached
        vMax=v(c);
        vMax_mph=convvel(v(c), 'm/s', 'mph');
        r=2;
        End_mode={'No more motor RPM'};
    end
    
    if RPM>=RPM_limit %checks if the speed limited by the external subtrack or open air runs has been reached
        vMax = v(c);
        vMax_mph = convvel(v(c), 'm/s','mph');
        r=3;
        End_mode = {'Max speed limited'};
    end
end

%Saving variables for use in secondary braking, plotting, or later reference
t_p_time = t; %time step array of the propulsion phase to help make the voltage vs time plot
t_prop=t(c);    %propulsion time
a_store = a(c);
s=c;
plotlim=c;
v_secondary=v;
x_secondary=x;
t_secondary=t;

%% Braking phase loop
%Primary Braking Case
while v(c)>=0
    if t(c)<=(t_prop+primary_delay)
        c = c + 1;
        v(c)=(v(c-1)-dt*rolling_drag/pM);
        x(c)=v(c-1)*dt + x(c-1);
        t(c)= c*dt;
    end
    if t(c)>(t_prop+primary_delay)
        c = c + 1;
        v(c)=v(c-1)+decel*dt; %worst case here since no rolling or aero drag included
        x(c)=v(c-1)*dt + x(c-1);
        t(c)= c*dt;
    end
end

%Secondary Braking Case
while v_secondary(s)>=0
    if t_secondary(s)<=(t_prop+primary_delay+secondary_delay)
        s = s + 1;
        v_secondary(s)=(v_secondary(s-1)-dt*rolling_drag/pM);
        x_secondary(s)=v_secondary(s-1)*dt + x_secondary(s-1);
        t_secondary(s)= s*dt;
    end
    
    if t_secondary(s)>(t_prop+primary_delay+secondary_delay)
        s = s + 1;
        v_secondary(s) = decel*dt + v_secondary(s-1);   %decel is the decelaration value for braking and is assumed constant
        x_secondary(s) = v_secondary(s-1)*dt + x_secondary(s-1);
        t_secondary(s) = s*dt;
    end
end

xMax_p = x(c);    %Distance the pod travelled by the end of the run
tMax_p = t(c);    %total time taken
xMax_s = x_secondary(s); %total distance travelled in secondary run
tMax_s = t_secondary(s); %total time taken for secondary run

%Checking to see which final times and distances are longest
if xMax_s >= xMax_p
    xMax = xMax_s;
else
    xMax = xMax_p;
end

if tMax_s >= tMax_p
    tMax = tMax_s;
else
    tMax = tMax_p;
end
%% Post process
%Compile data to table
dat=[wD, vMax, vMax_mph,t_prop, End_mode, tMax, xMax, a_store]; %output array
header = {'Wheel_diameter_inches','Velocity_max','Velocity_max_mph','Time_propulsion','End_Mode','Run_Time_s', 'Distance_travelled', 'Acceleration_last'}; %Column header titles
T = array2table(dat,'VariableNames',header);    %converted to a table

%Solve for pod KE at max speed
KE_tot = (0.5*pM*vMax^2)/1000 + sum(PowerLoss.*.001)/1000 %total KE of pod, using equivalent mass from above in KJ
KE_tot1 = sum(mech_KE1) %energy from "integrating" torque*omega curve from each RPM
KE_tot2 = sum(mech_KE2)
KE_elec = power(end) %in KJ
percent_error = 100*abs((KE_tot2-KE_elec)/KE_tot2); %Chose the mechanical kinetic energy as the theoretical value since it has less chance for error and is better defined

%% Plots
figure(1)
plot(x,v,'k')   %plot displacement vs velocity
title('Velocity vs. Position')
xlabel('Distance [m]')
ylabel('Velocity [m/s]')
hold on
plot(x_secondary(plotlim:s),v_secondary(plotlim:s),'--r')
legend('Nominal','Off-Nominal')

ylim([0,90])
hold off
figure(2)
plot(t,x,'k')   %plot displacemnt vs time
ylabel('Distance[m]')
xlabel('time[s]')
hold on
plot(t_secondary(plotlim:s),x_secondary(plotlim:s),'--r')
legend('Nominal','Off-Nominal')

hold off
figure(3)
plot(t,v,'k')   %plot velocity vs time
xlabel('time[s]')
ylabel('Velocity[m/s]')
hold on
plot(t_secondary(plotlim:s),v_secondary(plotlim:s),'--r')
legend('Nominal','Off-Nominal')

ylim([0,90])
hold off
figure(4)
plot(t_p_time,current_voltage,'k'); %plot battry pack voltage vs time(till end of propulsion phase)
xlabel('time[s]');
ylabel('voltage[V]');

figure(5)
plot(t_p_time,current_capacity,'k'); 
xlabel('time[s]');
ylabel('Capacity(aH)');

figure(6)
plot(t_p_time,CurrentPower,'k'); 
xlabel('time[s]');
ylabel('Power[W]');
end

function x = get_voltage(amps, current_capacity)

internal_resistance = 0.004; %ohms per cell

% capacity Percentages from 0% to 100% incremented by 5%
capacity_lut = [2.0 3.0 3.25 3.3 3.35 3.35 3.35 3.4 3.4 3.4 3.45 3.45 3.45 3.475 3.48 3.49 3.49 3.495 3.5 3.55 3.6]; %Updated 11/6 for a123 cells

total_capacity = 8; % 6 Ah the starting capacity of the battery
percent_charge = (current_capacity / total_capacity); % need to figure out the current capacity
lut_idx = round(percent_charge*length(capacity_lut)); %index for capacity_lut lookup table

x = (capacity_lut(lut_idx) - (internal_resistance * amps)) * 84 ; % 84 cells at this voltage - updated 11/6 for a123 cells

end