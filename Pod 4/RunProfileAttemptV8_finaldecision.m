%% run calcs

clear
clc
close all

% constants
final_wheel_decision=10.45;%this was decided on after looking at all manufacturing and design constraints. The decision shows robustness to the rolling drag and consistently provides 188mph
trackLength = 1250; %meters
safetyDistance = 31+90+20; %meters 31 for the 100ft rule, 90 for the max braking delay of 1 second between primary and secondary, and 20 as an additional margin 
wD = 6.25; % wheel diameter in inches
pM =137.7; %kg

decel = 2*-9.806; % g's of braking
secondary_delay=1;%time mesaured in seconds from when primary brakes are commanded takes into account waiting time on the microcontroller plus the time to actuate secondary
primary_delay=0.5;%time mesaured in seconds from when primary brakes are commanded takes into account lag within the braking system itself

rolling_drag=50;%N 
air_rho=0.00738; %kg/m^3 density of air at 0.135psi 77F 50%RH 
CD=0.19; %pod 4 (10/14) 
Area_pod=0.28;%m^2 pod 4 (10/14)
T=table();
Max_RPM=6750; %max motor RPM
gear_ratio=1;   %set to 1 as the system is now direct drive
num_vert= 2 ; % # of vertical wheels in the stability system of the same size
num_lat= 4 ;%# of lateral wheels of the same size
num_prop= 1 ;% # of propulsion wheels
minertia_lat=8.92e-5 ; % mass moment of inertia derived value from CAD in kg-m^2 
minertia_vert=8.92e-5  ; % same as above
r_lat=convlength(1.5,'in','m');   %pod 3 wheel size convlength(radius,'unit input','unit desired')
r_vert=convlength(1.5,'in','m'); %pod 3
for wD= 10.35:0.05:10.45%in the upper and lower bound was set to this after results convereged within this range
    
    r_prop=convlength(wD/2,'in','m');
    minertia_prop= (r_prop)^2 *(0.4468*wD-0.9262)*1/2 ; %modelled as a solid disk
    pM=135.72; %pod mass without propulsion wheel
    pM=pM+(0.4468*wD-0.9262); %pM with prop wheel weight
    m_rot=num_lat*(minertia_lat/r_lat^2)+num_vert*(minertia_vert/r_vert^2)+num_prop*(minertia_prop/r_prop^2);%converting over the wheel interia the an equivalent mass
    pM=pM+m_rot; % pM is equivalent with wheel weight now
    
    
    dt = 1e-3;  %time step size
    v = 0;  %velocity array initialize/reset
    x = 0;  %position array initialize/reset
    t = 0;  %time array initialize/reset
    a=0;    %acceleration array initialize/reset
    RPM = 0;    %RPM array initialize/reset
    c = 1;  %step counter initialize/reset
    r=0;    %boolean for propulsion loop initialize/reset
    
    current_voltage = 290;  %starting battery voltage
    accumulated_power = 0;
    power = zeros(25002);
    tt = linspace(0,25,25000); %create vector of 25000 evenly spaced points beween 0 and 25
    tt = [tt 25001 25002];  %adds 0 to tt vector
    current_capacity = 6; % 6 aH
    load_amps = 300; % countinous current required by the Motor
    ii = 2; %index variable for tt and power vectors
    
    torque=90*gear_ratio;   %torque on the wheel initialize
    force_prop= torque/((wD*0.0254)/2) - (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
    a(1)= force_prop/pM;%N for first time step of the propulsion phase
    while r==0%propulsion phase loop
        c = c + 1;  %incriment of time
        v(c) = a(c-1)*dt + v(c-1);  %forward euler to determine next velocity value
        x(c) = v(c-1)*dt + x(c-1);    %forward euler to determine next position value
        t(c) = (c-1)*dt;    %time array incriment
        
        RPM=(v(c)*60)/(pi()*(wD*0.0254)) ;     %RPM calculated
        RPM_motor=RPM*gear_ratio;   %seperate parameter if gear ratio is involved
        motor_torque=((1e-07*RPM_motor^2) + (-0.0019*RPM_motor) + 90);%torque at the motor
        torque=gear_ratio*motor_torque; %takes care of the torque lost with the RPM toque
        force_prop= torque/((wD*0.0254)/2)- (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
        a(c)= force_prop/pM;    %next acceleration value
        
        PowerLoss = (17.5 + (0.0499*RPM_motor) + (1.73E-05*RPM_motor^2)); % internal losses in watts
        CurrentPower = ((motor_torque*2*pi() * RPM_motor)/60) - PowerLoss; % Watts
        load_amps = CurrentPower/ current_voltage; % amps required for desired power
        charge_used = load_amps * (dt / 3600); %amp hrs used in single iteration
        current_capacity =  current_capacity - charge_used; %subtracts charge used from current capacity
        accumulated_power = accumulated_power + ((get_voltage(load_amps,current_capacity) * load_amps) / 1000); %for each iteration, calculates power, sums up
        current_voltage = get_voltage(load_amps, current_capacity); % new current voltage
        power(ii) = accumulated_power;
        
        ii = ii + 1; %increments index of ii
        
        Max_RPM = 24 * current_voltage; % 24 is slope of line of (Power/RPM), pretty linear
        
        
        
        
        
        if (((v(c))^2/(2*(-decel)))+primary_delay*v(c))>=(trackLength - safetyDistance - x(c)) %calculating if the pod can stop in time at the current velocity
            vMax=v(c);  %max velocity to display in the results
            vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
            r=1;    %to move out of the propulsion phase
            End_mode={'No more track'}; %Reason the run ended
            
        end
        
        if RPM_motor>=Max_RPM   %checks if the Max RPM at the given battery voltage has been reached
            vMax=v(c);
            vMax_mph=convvel(v(c), 'm/s', 'mph');
            r=2;
            End_mode={'No more motor RPM'};
        end
    end
    t_prop=t(c);    %propulsion time
    while v(c)>=0
        if t(c)<=(t_prop+primary_delay)
            c = c + 1;
            v(c)=(v(c-1)-dt*rolling_drag/pM);
            x(c)=v(c-1)*dt + x(c-1);
            t(c)= c*dt;
        end
        if t(c)>(t_prop+primary_delay)
            c = c + 1;
            v(c)=v(c-1)+decel*dt;
            x(c)=v(c-1)*dt + x(c-1);
            t(c)= c*dt;
            
        end
        
    end
    
    xMax = x(c);    %Distance the pod travelled by the end of the run
    tMax = t(c);    %total time taken
    dat=[gear_ratio, wD, vMax, vMax_mph,t_prop, End_mode, tMax, xMax,]; %output array
    data=array2table(dat());    %converted to a table
    T=vertcat(T,data);  %the values converted to the table are vertically concatinated into a master table T
    
end

%post process
T.Properties.VariableNames={'Gear_Ratio','Wheel_diameter','Velocity_max','Velocity_max_mph','Time_propulsion','End_Mode','Run_Time_s', 'Distance_travelled',};
T=sortrows(T,4,'descend') %to display the top3 results of the different wheel diameter

%plot of winning run
%the code runs the same as above just needs to plot the values of the fastest run as the run values aren't saved in each iteration

wD=final_wheel_decision;



% conditions
dt = 0.001;
v = 0;
x = 0;
t = 0;
a=0;
RPM = 0;
c = 1;
r=0;
current_voltage=0;
current_voltage(c) = 290;
accumulated_power = 0;
power = zeros(25002);
tt = linspace(0,25,25000); %create vector of 25000 evenly spaced points beween 0 and 25
tt = [tt 25001 25002];  %adds 0 to tt vector
current_capacity = 6; % 6 aH
load_amps = 300; % countinous current required by the Motor
ii = 1; %index variable for tt and power vectors
r_prop=convlength(wD/2,'in','m');
minertia_prop= (r_prop)^2 *(0.4468*wD-0.9262)*1/2 ; %modelled as a solid disk
pM=135.72; %pod mass without propulsion wheel
pM=pM+0.4468*wD-0.9262; %pM with prop wheel weight
m_rot=num_lat*(minertia_lat/r_lat^2)+num_vert*(minertia_vert/r_vert^2)+num_prop*(minertia_prop/r_prop^2);
pM=pM+m_rot; % pM is equivalent with wheel weight now


torque=90*gear_ratio;
force_prop= torque/((wD*0.0254)/2)- (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
a(1)= force_prop/pM;%N
r_prop=convlength(wD/2,'in','m');
t_func(1)=(force_prop*r_prop)*8.85;%torque over time this is a derived value only in-lbs for the dynamic model
while r==0%prop loop
    c = c + 1;
    v(c) = a(c-1)*dt + v(c-1);  %forward euler to determine next velocity value
    x(c) = v(c-1)*dt + x(c-1);    %forward euler to determine next position value
    t(c) = (c-1)*dt;    %time array incriment
    
    RPM=(v(c)*60)/(pi()*(wD*0.0254)) ;
    RPM_motor=RPM*gear_ratio;
    motor_torque=((1e-07*RPM_motor^2) + (-0.0019*RPM_motor) + 90);%torque at the motor
    torque=gear_ratio*motor_torque; %takes care of the torque lost with the RPM toque
    force_prop= torque/((wD*0.0254)/2)- (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
    a(c)= force_prop/pM;
    t_func(c)=(force_prop*r_prop)*8.85;%torque over time this is a derived value only in-lbs for the dynamic model
    
    
    CurrentPower(c) = (motor_torque*2*pi() * RPM_motor)/60; % Watts
    load_amps = CurrentPower(c-1) / current_voltage(c-1); % amps required for desired power
    charge_used = load_amps * (dt / 3600); %amp hrs used in single iteration
    current_capacity(c) =  current_capacity(c-1) - charge_used; %subtracts charge used from current capacity
    accumulated_power = accumulated_power + ((get_voltage(load_amps,current_capacity(c)) * load_amps) / 1000); %for each iteration, calculates power, sums up
    current_voltage(c) = get_voltage(load_amps, current_capacity(c)); % new current voltage
    power(ii) = accumulated_power;
    
    ii = ii + 1; %increments index of ii
    
    Max_RPM = 24 * current_voltage; % 24 is slope of line of (Power/RPM), pretty linear
    
    
    if RPM_motor>=Max_RPM
        vMax=v(c);
        vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
        r=1;
        
    end
    if (trackLength - safetyDistance - x(c))<=(v(c)^2/(2*-decel))
        vMax=v(c);
        vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
        r=2;
        
    end
    t_p_time(c)=t(c);%time step array of the propulsion phase to help make the voltage vs time plot
    t_p_max=t_p_time(c);
end

braking_dist=(v(c)^2/(2*-decel));   %m total distance the braking phase takes
s=c;
plotlim=c;

v_secondary=v;
x_secondary=x;
t_secondary=t_p_time;
while v(c)>=0
    if t(c)<=(t_p_max+primary_delay)
        c = c + 1;
        v(c)=(v(c-1)-dt*rolling_drag/pM);
        x(c)=v(c-1)*dt + x(c-1);
        t(c)= c*dt;
    end
    if t(c)>(t_p_max+primary_delay)
        c = c + 1;
        v(c)=v(c-1)+decel*dt;
        x(c)=v(c-1)*dt + x(c-1);
        t(c)= c*dt;
        
    end
    
end
while v_secondary(s)>=0
    if t_secondary(s)<=(t_p_max+secondary_delay)
        s = s + 1;
        v_secondary(s)=(v_secondary(s-1)-dt*rolling_drag/pM);
        x_secondary(s)=v_secondary(s-1)*dt + x_secondary(s-1);
        t_secondary(s)= s*dt;
    end
    
    if t_secondary(s)>(t_p_max+secondary_delay)
        s = s + 1;
        v_secondary(s) = decel*dt + v_secondary(s-1);   %decel is the decelaration value for braking and is assumed constant
        x_secondary(s) = v_secondary(s-1)*dt + x_secondary(s-1);
        t_secondary(s) = s*dt;
    end
    
    
end

%plots
figure(1)
plot(x,v,'k')   %plot displacement vs velocity
xlabel('Distance[m]')
ylabel('Velocity[m/s]')
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
