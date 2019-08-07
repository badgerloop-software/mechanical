%% run calcs

clear
clc
close all

%% Constants
final_wheel_decision=10.45;%inches - this was decided on after looking at all manufacturing and design constraints. The decision shows robustness to the rolling drag and consistently provides 188mph
trackLength = 1250/2; % external subrack length - meters
safetyDistance = 0; %meters 31 for the 100ft rule, 90 for the max braking delay of 1 second between primary and secondary, and 20 as an additional margin 
pM =375/2.204; %kg - updated 7/3/19

% Pod dimensions - ALL FROM BACK OF POD AND TAKING TOP SUBTRACK GROUND (DATUM)
full_length = convlength(88,'in','m'); %total length of pod 4
x_stable = convlength(63.5,'in','m'); %distance to front vertical stability
x_prop = convlength(19.5,'in','m'); %distance to propulsion wheel
x_CG = convlength(39,'in','m'); %distance to pod 4 CG
y_stable = 0; %height of stability wheel contact
y_prop = convlength(4.5,'in','m'); %height propulsion wheel contact
y_CG = convlength(6,'in','m'); %height of center of gravity

%% Braking deceleration calculations
pressure = 130; %psi
F_N=convforce(4*pi()*pressure,'lbf','N'); %F_N is the normal force put out by the actuator. pi*psi
cof=0.4; %coefficient of friction depending on the brake pad used
theta = 40; %[deg] - angle made when actuated

%Force balance 
braking_coeff = [-1,cof,0;-1,0,cosd(theta);0,1,sind(theta)];
braking_rhs = [0;0;-F_N];
braking_ans = braking_coeff\braking_rhs; %[F_f;F_R;F_j]

F_f=braking_ans(1); %F_f is friction force
F_j = braking_ans(3); %F_j is the force in the linkage. x direction balance
F_R = braking_ans(2); %y direction balance
decel = F_f/pM; 
G_s_decel = decel/9.806; %g force calc

%decel = 1.415*-9.806; % g's of braking 
secondary_delay=1;%time mesaured in seconds from when primary brakes are commanded takes into account waiting time on the microcontroller plus the time to actuate secondary
primary_delay=0.25;%time mesaured in seconds from when primary brakes are commanded takes into account lag within the braking system itself

torque_max = 90;
rolling_drag=30;%N - updated 7/19 from approx 4Nm torque requred to move pod 
air_rho=1.184; %kg/m^3 density of air at 14.7psi 77F 50%RH 
CD=0.19; %pod 4 (10/14) 
Area_pod=0.28;%m^2 pod 4 (10/14)
T=table();
Max_RPM=6750; %max motor RPM
gear_ratio=1;   %set to 1 as the system is now direct drive

%Wheel parameters
num_vert= 2 ; % # of vertical wheels in the stability system of the same size
num_lat= 4 ;%# of lateral wheels of the same size
num_prop= 1 ;% # of propulsion wheels
minertia_lat=8.92e-5 ; % mass moment of inertia derived value from CAD in kg-m^2 
minertia_vert=8.92e-5  ; % same as above
r_lat=convlength(1.5,'in','m'); %pod 4 wheel size convlength(radius,'unit input','unit desired')
r_vert=convlength(1.5,'in','m'); %pod 4
wD = 10.45; %inches - the upper and lower bound was set to this after results convereged within this range
r_prop=convlength(wD/2,'in','m'); %meters - radius prop wheel with coating pod 4
minertia_prop= (r_prop)^2 *(0.4468*wD-0.9262)*1/2 ; %modelled as a solid disk - moment of inertia vs. radius. this is equation of that curve
pM=375/2.204; %pod mass without propulsion wheel
m_rot=num_lat*(minertia_lat/r_lat^2)+num_vert*(minertia_vert/r_vert^2)+num_prop*(minertia_prop/r_prop^2);%converting over the wheel interia to an equivalent mass
pM=pM+m_rot; % pM is equivalent with wheel weight now
%% Initialize Propulsion Loop
%mu_list = []; %init
d = 1; %for loop counter
mu_range = 0.4; %range of friction coefficients to check
for mu = mu_range
    dt = 1e-3;  %time step size
    v = 0;  %velocity array initialize/reset
    x = 0;  %position array initialize/reset
    t = 0;  %time array initialize/reset
    a=0;    %acceleration array initialize/reset
    normal = 0; %propulsion normal force intialize/reset
    RPM = 0;    %RPM array initialize/reset
    c = 1;  %step counter initialize/reset
    s = 0; %step counter for secondary braking initialize/reset
    r=0;    %boolean for propulsion loop initialize/reset
    p=0; %boolean for secondary braking condition intialize/reset
       
    %Solving for static case of normal force
    static_coeff = [(x_CG-x_prop), -(x_stable-x_CG); 1, 1];
    static_rhs = [0;pM*9.81];
    static_ans = static_coeff\static_rhs;
    normal_static = static_ans(1);  %normal stability is other variable
    
    %Initialize acceleration vector    
    torque=90*gear_ratio;   %torque on the wheel initialize
%     if mu*normal_static > torque/((wD*0.0254)/2)
%             force_prop = torque/((wD*0.0254)/2) - rolling_drag;
%     else
%             force_prop = mu*normal_static - rolling_drag;       
%     end
    command_torque_open = 50;
    force_prop = command_torque_open/((wD*0.0254)/2) - rolling_drag;
    a(1)= force_prop/pM;%N for first time step of the propulsion phase
    normal(1) = normal_static;
    
    %Battery stuff
    current_voltage = 290;  %starting battery voltage
    accumulated_power = 0;
    power = zeros(25002);
    tt = linspace(0,25,25000); %create vector of 25000 evenly spaced points beween 0 and 25
    tt = [tt 25001 25002];  %adds 0 to tt vector
    current_capacity = 6; % 6 aH
    load_amps = 300; % countinous current required by the Motor
    ii = 2; %index variable for tt and power vectors

%% Propulsion Phase
    while r==0%propulsion phase loop
        c = c + 1;  %incriment of time
        v(c) = a(c-1)*dt + v(c-1);  %forward euler to determine next velocity value
        x(c) = v(c-1)*dt + x(c-1);    %forward euler to determine next position value
        t(c) = (c-1)*dt;    %time array incriment

        RPM=(v(c)*60)/(pi()*(wD*0.0254)); %RPM calculated
        RPM_motor=RPM*gear_ratio;   %seperate parameter if gear ratio is involved
        motor_torque=((1e-07*RPM_motor^2) + (-0.0019*RPM_motor) + 90);%torque at the motor
        torque= gear_ratio*motor_torque; %takes care of the torque lost with the RPM toque
        if mu*normal(c-1) > torque/((wD*0.0254)/2) 
            force_prop = torque/((wD*0.0254)/2)- (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
            mu_list(d) = mu; %make list of mu values that are useable
        else
            force_prop = mu*normal_static - (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;       
        end
        
        force_prop = command_torque_open/((wD*0.0254)/2) - (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
        a(c) = force_prop/pM;
        
        dynamic_coeff = [1, 1; x_CG-x_prop, -x_stable+x_CG];
        dynamic_rhs = [pM*9.81; force_prop*(y_CG-y_prop)];
        dynamic_ans = dynamic_coeff\dynamic_rhs;
        normal(c) = dynamic_ans(1); %propulsion normal force

        %Battery Stuff
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
        
        %Setup for off-nominal run
        if (((v(c))^2/(2*(-decel)))+(primary_delay+secondary_delay)*v(c))>=(trackLength - safetyDistance - x(c)) && p ~= 1
            v_secondary = v;
            x_secondary = x;
            t_secondary = t;
            s = c;
            t_prop_secondary = t(c);
            command_braking = x(c);
            p = 1; %skip condition
            %disp(x(c))
            %disp(mu)
        end
           
        %Propulsion phase end conditions
        if (((v(c))^2/(2*(-decel))) + (primary_delay+secondary_delay)*v(c))>=(trackLength - safetyDistance - x(c)) %calculating if the pod can stop in time at the current velocity
            vMax=v(c);  %max velocity to display in the results
            vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
            r=1;    %to move out of the propulsion phase
            v_secondary = v;
            x_secondary = x;
            t_secondary = t;
            s = c;
            command_braking = x(c)
            t_prop_secondary = t(c)
            End_mode={'No more track'}; %Reason the run ended
        end

        if RPM_motor>=Max_RPM   %checks if the Max RPM at the given battery voltage has been reached
            vMax=v(c);
            vMax_mph=convvel(v(c), 'm/s', 'mph');
            r=2;
            End_mode={'No more motor RPM'};
        end
        
         if t(c) > 10
            vMax=v(c);
            vMax_mph=convvel(v(c), 'm/s', 'mph');
            r = 3;
            v_secondary = v;
            x_secondary = x;
            t_secondary = t;
            s = c;
            command_braking = x(c)
            t_prop_secondary = t(c)
        end
        
    t_prop=t(c); %propulsion time
    end   
%% Braking Phase 
    %Primary braking case
    while v(c)>=0 %primary braking phase loop
        if t(c)<=(t_prop+primary_delay) %coasting case
            c = c + 1;
            v(c)=(v(c-1) - dt*rolling_drag/pM);
            x(c)=v(c-1)*dt + x(c-1);
            t(c)= c*dt;            
        else
            c = c + 1;
            v(c)=v(c-1)+decel*dt;
            x(c)=v(c-1)*dt + x(c-1);
            t(c)= c*dt;
        end
    end
    
    %Secondary braking case (off-nominal run)
    while v_secondary(s)>=0        
        if t_secondary(s)<=(t_prop_secondary+secondary_delay+primary_delay)
            s = s + 1;
            v_secondary(s)=(v_secondary(s-1) - dt*rolling_drag/pM);
            x_secondary(s)=v_secondary(s-1)*dt + x_secondary(s-1);
            t_secondary(s)= s*dt;
        else
            s = s + 1;
            v_secondary(s) = decel*dt + v_secondary(s-1);   %decel is the decelaration value for braking and is assumed constant
            x_secondary(s) = v_secondary(s-1)*dt + x_secondary(s-1);
            t_secondary(s) = s*dt;
        end    
    end
    
    %Storing data from each iteration (differing values of mu)
    full_acceleration{d} = a;
    full_velocity{d} = v;
    full_velocity_mph{d} = convvel(v,'m/s','mph');
    full_velocity_secondary{d} = v_secondary;
    full_velocity_secondary_mph{d} = convvel(v_secondary,'m/s','mph');
    full_position{d} = x;
    full_position_secondary{d} = x_secondary;
    full_time{d} = t;
    full_time_secondary{d} = t_secondary;
    full_normal{d} = normal;
    full_time_prop_secondary(d) = t_prop_secondary;
    full_brake_initialize_secondary(d) = command_braking;
    
    
    max_velocity(d,1) = mu;
    %max_velocity(d,2) = max(full_velocity_mph{d});
    max_velocity(d,2) = max(full_velocity_secondary_mph{d});
    d = d + 1;
    
end

%% Post Processing
array2table(max_velocity, 'VariableNames',{'Friction_Coefficient','Maximum_Velocity_Secondary_mph'})
min_req_mu = (torque_max/((wD*0.0254)/2)) / normal_static %min required mu for max speed
mu_actual = 0.4; 
force_prop_actual = mu_actual*normal_static;
command_torque = force_prop_actual*((wD*0.0254)/2);
req_brake_dist = convvel(max_velocity(1,2),'mph','m/s')^2 / (2*decel)
max_RPM_actual = convvel(max_velocity(1,2),'mph','m/s')/((wD*0.0254)/2) * 60/(2*pi())
force_prop_wheel = command_torque_open/((wD*0.0254)/2)

%% Run profile info
run_data = [command_torque_open,force_prop_wheel,max_RPM_actual,t_prop_secondary,max_velocity(1,2),command_braking,req_brake_dist,safetyDistance,trackLength]';
array2table(run_data,'RowNames',{'Command_Torque','Propulsion_wheel_force','Max_RPM','Propulsing_time','Max_velocity','Distance_to_command_braking','Required_braking_distance','Safety_distance','Track_length'})


%% Plotting
% Plot 1 - Velocity [mph] vs. Position [m] nominal run
figure
hold on
title('Velocity vs. Position - Nominal Run')
xlabel('Position [m]')
ylabel('Velocity [mph]')
ylim([0,185])
xlim([0,1600])
for k = 1:length(full_position)
    plot(full_position{k},full_velocity_mph{k})
end
legendCell = cellstr(num2str(mu_range', 'mu=%-5.2f'));
legend(legendCell)
hold off

%Plot 2 - Velocity [mph] vs. Position [m] off nominal run
figure
title('Velocity vs. Position - Off Nominal Run')
hold on
xlabel('Position [m]')
ylabel('Velocity [mph]')
ylim([0,185])
xlim([0,1600])
for k = 1:length(full_position_secondary)
    plot(full_position_secondary{k},full_velocity_secondary_mph{k})
end
legendCell = cellstr(num2str(mu_range', 'mu=%-5.2f'));
legend(legendCell)
hold off














% 
% xMax = x(c);    %Distance the pod travelled by the end of the run
% tMax = t(c);    %total time taken
% dat=[gear_ratio, wD, vMax, vMax_mph,t_prop, End_mode, tMax, xMax,]; %output array
% data=array2table(dat());    %converted to a table
% T=vertcat(T,data);  %the values converted to the table are vertically concatinated into a master table T
% 
% 
% 
% %post process
% T.Properties.VariableNames={'Gear_Ratio','Wheel_diameter','Velocity_max','Velocity_max_mph','Time_propulsion','End_Mode','Run_Time_s', 'Distance_travelled',};
% T=sortrows(T,4,'descend')%to display the top3 results of the different wheel diameter
% 
% %plot of winning run
% %the code runs the same as above just needs to plot the values of the fastest run as the run values aren't saved in each iteration
% 
% wD=final_wheel_decision;
% 
% 
% 
% % conditions
% dt = 0.001;
% v = 0;
% x = 0;
% t = 0;
% a=0;
% RPM = 0;
% c = 1;
% r=0;
% current_voltage=0;
% current_voltage(c) = 290;
% accumulated_power = 0;
% power = zeros(25002);
% tt = linspace(0,25,25000); %create vector of 25000 evenly spaced points beween 0 and 25
% tt = [tt 25001 25002];  %adds 0 to tt vector
% current_capacity = 6; % 6 aH
% load_amps = 300; % countinous current required by the Motor
% ii = 1; %index variable for tt and power vectors
% r_prop=convlength(wD/2,'in','m');
% minertia_prop= (r_prop)^2 *(0.4468*wD-0.9262)*1/2 ; %modelled as a solid disk
% pM=135.72; %pod mass without propulsion wheel
% pM=pM+0.4468*wD-0.9262; %pM with prop wheel weight
% m_rot=num_lat*(minertia_lat/r_lat^2)+num_vert*(minertia_vert/r_vert^2)+num_prop*(minertia_prop/r_prop^2);
% pM=pM+m_rot; % pM is equivalent with wheel weight now
% 
% 
% torque=90*gear_ratio;
% force_prop= torque/((wD*0.0254)/2)- (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
% a(1)= force_prop/pM;%N
% r_prop=convlength(wD/2,'in','m');
% t_func(1)=(force_prop*r_prop)*8.85;%torque over time this is a derived value only in-lbs for the dynamic model
% while r==0%prop loop
%     c = c + 1;
%     v(c) = a(c-1)*dt + v(c-1);  %forward euler to determine next velocity value
%     x(c) = v(c-1)*dt + x(c-1);    %forward euler to determine next position value
%     t(c) = (c-1)*dt;    %time array incriment
%     
%     RPM=(v(c)*60)/(pi()*(wD*0.0254)) ;
%     RPM_motor=RPM*gear_ratio;
%     motor_torque=((1e-07*RPM_motor^2) + (-0.0019*RPM_motor) + 90);%torque at the motor
%     torque=gear_ratio*motor_torque; %takes care of the torque lost with the RPM toque
%     force_prop= torque/((wD*0.0254)/2)- (0.5 *CD*air_rho*Area_pod*(v(c))^2)-rolling_drag;
%     a(c)= force_prop/pM;
%     t_func(c)=(force_prop*r_prop)*8.85;%torque over time this is a derived value only in-lbs for the dynamic model
%     
%     
%     CurrentPower(c) = (motor_torque*2*pi() * RPM_motor)/60; % Watts
%     load_amps = CurrentPower(c-1) / current_voltage(c-1); % amps required for desired power
%     charge_used = load_amps * (dt / 3600); %amp hrs used in single iteration
%     current_capacity(c) =  current_capacity(c-1) - charge_used; %subtracts charge used from current capacity
%     accumulated_power = accumulated_power + ((get_voltage(load_amps,current_capacity(c)) * load_amps) / 1000); %for each iteration, calculates power, sums up
%     current_voltage(c) = get_voltage(load_amps, current_capacity(c)); % new current voltage
%     power(ii) = accumulated_power;
%     
%     ii = ii + 1; %increments index of ii
%     
%     Max_RPM = 24 * current_voltage; % 24 is slope of line of (Power/RPM), pretty linear
%     
%     
%     if RPM_motor>=Max_RPM
%         vMax=v(c);
%         vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
%         r=1;
%         
%     end
%     if (trackLength - safetyDistance - x(c))<=(v(c)^2/(2*-decel))
%         vMax=v(c);
%         vMax_mph=convvel(v(c), 'm/s', 'mph');   %mph of the result
%         r=2;
%         
%     end
%     t_p_time(c)=t(c);%time step array of the propulsion phase to help make the voltage vs time plot
%     t_p_max=t_p_time(c);
% end
% 
% braking_dist=(v(c)^2/(2*-decel));   %m total distance the braking phase takes
% s=c;
% plotlim=c;
% 
% v_secondary=v;
% x_secondary=x;
% t_secondary=t_p_time;
% while v(c)>=0
%     if t(c)<=(t_p_max+primary_delay)
%         c = c + 1;
%         v(c)=(v(c-1)-dt*rolling_drag/pM);
%         x(c)=v(c-1)*dt + x(c-1);
%         t(c)= c*dt;
%     end
%     if t(c)>(t_p_max+primary_delay)
%         c = c + 1;
%         v(c)=v(c-1)+decel*dt;
%         x(c)=v(c-1)*dt + x(c-1);
%         t(c)= c*dt;
%         
%     end
%     
% end
% while v_secondary(s)>=0
%     if t_secondary(s)<=(t_p_max+secondary_delay)
%         s = s + 1;
%         v_secondary(s)=(v_secondary(s-1)-dt*rolling_drag/pM);
%         x_secondary(s)=v_secondary(s-1)*dt + x_secondary(s-1);
%         t_secondary(s)= s*dt;
%     end
%     
%     if t_secondary(s)>(t_p_max+secondary_delay)
%         s = s + 1;
%         v_secondary(s) = decel*dt + v_secondary(s-1);   %decel is the decelaration value for braking and is assumed constant
%         x_secondary(s) = v_secondary(s-1)*dt + x_secondary(s-1);
%         t_secondary(s) = s*dt;
%     end
%     
%     
% end
% 
% %plots
% figure(1)
% plot(x,v,'k')   %plot displacement vs velocity
% xlabel('Distance[m]')
% ylabel('Velocity[m/s]')
% hold on
% plot(x_secondary(plotlim:s),v_secondary(plotlim:s),'--r')
% legend('Nominal','Off-Nominal')
% 
% ylim([0,90])
% hold off
% figure(2)
% plot(t,x,'k')   %plot displacemnt vs time
% ylabel('Distance[m]')
% xlabel('time[s]')
% hold on
% plot(t_secondary(plotlim:s),x_secondary(plotlim:s),'--r')
% legend('Nominal','Off-Nominal')
% 
% hold off
% figure(3)
% plot(t,v,'k')   %plot velocity vs time
% xlabel('time[s]')
% ylabel('Velocity[m/s]')
% hold on
% plot(t_secondary(plotlim:s),v_secondary(plotlim:s),'--r')
% legend('Nominal','Off-Nominal')
% 
% ylim([0,90])
% hold off
% figure(4)
% plot(t_p_time,current_voltage,'k'); %plot battry pack voltage vs time(till end of propulsion phase)
% xlabel('time[s]');
% ylabel('voltage[V]');
% 
% figure(5)
% plot(t_p_time,current_capacity,'k'); 
% xlabel('time[s]');
% ylabel('Capacity(aH)');
% 
% figure(6)
% plot(t_p_time,CurrentPower,'k'); 
% xlabel('time[s]');
% ylabel('Power[W]');
