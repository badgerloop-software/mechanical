%%% Pod Pitch Calcs V1
%Last edited 8/5/19 by Nathan Berg
clear
clc
close all

%% Constants
pM =375/2.204; %kg - updated 7/3/19
k_spring = 60*175.127; %spring constant - lbf/in to N/m
b_spring = 442*4.448/(.0254*12); %damping coefficient - lbf-s/ft to N-s/m
rolling_drag=30;%N - updated 7/19 from approx 4Nm torque requred to move pod
air_rho=1.184; %kg/m^3 density of air at 14.7psi 77F 50%RH 
CD=0.19; %pod 4 (10/14) 
Area_pod=0.28;%m^2 pod 4 (10/14)
Max_RPM=6750; %max motor RPM
secondary_delay=1;%time mesaured in seconds from when primary brakes are commanded takes into account waiting time on the microcontroller plus the time to actuate secondary
primary_delay=0.25;%time mesaured in seconds from when primary brakes are commanded takes into account lag within the braking system itself

%Wheel parameters
num_vert= 2 ; % # of vertical wheels in the stability system of the same size
num_lat= 4 ;%# of lateral wheels of the same size
num_prop= 1 ;% # of propulsion wheels
minertia_lat=8.92e-5 ; % mass moment of inertia derived value from CAD in kg-m^2 
minertia_vert=8.92e-5  ; % same as above
r_lat=convlength(1.5,'in','m'); %pod 4 wheel size convlength(radius,'unit input','unit desired')
r_vert=convlength(1.5,'in','m'); %pod 4
wD = convlength(10.45,'in','m'); %meters - the upper and lower bound was set to this after results convereged within this range
r_prop= wD/2; %meters - radius prop wheel with coating pod 4
minertia_prop= (r_prop)^2 *(0.4468*(wD/.0254)-0.9262)*1/2 ; %modelled as a solid disk - moment of inertia vs. radius. this is equation of that curve
m_rot=num_lat*(minertia_lat/r_lat^2)+num_vert*(minertia_vert/r_vert^2)+num_prop*(minertia_prop/r_prop^2);%converting over the wheel interia to an equivalent mass
%Don't use here since doing CG related analysis. %pM=pM+m_rot; % pM is equivalent with wheel weight now of rotating masses

%Pod dimensions - ALL FROM BACK OF POD AND TAKING TOP SUBTRACK GROUND (DATUM). x dir along axis of pod, y dir is vertical
full_length = convlength(88,'in','m'); %total length of pod 4
x_stable = convlength(63.5,'in','m'); %distance to front vertical stability
x_prop = convlength(19.5,'in','m'); %19.5 distance to propulsion wheel
x_CG = convlength(39,'in','m'); %distance to pod 4 CG
y_stable = 0; %height of stability wheel contact
y_prop = convlength(4.5,'in','m'); %4.5 height propulsion wheel contact
y_CG = convlength(8.5,'in','m'); %height of center of gravity

%Deflection Calculation Dimensions - refer to James' document for visual. x dir along axis of pod, z dir is vertical
x_1 = convlength(3.954,'in','m'); %x distance from stability structure mount to stability rod
x_2 = convlength(3.076,'in','m'); %x distance from propulsion stability rod to propulsion pivot
x_3 = convlength(6.993,'in','m'); %x distance from propulsion pivot to prop wheel center
z_1 = convlength(3.992,'in','m'); %3.992z distance from propulsion pivot to wheel contact point
z_2 = convlength(6.210,'in','m'); %z distance from propulsion pivot to stability rod
L = convlength(5.93,'in','m'); %Distance from propulsion pivot to springs - perpendicular to spring axis
alpha = 57.51; %angle of prop stability to surface of channel - deg
MR = 1/0.92; %motion ratio - wheel displacement/spring displacement

%NOTE: I think static deflection could be used to verify preload in some way from taking it at unloaded to static load

%% Braking deceleration calculations - currently not using this info
% pressure = 140; %psi
% F_N=convforce(4*pi()*pressure,'lbf','N') - convforce(100,'lbf','N'); %F_N is the normal force put out by the actuator. pi*psi
% cof=0.3; %coefficient of friction depending on the brake pad used
% theta = 40; %[deg] - angle made when actuated
% 
% %Force balance 
% braking_coeff = [-1,cof,0;-1,0,cosd(theta);0,1,sind(theta)];
% braking_rhs = [0;0;-F_N];
% braking_ans = braking_coeff\braking_rhs; %[F_f;F_R;F_j]
% 
% F_f=braking_ans(1); %F_f is friction force
% F_j = braking_ans(3); %F_j is the force in the linkage. x direction balance
% F_R = braking_ans(2); %y direction balance
% decel = F_f/pM; 
% G_s_decel = decel/9.806; %g force calc
%decel = 1.415*-9.806; % g's of braking - old value

%% Stability/Propulsion Calculations
%Full pod static analysis
static_coeff = [(x_CG-x_prop), -(x_stable-x_CG); 1, 1];
static_rhs = [0;pM*9.81];
static_ans = static_coeff\static_rhs;
normal_static = static_ans(1);  %normal front stability is other variable

%Setup for loop
i = 1; %counter
command_torque_initial = [0:210]; %all torques to calculate for in loop
single_torque_value = 50; %Value to have the table with all relevant values generated for

for command_torque = command_torque_initial %50 is single value
    %Pod acceleration calculations
    force_prop = command_torque/(wD/2);
    a = force_prop/pM;
    a_Gs = a/9.81;

    %Solving dynamic situation
    dynamic_coeff = [1, 1; x_CG-x_prop, -x_stable+x_CG];
    dynamic_rhs = [pM*9.81; force_prop*(y_CG-y_prop)];
    dynamic_ans = dynamic_coeff\dynamic_rhs;
    normal_dynamic = dynamic_ans(1); %propulsion normal force

    %% Solving Deflections - Pod 4 Setup
    %Static deflection
    f_s_4_static = (normal_static*x_3)/(2*L); %static spring force of 1 spring
    delta_spring_4_static = f_s_4_static/k_spring; %static compression of springs
    delta_wheel_4_static = delta_spring_4_static*MR; %wheel movement from static loading
    k_eff_static = normal_static/delta_wheel_4_static; %effective stiffness (1 spring right above wheel)

    %Dynamic deflection
    f_s_4_dynamic = ((normal_dynamic*x_3)+(force_prop*z_1))/(L*2); %From moment balance - 2 included since 2 springs
    delta_spring_4_dynamic = f_s_4_dynamic/k_spring; %deflection of spring
    delta_wheel_4_dynamic = delta_spring_4_dynamic*MR; %vertical movement of wheel from spring displacement
    pitch_angle_4 = atand(delta_wheel_4_dynamic/(x_CG-x_prop)); %pitch angle calculated from x location of CG
    k_eff_dynamic = normal_dynamic/delta_wheel_4_dynamic; %k-eff of dynamic situaiton

    %% Solving Deflections - Anti-Squat Geometry
    %Assume static case is still the same if the wheel is located in the same place
    %Just need to flip sign on propulsion force

    %Dynamic deflection
    f_s_as = ((normal_dynamic*x_3)-(force_prop*z_1))/(L*2); %From moment balance - 2 included since 2 springs
    delta_spring_as = f_s_as/k_spring; %deflection of spring
    delta_wheel_as = delta_spring_as*MR; %vertical movement of wheel from spring displacement
    pitch_angle_as = atand(delta_wheel_as/(x_CG-x_prop)); %pitch angle calculated from x location of CG
    k_eff_dynamic_as = normal_dynamic/delta_wheel_as; %k-eff of anti-squat setup
    
    %% Single torque table
    if command_torque == single_torque_value %Single torque value in Nm for table generation
        
        %Unit Conversions - single torque
        forces_met = [f_s_4_dynamic,f_s_as,f_s_4_static];
        forces_eng = convforce(forces_met,'N','lbf');
        lengths_met = [delta_spring_4_dynamic,delta_spring_as,delta_spring_4_static,delta_wheel_4_dynamic,delta_wheel_as,delta_wheel_4_static];
        lengths_eng = convlength(lengths_met,'m','in');
        k_eff_met_single = [k_eff_dynamic,k_eff_dynamic_as,k_eff_static];
        k_eff_eng_single = k_eff_met_single./175.126; %N/m to lbf/in
        
        %Compile Results - single torque
        data = [forces_eng(1)*2,2*forces_eng(2),2*forces_eng(3);lengths_eng(1),lengths_eng(2),lengths_eng(3);lengths_eng(4),lengths_eng(5),lengths_eng(6);k_eff_eng_single(1),k_eff_eng_single(2),k_eff_eng_single(3);pitch_angle_4,pitch_angle_as,0;command_torque,command_torque,0];
        array2table(data,'VariableNames',{'Current_pod_4_setup','Anti_squat_setup','Static'},'RowNames',{'Total_Spring_Force','Spring_Movement','Wheel_Movement','Effective_Stiffness','Pitch_Angle','Command_Torque_Nm'})
        disp('All english units unless otherwise noted')
    end
    
    %% Compile Results - multiple torques (for loop)    
    k_eff_met(1,i) = k_eff_dynamic;
    k_eff_met(2,i) = k_eff_dynamic_as;
    k_eff_met(3,i) = k_eff_static;
    i = i + 1;
end

%Unit Conversion - multiple torques
k_eff_eng = k_eff_met./175.126; %N/m to lbf/in

%% Plot
figure
plot(command_torque_initial,k_eff_eng(3,:))
title('K-eff Graph')
xlabel('Command Torque [N*m]')
ylabel('Effective Stiffness [lbf/in]')
hold on
plot(command_torque_initial,k_eff_eng(1,:))
plot(command_torque_initial,k_eff_eng(2,:))
legend('Static Case','Pod 4 Dynamic Case','Anti-Squat Dynamic Case','Location','NorthWest')
