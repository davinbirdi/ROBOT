%Performs inverse kinematics for the robot

clear;
clc;

i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

%function returning skew symmetric matrix representation of a 3x1 vector s
skew = @(s) [0 -s(3,1) s(2,1); ...
            s(3,1) 0 -s(1,1); ...
            -s(2,1) s(1,1) 0];

%todo: replace with user input
thetaInput = rand*pi; %random value from 0 to pi
sInput = rand(3,1); %3x1 vector of random values
sInput = sInput./norm(sInput); %normalize sInput

%desired position and orientation of platform
C10 = [expm(thetaInput*skew(sInput))*i expm(thetaInput*skew(sInput))*j expm(thetaInput*skew(sInput))*k];
d10 = [0; 0; -5*rand];

%parameters
lp = 0.2; %scaling factor for p, describing platform size
lb = 1; %scaling factor for b, desciribng base size
l = 1.5; %length of arm
dguess = 1; %initial guess for d_i (joint variable) when calling fsolve

%columns are p1, p2, p3 representing platform corner positions wrt
%platform frame
p = lp*[0 -sqrt(3)/2 sqrt(3)/2; ...
        1 -1/2       -1/2; ...
        0 0          0];

%columns are b1, b2, b3 representing base corner positions wrt
%platform frame
b = lb*[0 -sqrt(3)/2 sqrt(3)/2; ...
        1 -1/2       -1/2; ...
        0 0          0];

for idx = 1:3 %iterate over joints
    
    %function that should equal 0 when the input variable d_i (ith joint
    %variable) solves the inverse kinematics
    func = @(x) norm(d10 - b(:,idx) + x*k + C10*p(:,idx)) - l;
    
    %solve for d_i. Asumes that func is monotonic wrt x
    d(idx) = fsolve(func, dguess,optimoptions('fsolve','Display','none'));
    
    %DEBUGGING ONLY:
    %plot func. Demonstrates that fsolve can return two possible solutions
    %Need to do further analysis to determine better way of solving inverse kinematics
    x = linspace(-2,10,100);
    for i = 1:max(size(x))
        y(i) = func(x(i)) + l;
    end
    subplot(3,1,idx),plot(x,y);
    hold on;
    plot([x(1) x(end)], [l l]);
    title(['norm(p_' num2str(idx) '-m_' num2str(idx) ') vs d_' num2str(idx)]);
    legend('norm(p-m)','l');
    set(gca,'FontSize',15);
    grid on;
    hold off;
    %END DEBUGGING ONLY
end

%display results
platformPosition = d10
platformRotation = thetaInput*sInput
solution = d