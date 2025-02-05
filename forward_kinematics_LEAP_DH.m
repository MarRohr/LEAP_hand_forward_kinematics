%% Notes: 
%  The order of the fingers corresponds to the URDF.
%  The order of the real hand is different.
%
function EE = forward_kinematics_LEAP_DH(q) 

    %% hand parameters 
    % lengths fingers
    l1 = 0.038; 
    l2 = 0.015;
    l3 = 0.036;
    l4 = 0.050; %Fingertip-specific

    % lengths thumb
    l2t = 0.032;
    l3t = 0.047;
    l4t = 0.062; %Fingertip-specific

    %% DH parameters
    % index finger
    a1_1 = l1;
    d1_1 = 0;
    alpha1_1 = pi/2;
    theta1_1 = q(1);
    
    a2_1 = l2;
    d2_1 = 0;
    alpha2_1 = pi/2;
    theta2_1 = -q(2);
    
    a3_1 = l3;
    d3_1 = 0;
    alpha3_1 = 0;
    theta3_1 = -q(3);
    
    a4_1 = l4;
    d4_1 = 0;
    alpha4_1 = 0;
    theta4_1 = -q(4);

    % middle finger
    a1_2 = l1;
    d1_2 = 0;
    alpha1_2 = pi/2;
    theta1_2 = q(9);
    
    a2_2 = l2;
    d2_2 = 0;
    alpha2_2 = pi/2;
    theta2_2 = -q(10);
    
    a3_2 = l3;
    d3_2 = 0;
    alpha3_2 = 0;
    theta3_2 = -q(11);
    
    a4_2 = l4;
    d4_2 = 0;
    alpha4_2 = 0;
    theta4_2 = -q(12);

    % pinky
    a1_3 = l1;
    d1_3 = 0;
    alpha1_3 = pi/2;
    theta1_3 = q(13);
    
    a2_3 = l2;
    d2_3 = 0;
    alpha2_3 = pi/2;
    theta2_3 = -q(14);
    
    a3_3 = l3;
    d3_3 = 0;
    alpha3_3 = 0;
    theta3_3 = -q(15);
    
    a4_3 = l4;
    d4_3 = 0;
    alpha4_3 = 0;
    theta4_3 = -q(16);

    % thumb
    a1_t = 0;
    d1_t = 0;
    alpha1_t = pi/2;
    theta1_t = q(5)+pi/2;
    
    a2_t = 0;
    d2_t = l2t;
    alpha2_t = pi/2;
    theta2_t = q(6)+pi+pi/2;
    
    a3_t = l3t;
    d3_t = 0;
    alpha3_t = 0;
    theta3_t = -q(7)+pi/2;
    
    a4_t = l4t;
    d4_t = 0;
    alpha4_t = 0;
    theta4_t = -q(8);
    
    %% transformations
    % finger 1
    T1_1 = DHTransform(a1_1, alpha1_1, d1_1, theta1_1);
    T2_1 = DHTransform(a2_1, alpha2_1, d2_1, theta2_1);
    T3_1 = DHTransform(a3_1, alpha3_1, d3_1, theta3_1);
    T4_1 = DHTransform(a4_1, alpha4_1, d4_1, theta4_1);

    % finger 2
    T1_2 = DHTransform(a1_2, alpha1_2, d1_2, theta1_2);
    T2_2 = DHTransform(a2_2, alpha2_2, d2_2, theta2_2);
    T3_2 = DHTransform(a3_2, alpha3_2, d3_2, theta3_2);
    T4_2 = DHTransform(a4_2, alpha4_2, d4_2, theta4_2);

    % finger 3
    T1_3 = DHTransform(a1_3, alpha1_3, d1_3, theta1_3);
    T2_3 = DHTransform(a2_3, alpha2_3, d2_3, theta2_3);
    T3_3 = DHTransform(a3_3, alpha3_3, d3_3, theta3_3);
    T4_3 = DHTransform(a4_3, alpha4_3, d4_3, theta4_3);

    % thumb
    T1_t = DHTransform(a1_t, alpha1_t, d1_t, theta1_t);
    T2_t = DHTransform(a2_t, alpha2_t, d2_t, theta2_t);
    T3_t = DHTransform(a3_t, alpha3_t, d3_t, theta3_t);
    T4_t = DHTransform(a4_t, alpha4_t, d4_t, theta4_t);
    
    %% finger bases
    % finger 1
    T01_1 = [  1,0,0,-0.0071;...
               0,0,1,0.0231-0.0145;...
               0,-1,0,-0.0187;...
               0,0,0,1];

    % finger 2
    T01_2 = [  1,0,0,-0.0071;...
               0,0,1,0.0231-0.0145-0.045;...
               0,-1,0,-0.0187;...
               0,0,0,1];

    % finger 3
    T01_3 = [  1,0,0,-0.0071;...
               0,0,1,0.0231-0.0145-0.045-0.045;...
               0,-1,0,-0.0187;...
               0,0,0,1];

    % thumb
    T01_t = [  0,0,-1,-0.0694-0.013;...
               1,0,0,-0.0012;...
               0,-1,0, -0.0216;...
               0,0,0,1];

    %% fintertip positions
    ftip_1 = T01_1*T1_1*T2_1*T3_1*T4_1;
    ftip_2 = T01_2*T1_2*T2_2*T3_2*T4_2;
    ftip_3 = T01_3*T1_3*T2_3*T3_3*T4_3;
    ftip_t = T01_t*T1_t*T2_t*T3_t*T4_t;

    EE = {ftip_1, ftip_2, ftip_3, ftip_t};
end

function T = DHTransform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end