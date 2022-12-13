function robot = parse_urdf(file)
% Loading file from urdf
% The function is tailored for kuka robots: serial robots with 7 DOF
% Modify it if you have  a robot with different degrees of freedom
robot = xml2struct(file);

% Extracting parameters of the robot
for i = 2:8 % the 1st joint is world_iiwa_joint
    % axis of rotation of a joint i in coordinate system of joint i    
    axis_of_rot = str2num(robot.robot.joint{i}.axis.Attributes.xyz)';
    % mass of link (i+1) because joint i rotates link (i+1) as the numbering of
    % links starts from base link that it not moving
    link_mass = str2double(robot.robot.link{i+1}.inertial.mass.Attributes.value);
    % poistion of the com in frame attached to link
    com_pos = str2num(robot.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    com_vec2mat = vec2skewSymMat(com_pos);
    % inertial parameters of the link expressed in coordinate system attached
    % the center of mass.
    ixx = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixx);
    ixy = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixy);
    ixz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixz);
    iyy = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.iyy);
    iyz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.iyz);
    izz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.izz);
    % the inertia tensor wrt the frame oriented as the body frame and with the
    % origin in the COM
    link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
    % manipulator regressor
    j = i-1;% the jth joint
    robot.m(j) = link_mass;
    robot.k(:,j) = axis_of_rot;
    robot.r_com(:,j) = com_pos;
    robot.I(:,:,j) = link_inertia;
    robot.h(:,j) = link_mass*com_pos;
    robot.I_vec(:,j) = inertiaMatrix2Vector(link_inertia-...
                            link_mass*com_vec2mat*com_vec2mat);
    robot.pi(:,j) = [robot.I_vec(:,j); robot.h(:,j); robot.m(j)];
end