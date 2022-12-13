function test_coriolis_matrix()
addpath('autogen')
path_to_urdf = 'iiwa14.urdf';
kuka = parse_urdf(path_to_urdf);

rbt = importrobot('iiwa14.urdf');
rbt.DataFormat = 'column';
rbt.Gravity = [0 0 -9.81];

num_iter = 100;
log_tau_matlab = [];log_tau_manip = [];
for i = 1:num_iter
    q = -2*pi + 4*pi*rand(7,1);
    q_d = 0.2*pi*rand(7,1);
    q_2d = zeros(7,1);
    
% verifying coriolis     
    tau_matlab = velocityProduct(rbt,q,q_d);
    tau_manip = C_mtrx_fcn(q, q_d, kuka.pi(:))*q_d;
    log_tau_matlab = [log_tau_matlab,tau_matlab];
    log_tau_manip = [log_tau_manip,tau_manip];

    %   verifying if regressor is computed correctly
    assert(norm(tau_matlab - tau_manip) < 1e-8);

end

fprintf("Rigid Body Coriolis Matrix Test - OK!\n");