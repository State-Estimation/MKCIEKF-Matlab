clc;
clear;

%��������������
gyro_noise_std = 0.002*ones(3,1); 
gyro_bias_noise_std = 0.001*ones(3,1);

%���ü��ٶȼ�����
accel_noise_std = 0.04*ones(3,1); 
accel_bias_noise_std = 0.001*ones(3,1);

%���ýӴ������ͱ���������
contact_noise_std = 0.09*ones(3,1);
encoder_noise_std = deg2rad(0.5)*ones(3,1);

base_pose_std = [0.01*ones(3,1); 0.01*ones(3,1)]; 
base_velocity_std = 0.1*ones(3,1);
contact_position_std = 1.0*ones(3,1);
gyro_bias_std = 0.01*ones(3,1);
accel_bias_std = 0.1*ones(3,1);
forward_kinematics_std = 0.03*ones(3,1);

%��ʼ��Э�������
Qg = diag(gyro_noise_std.^2);          %���ٶ�Э�������
Qbg = diag(gyro_bias_noise_std.^2);    %���ٶ�ƫ��Э�������
Qa = diag(accel_noise_std.^2);         %���ٶ�Э�������
Qba = diag(accel_bias_noise_std.^2);   %���ٶ�ƫ��Э�������
Qco = diag(contact_noise_std.^2);      %�Ӵ�Э�������
Qe = diag(encoder_noise_std.^2);       %������Э�������
Np = diag(forward_kinematics_std.^2);  %ǰ���˶�ѧЭ�������

%��������
load('data\ground_truth_data.mat');
load('data\imu_data.mat');
load('data\leg_data.mat');

%��ȡimu�����ļ��ٶȣ����ٶ���Ϣ
acce_ = transpose(imu_data{:, 2:4});
gyro_ = transpose(imu_data{:, 5:7});

%��ȡλ�á��ٶȡ��������ʵ��Ϣ
position_true = transpose(ground_truth_data{:, 2:4});
velocity_true = transpose(ground_truth_data{:, 9:11});
R_true = transpose(ground_truth_data{:, 5:8});

%��ʼ����λ�á��ٶȡ�imuƫ��������ٶ�
a_R = quaternionToRotationMatrix(ground_truth_data{1, 5:8}); %��ó�ʼ��̬����
p_ = transpose((ground_truth_data{1, 2:4}))+[0;0;0.355];   %��ʼλ��
v_ = [0; 0; 0];
R_ = a_R;
ba_ = [1; 1; 1] * 3.552e-04;
bg_ = [1; 1; 1] * 1.655e-05;
gravity = [0; 0; -9.8];

%���ò�������Ͳ���ʱ��
dt = 0.001;
end_time =65 ;
time = 0:dt:end_time;
num_steps = length(time);

%�洢�˲������λ�á��ٶȡ�������Ϣ
p_history = zeros(3, num_steps);
v_history = zeros(3, num_steps);
R_history = zeros(3, num_steps);
o_history = zeros(3, num_steps);
q_history = zeros(4, num_steps);
o_true = zeros(3, num_steps);

%��ò���ʱ���ڵ�λ�á��ٶȡ��������ʵ��Ϣ
position_true_resampled = position_true(:, 1:num_steps);
velocity_true_resampled = velocity_true(:, 1:num_steps);
R_true_resampled = R_true(:, 1:num_steps);

%��ò�������bodyΪ����ϵ������λ�á��ٶ��Լ����������ĽӴ���Ϣ
d0_measure_pos = transpose(leg_data{1:num_steps, 2:4});
d1_measure_pos = transpose(leg_data{1:num_steps, 12:14});
d2_measure_pos = transpose(leg_data{1:num_steps, 22:24});
d3_measure_pos = transpose(leg_data{1:num_steps, 32:34});

d0_measure_speed = transpose(leg_data{1:num_steps, 5:7});
d1_measure_speed = transpose(leg_data{1:num_steps, 15:17});
d2_measure_speed = transpose(leg_data{1:num_steps, 25:27});
d3_measure_speed = transpose(leg_data{1:num_steps, 35:37});

contact_0 = transpose(leg_data{1:num_steps, 11});
contact_1 = transpose(leg_data{1:num_steps, 21});
contact_2 = transpose(leg_data{1:num_steps, 31});
contact_3 = transpose(leg_data{1:num_steps, 41});

%��ʼ����λ��
d0 = [0; 0; 0];
d1 = [0; 0; 0];
d2 = [0; 0; 0];
d3 = [0; 0; 0];

%��ʼ״̬ת��Э�������
P = blkdiag(diag(base_pose_std(1:3).^2), ...
                  diag(base_velocity_std.^2), ...
                  diag(base_pose_std(4:6).^2), ...
                  diag(contact_position_std.^2), ...
                  diag(contact_position_std.^2), ...
                  diag(contact_position_std.^2), ...
                  diag(contact_position_std.^2), ...
                  diag(gyro_bias_std.^2),...
                  diag(accel_bias_std.^2));

%�����˲�
for i = 1:num_steps

    %��һ���˲�ʱʹ�ó�ʼ��������״̬������Ԥ��͸���
    if i == 1
        [X, theta] = ConstructState(R_, v_, p_, d0, d1, d2, d3, bg_, ba_);
        [X_pred, theta_pred, P_pred] = PredictState(X, theta, P, gyro_, acce_, dt, gravity, i, contact_0, contact_1, contact_2, contact_3, Qg, Qa, Qco, Qbg, Qba);
        [X_update, theta_update, P_update] = UpdateState_forwardkinematics(X_pred, theta_pred, P_pred, contact_0, contact_1, contact_2, contact_3, d0_measure_pos, d1_measure_pos, d2_measure_pos, d3_measure_pos, i, Qe, Np);
        
        %���˲�����洢��������
        p_history(:, i) = X_update(1:3, 5) + [0;0;0.0];
        v_history(:, i) = X_update(1:3, 4);
        o_history(:, i) = RotationMatrixToEulerAngles(X_update(1:3, 1:3));
        q_history(:, i) = rotationMatrixToQuaternion(X_update(1:3, 1:3));
        o_true(:, i) = QuaternionToEulerAngles(R_true_resampled(1:4, i));

    %֮��ÿ�μ�ʹ���ϴθ��¹��̺�õ���״̬��Ϊ�����˲��ĳ�ʼ״̬
    else
        X = X_update;
        theta = theta_update;
        P = P_update;
        [X_pred, theta_pred, P_pred] = PredictState(X, theta, P, gyro_, acce_, dt, gravity, i, contact_0, contact_1, contact_2, contact_3, Qg, Qa, Qco, Qbg, Qba);
        [X_update, theta_update, P_update] = UpdateState_forwardkinematics(X_pred, theta_pred, P_pred, contact_0, contact_1, contact_2, contact_3, d0_measure_pos, d1_measure_pos, d2_measure_pos, d3_measure_pos, i, Qe, Np);
        
        %���˲�����洢��������
        p_history(:, i) = X_update(1:3, 5) + [0;0;0.0];
        v_history(:, i) = X_update(1:3, 4);
        o_history(:, i) = RotationMatrixToEulerAngles(X_update(1:3, 1:3));
        q_history(:, i) = rotationMatrixToQuaternion(X_update(1:3, 1:3)); 
        o_true(:, i) = QuaternionToEulerAngles(R_true_resampled(1:4, i));

    end
end
for i = 1:num_steps
    position_true_resampled(3,i) = position_true_resampled(3,i)+0.355;
end
%%�����˲��������ʵ�����λ�á����򡢶�ά�켣�Ա�ͼ
plot_results(time, p_history, position_true_resampled, o_history, o_true);

%��Ҫ��ӡ������
outcome_iekf = transpose([time;p_history(:,:);q_history(:,:)]);
% �� outcome_only_imu ����Ϊ MAT �ļ�
save('IEKF.mat', 'outcome_iekf');

% �� outcome_eskf ����Ϊ TXT �ļ�
writematrix(outcome_iekf, 'IEKF', 'Delimiter', ' ')

%״̬��Ԥ�����
function [X_pred, theta_pred, P_pred] = PredictState(X, theta, P, gyro_, acce_, dt, gravity, i, contact_0, contact_1, contact_2, contact_3, Qg, Qa, Qco, Qbg, Qba)

    [R_, v_, p_, d0, d1, d2, d3, bg_, ba_] = SeperateState(X, theta);
    R_pred = R_ * expm(skew((gyro_(1:3, i) - bg_) * dt));
    v_pred = v_ + gravity * dt + R_ * (acce_(1:3, i) - ba_) * dt;
    p_pred = p_ + v_ * dt + 0.5 * gravity * dt + 0.5 * R_ * (acce_(1:3, i) - ba_) * dt * dt;
    bg_pred = bg_;
    ba_pred = ba_;
    Ac = [     zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
          skew(gravity), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
               zeros(3),   eye(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
               zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
               zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
               zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
               zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3), zeros(3)]; 
    Ac = blkdiag(Ac, zeros(6));
    Ac(1:end-6, end-5:end) = [         -R_, zeros(3);
                              -skew(v_)*R_,      -R_;
                              -skew(p_)*R_, zeros(3);
                              -skew(d0)*R_, zeros(3);
                              -skew(d1)*R_, zeros(3);
                              -skew(d2)*R_, zeros(3);
                              -skew(d3)*R_, zeros(3)];
    Ak = eye(size(Ac)) + Ac * dt;
    Lc = blkdiag(Adjoint(X), eye(6));
    Qc = blkdiag(Qg, Qa, zeros(3), (Qco+(1e4*eye(3).*(1-contact_0(i)))), (Qco+(1e4*eye(3).*(1-contact_1(i)))), (Qco+(1e4*eye(3).*(1-contact_2(i)))), (Qco+(1e4*eye(3).*(1-contact_3(i)))), Qbg, Qba);
     Qk = Ak * Lc * Qc * Lc' * Ak' * dt; %ԭ����
    %Qk = Qc;
    P_pred =  Ak * P * Ak' + Qk;
    [X_pred, theta_pred] =  ConstructState(R_pred, v_pred, p_pred, d0, d1, d2, d3, bg_pred, ba_pred);

end


%״̬�ĸ��¹���
function [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI)

    S = H * P_pred *H' + N;
    K = (P_pred * H') / S;
    X_Cell = repmat({X_pred}, 1, length(Y)/size(X_pred, 1));
    Z = blkdiag(X_Cell{:}) * Y - b;
    delta = K * PI * Z;
    dX = exp(delta(1:end-6));
    dtheta = delta(end-5:end);
    X_update = dX * X_pred;
    theta_update = dtheta + theta_pred;
    I = eye(size(P_pred));
    P_update = (I - K*H) * P_pred * (I - K*H)' + K * N * K';

end


%�������������ĽӴ������ʹ�ò�ͬ�Ĺ۲ⷽ��
function [X_update, theta_update, P_update] = UpdateState_forwardkinematics(X_pred, theta_pred, P_pred, contact_0, contact_1, contact_2, contact_3, d0_measure_pos, d1_measure_pos, d2_measure_pos, d3_measure_pos, i, Qe, Np)
    
    [R_pred, v_pred, p_pred, d0, d1, d2, d3, bg_pred, ba_pred] = SeperateState(X_pred, theta_pred);

    %������ͬʱ�����Ӵ�
    if contact_0(i) == 1 && contact_1(i) == 1 && contact_2(i) == 1 && contact_3(i) == 1

%ע��۲��������д��������ʽ���ᵼ�¹۲����Z = X * Y - b�����������ɢ
%         Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
%              d1_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
%              d2_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
%              d3_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0];


        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0;
             d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];       
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3), zeros(3), zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np, zeros(3), zeros(3);
             zeros(3), zeros(3), R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), zeros(3), zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,27);
              zeros(3,9), eye(3), zeros(3,6), zeros(3,18);
              zeros(3,18), eye(3), zeros(3,6), zeros(3,9);
              zeros(3,27), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);


    %ֻ��������ͬʱ�����Ӵ�
    elseif contact_0(i) == 0 && contact_1(i) == 1 && contact_2(i) == 1 && contact_3(i) == 1

        Y = [d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0;
             d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3), zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,18);
              zeros(3,9), eye(3), zeros(3,6), zeros(3,9);
              zeros(3,18), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 1 && contact_1(i) == 0 && contact_2(i) == 1 && contact_3(i) == 1

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3), zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,18);
              zeros(3,9), eye(3), zeros(3,6), zeros(3,9);
              zeros(3,18), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 1 && contact_1(i) == 1 && contact_2(i) == 0 && contact_3(i) == 1

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3), zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,18);
              zeros(3,9), eye(3), zeros(3,6), zeros(3,9);
              zeros(3,18), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 1 && contact_1(i) == 1 && contact_2(i) == 1 && contact_3(i) == 0

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0;
             d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3), zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,18);
              zeros(3,9), eye(3), zeros(3,6), zeros(3,9);
              zeros(3,18), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);


    %ֻ��������ͬʱ�����Ӵ�
    elseif contact_0(i) == 0 && contact_1(i) == 0 && contact_2(i) == 1 && contact_3(i) == 1

        Y = [d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,9);
              zeros(3,9), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 0 && contact_1(i) == 1 && contact_2(i) == 0 && contact_3(i) == 1

        Y = [d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,9);
              zeros(3,9), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 0 && contact_1(i) == 1 && contact_2(i) == 1 && contact_3(i) == 0

        Y = [d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0;
             d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,9);
              zeros(3,9), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 1 && contact_1(i) == 0 && contact_2(i) == 0 && contact_3(i) == 1

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,9);
              zeros(3,9), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 1 && contact_1(i) == 0 && contact_2(i) == 1 && contact_3(i) == 0

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,9);
              zeros(3,9), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 1 && contact_1(i) == 1 && contact_2(i) == 0 && contact_3(i) == 0

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0;
             d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6);
             zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6)];
        N = [R_pred*Qe*R_pred' + Np, zeros(3);
             zeros(3), R_pred*Qe*R_pred' + Np];
        PI = [eye(3), zeros(3,6), zeros(3,9);
              zeros(3,9), eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    
    %ֻ��һ���������Ӵ�
    elseif contact_0(i) == 1 && contact_1(i) == 0 && contact_2(i) == 0 && contact_3(i) == 0

        Y = [d0_measure_pos(1:3, i); 0; 1; -1; 0; 0; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), eye(3), zeros(3), zeros(3), zeros(3), zeros(3, 6)];
        N = R_pred*Qe*R_pred' + Np;
        PI = [eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 0 && contact_1(i) == 1 && contact_2(i) == 0 && contact_3(i) == 0 

        Y = [d1_measure_pos(1:3, i); 0; 1; 0; -1; 0; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), eye(3), zeros(3), zeros(3), zeros(3, 6)];
        N = R_pred*Qe*R_pred' + Np;
        PI = [eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 0 && contact_1(i) == 0 && contact_2(i) == 1 && contact_3(i) == 0

        Y = [d2_measure_pos(1:3, i); 0; 1; 0; 0; -1; 0];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), eye(3), zeros(3), zeros(3, 6)];
        N = R_pred*Qe*R_pred' + Np;
        PI = [eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);

    elseif contact_0(i) == 0 && contact_1(i) == 0 && contact_2(i) == 0 && contact_3(i) == 1

        Y = [d3_measure_pos(1:3, i); 0; 1; 0; 0; 0; -1];
        b = zeros(size(Y));
        H = [zeros(3), zeros(3), -eye(3), zeros(3), zeros(3), zeros(3), eye(3), zeros(3, 6)];
        N = R_pred*Qe*R_pred' + Np;
        PI = [eye(3), zeros(3,6)];
        [X_update, theta_update, P_update] = UpdateState(X_pred, theta_pred, P_pred, Y, b, H, N, PI);


    end
end


%��״̬���з��룬��ȡ���еĸ���״̬����
function [R_, v_, p_, d0, d1, d2, d3, bg_, ba_] = SeperateState(X, theta)

    R_ = X(1:3, 1:3);
    v_ = X(1:3, 4);
    p_ = X(1:3, 5);
    d0 = X(1:3, 6);
    d1 = X(1:3, 7);
    d2 = X(1:3, 8);
    d3 = X(1:3, 9);
    bg_ = theta(1:3);
    ba_ = theta(4:6);

end


%����״̬��������״̬
function [X, theta] = ConstructState(R_, v_, p_, d0, d1, d2, d3, bg_, ba_)
  
    X = [R_, v_, p_, d0, d1, d2, d3;
        0, 0, 0, 1, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1];
    theta = [bg_; ba_];

end


%ŷ����ת��ת����
function rotation_matrix = EulerAnglesToRotationMatrix(euler_angles)

    roll = euler_angles(1);%x
    pitch = euler_angles(2);%y
    yaw = euler_angles(3);%z
    Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    rotation_matrix = Rx * Ry * Rz;

end


%���Գƾ���
function A = skew(v)

    A = [0, -v(3), v(2);
        v(3), 0, -v(1);
        -v(2), v(1), 0];

end


%ָ��ӳ��
function exp_result = exp(v)

    N = (length(v)-3) / 3;
    Lg = zeros(N + 3);
    Lg(1:3, :) = [skew(v(1:3)), reshape(v(4:end), 3, [])];
    exp_result = expm(Lg);

end


%�����������
function Adjx = Adjoint(X)

    N = size(X, 2) - 3;
    R = X(1:3, 1:3);
    R_cell = repmat({R}, 1, N + 1);
    Adjx = blkdiag(R_cell{:});
    for i = 1:N
        Adjx(1 + 3*i:3 + 3*i, 1:3) = skew(X(1:3, 3 + i))*R;
    end

end


%��ת����תŷ����
function euler_angles = RotationMatrixToEulerAngles(rotation_matrix)

    euler_angles = transpose(rotm2eul(rotation_matrix, 'ZYX'));
    euler_angles = rad2deg(euler_angles);

end


%��Ԫ��תŷ����
function euler_angles = QuaternionToEulerAngles(quaternion)
    
    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);
    w = quaternion(4);
    norm = sqrt(x^2 + y^2 + z^2 + w^2);
    x = x / norm;
    y = y / norm;
    z = z / norm;
    w = w / norm;
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2; % use 90 degrees if out of range
    else
        pitch = asin(sinp);
    end
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    euler_angles = transpose([roll, pitch, yaw]);
  
    % ������תΪ�Ƕ�
    euler_angles = rad2deg(euler_angles);

end


%��Ԫ��ת��ת����
function rotation_matrix = QuaternionToRotationMatrix(quaternion)

    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);
    w = quaternion(4);
    norm = sqrt(x^2 + y^2 + z^2 + w^2);
    x = x / norm;
    y = y / norm;
    z = z / norm;
    w = w / norm;
    rotation_matrix = [1 - 2*y^2 - 2*z^2, 2*x*y - 2*z*w, 2*x*z + 2*y*w;
                       2*x*y + 2*z*w, 1 - 2*x^2 - 2*z^2, 2*y*z - 2*x*w;
                       2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x^2 - 2*y^2];

end

%% ��Ԫ��תR����
function R = quaternionToRotationMatrix(quaternion)
    %QUATERNIONTOROTATIONMATRIX Converts quaternion to rotation matrix.
    % Input:
    %   x, y, z, w - Quaternion components
    % Output:
    %   R - 3x3 rotation matrix
    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);
    w = quaternion(4);
    % Normalize the quaternion
    norm_q = sqrt(x^2 + y^2 + z^2 + w^2);
    x = x / norm_q;
    y = y / norm_q;
    z = z / norm_q;
    w = w / norm_q;
    
    % Compute the rotation matrix elements
    R = [1 - 2*y^2 - 2*z^2, 2*x*y - 2*z*w, 2*x*z + 2*y*w;
         2*x*y + 2*z*w, 1 - 2*x^2 - 2*z^2, 2*y*z - 2*x*w;
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x^2 - 2*y^2];
end

%% R����ת��Ԫ��
function quaternion = rotationMatrixToQuaternion(R)
    %ROTATIONMATRIXTOQUATERNION Converts rotation matrix to quaternion.
    % Input:
    %   R - 3x3 rotation matrix
    % Output:
    %   x, y, z, w - Quaternion components
    
    % Ensure the matrix is orthogonal and its determinant is 1
    if abs(det(R) - 1) > 1e-6
        error('Input matrix is not a valid rotation matrix');
    end
    
    % Compute the trace of the matrix
    trace_R = trace(R);
    
    if trace_R > 0
        S = sqrt(trace_R + 1.0) * 2; % S = 4 * w
        w = 0.25 * S;
        x = (R(3,2) - R(2,3)) / S;
        y = (R(1,3) - R(3,1)) / S;
        z = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S = 4 * x
        w = (R(3,2) - R(2,3)) / S;
        x = 0.25 * S;
        y = (R(1,2) + R(2,1)) / S;
        z = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S = 4 * y
        w = (R(1,3) - R(3,1)) / S;
        x = (R(1,2) + R(2,1)) / S;
        y = 0.25 * S;
        z = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S = 4 * z
        w = (R(2,1) - R(1,2)) / S;
        x = (R(1,3) + R(3,1)) / S;
        y = (R(2,3) + R(3,2)) / S;
        z = 0.25 * S;
    end
    quaternion = [x, y, z, w];
end

%�����˲��������ʵ�����λ�á����򡢶�ά�켣�Ա�ͼ
function plot_results(time, p_history, position_true_resampled, o_history, o_true)

    %λ�öԱ�ͼ
    figure;
    subplot(3, 1, 1);
    plot(time, p_history(1, :), 'b', time, position_true_resampled(1, :), 'r');
    title('Position - X');
    legend('Computed', 'True');

    subplot(3, 1, 2);
    plot(time, p_history(2, :), 'b', time, position_true_resampled(2, :), 'r');
    title('Position - Y');
    legend('Computed', 'True');    

    subplot(3, 1, 3);
    plot(time, p_history(3, :), 'b', time, position_true_resampled(3, :), 'r');
    title('Position - Z');
    legend('Computed', 'True');

    %����Ա�ͼ
    figure;
    subplot(3, 1, 1);
    plot(time, o_history(3, :), 'b', time, o_true(1, :), 'r');
    title('Orientation - Roll');
    legend('Computed', 'True');

    subplot(3, 1, 2);
    plot(time, o_history(2, :), 'b', time, o_true(2, :), 'r');
    title('Orientation - Pitch');
    legend('Computed', 'True');

    subplot(3, 1, 3);
    plot(time, o_history(1, :), 'b', time, o_true(3, :), 'r');
    title('Orientation - Yaw');
    legend('Computed', 'True');

    %��ά�켣�Ա�ͼ
    figure;
    plot(position_true_resampled(1, :), position_true_resampled(2, :), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(p_history(1, :), p_history(2, :), 'b', 'LineWidth', 1.5);
    title('2D Trajectory - XY Plane');
    xlabel('X Position');
    ylabel('Y Position');
    legend('True', 'Computed');
    hold off;


end



