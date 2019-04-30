close all; clc;
clear x_all y_all math_traj base_start base_dest base_traj

Vc = 0.1; 
delta_t = 0.1;

base_start.x = 0.2;
base_start.y = -0.2;
base_start.z = 0.5;
base_dest.x = 0.8;
base_dest.y = 0.3;
base_dest.z = 0.1;

base_traj = get_base_traj(base_start, base_dest, Vc, delta_t);

for i = 1:size(base_traj,2)
    x_all(i) = base_traj(1,i);
    y_all(i) = base_traj(2,i);
    z_all(i) = base_traj(3,i);
end

figure, 
plot3(x_all, y_all, z_all);
axis equal;
grid on;
xlim([0, 1.0]);
ylim([-0.5, 0.5]);
zlim([0, 0.5]);
xlabel('x in meters');
ylabel('y in meters');
zlabel('z in meters');
view(175, 15);

function point_all = gen_math_traj(point_x, point_y, Vc, delta_t)
    if(point_x>0.00001)
        k = point_y/point_x^2;
    else
        k = 0;
    end
    
    point_start = output_math_curve(point_x, k, Vc);
    point_num = 1;
    point_all(point_num) = point_start;
    
    point_pre = point_start;
    while(point_num<10000)
        point_num = point_num + 1;
        
        % using delta_x to generate next point will slower motion when close
        % to orgin. 
        if(point_pre.Vx < point_pre.Vy && point_pre.y > point_pre.Vy * delta_t)
            point_x = sqrt((point_pre.y - point_pre.Vy * delta_t)/k);
        else
            point_x = point_pre.x - point_pre.Vx * delta_t;
        end
            
        if(point_x>0)
            point_all(point_num) = output_math_curve(point_x, k, Vc);
        else
            point_all(point_num) = output_math_curve(0, k, Vc);
            break;
        end
        point_pre = point_all(point_num);
    end

end


function point = output_math_curve(x, k, Vc)
    point.x = x;
    point.y = k * x^2;
    
    y_dot = 2 * k * x; % curve slop
    
    point.Vx = Vc * cos(atan(y_dot));
    point.Vy = Vc * sin(atan(y_dot));
end


function base_traj = get_base_traj(base_start, base_dest, Vc, delta_t)
    % build up transformation matrix
    theta = atan2(base_dest.y-base_start.y, base_dest.x-base_start.x );
    R = eye(3) * rotz(theta) * roty(-pi/2) * rotx(pi/2);
    Tmath_wrt_base = [R, [base_dest.x; base_dest.y; base_dest.z]; 0, 0, 0, 1];
    Tbase_wrt_math = [R', -R'*[base_dest.x; base_dest.y; base_dest.z]; 0, 0, 0, 1];
    
    math_dest = Tbase_wrt_math * [base_dest.x; base_dest.y; base_dest.z; 1];
%     print(['math_dest is ', num2str(math_dest')]);
    
    math_start = Tbase_wrt_math * [base_start.x; base_start.y; base_start.z; 1];
    math_traj = gen_math_traj(math_start(1), math_start(2), Vc, delta_t);
    for i = 1:length(math_traj)
        base_traj(:,i) = Tmath_wrt_base * [math_traj(i).x; math_traj(i).y; 0; 1];
    end
end

function Rx = rotx(tx)
    Rx = [1,   0,  0;
        0,  cos(tx), -sin(tx);
        0,  sin(tx), cos(tx)];
end

function Ry = roty(ty)
    Ry = [cos(ty),   0,  sin(ty);
        0,  1, 0;
        -sin(ty),  0, cos(ty)];
end

function Rz = rotz(tz)
    Rz = [cos(tz),   -sin(tz),  0;
        sin(tz),    cos(tz),  0;
        0,  0, 1];
end

