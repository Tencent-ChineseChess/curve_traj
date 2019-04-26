close all; clc;
clear x_all y_all math_traj;
math_traj = gen_math_traj(3, 5, 0.1, 0.1);
for i = 1:length(math_traj)
    x_all(i) = math_traj(i).x;
    y_all(i) = math_traj(i).y;
end
figure, 
plot(x_all, y_all);
axis equal;

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

function base_traj = get_base_traj(math_traj)

end
