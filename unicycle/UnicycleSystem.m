classdef UnicycleSystem
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     UnicycleSystem
% 
% Inputs:
%    None
% 
% Outputs:
%    None
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%


    properties
        Q       
        R       
        deltaT  
        b       
    end
    
    methods
        function obj = UnicycleSystem(Q, R, deltaT)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     obj = UnicycleSystem(Q, R, deltaT)
% 
% Inputs:
%    Q - [m,n] size,[double] type,Description
%    R - [m,n] size,[double] type,Description
%    deltaT - [m,n] size,[double] type,Description
% 
% Outputs:
%    obj - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            obj.Q = Q;
            obj.R = R;
            obj.deltaT = deltaT;
            obj.b = [0; 0; 1]; 
        end
        
        function [x, u, z] = gen_data(obj, x0, u, t, noise)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [x, u, z] = gen_data(obj, x0, u, t, noise)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    x0 - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
%    t - [m,n] size,[double] type,Description
%    noise - [m,n] size,[double] type,Description
% 
% Outputs:
%    x - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
%    z - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            if isa(u, 'function_handle')
                u = arrayfun(@(t) u(t), 1:t, 'UniformOutput', false);
                u = cell2mat(u(:));
            elseif isscalar(u)
                u = repmat(u, t, 2);
            elseif size(u, 1) == 1
                u = repmat(u, t, 1);
            end
            
           
            x = zeros(t+1, 3, 3);
            z = zeros(t+1, 3);
            
            
            if numel(x0) == 3
                x0 = [cos(x0(3)), -sin(x0(3)), x0(1); 
                      sin(x0(3)), cos(x0(3)), x0(2); 
                      0, 0, 1];
            elseif size(x0, 1) ~= 3 || size(x0, 2) ~= 3
                error('Invalid size for x0!');
            end

           
            x(1, :, :) = x0;
            
            for i = 1:t
                
                x(i+1, :, :) = obj.f_lie(squeeze(x(i, :, :)), u(i, :), noise);
                
                z(i+1, :) = obj.h(squeeze(x(i+1, :, :)), noise);
            end

            x(1,:,:)=[]; 
            u;
            z(1,:)=[];
        end
        
        function [x, u, z] = gen_data_outlier(obj, x0, u, t, noise)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [x, u, z] = gen_data_outlier(obj, x0, u, t, noise)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    x0 - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
%    t - [m,n] size,[double] type,Description
%    noise - [m,n] size,[double] type,Description
% 
% Outputs:
%    x - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
%    z - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            if isa(u, 'function_handle')
                u = arrayfun(@(t) u(t), 1:t, 'UniformOutput', false);
                u = cell2mat(u(:));
            elseif isscalar(u)
                u = repmat(u, t, 2);
            elseif size(u, 1) == 1
                u = repmat(u, t, 1);
            end
            
            
            x = zeros(t+1, 3, 3);
            z = zeros(t+1, 3);
            
           
            if numel(x0) == 3
                x0 = [cos(x0(3)), -sin(x0(3)), x0(1); 
                      sin(x0(3)), cos(x0(3)), x0(2); 
                      0, 0, 1];
            elseif size(x0, 1) ~= 3 || size(x0, 2) ~= 3
                error('Invalid size for x0!');
            end

            
            x(1, :, :) = x0;
            
            for i = 1:t
                
                x(i+1, :, :) = obj.f_lie(squeeze(x(i, :, :)), u(i, :), noise);
               
                z(i+1, :) = obj.h1(squeeze(x(i+1, :, :)), noise);
            end

            x(1,:,:)=[]; 
            u;
            z(1,:)=[];
        end

        function state_new = f_lie(obj, state, u, noise)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     state_new = f_lie(obj, state, u, noise)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
%    noise - [m,n] size,[double] type,Description
% 
% Outputs:
%    state_new - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            if nargin==4
            if noise
                w = mvnrnd(zeros(1, 3), obj.Q);
            else
                w = zeros(1, 3);
            end
            else
                w = zeros(1, 3);
            end
            
            
            xi = [u(1); 0; u(2)] + w(:); 
            state_new = state * expm(obj.carat(xi) * obj.deltaT);
        end
        
        function state_new = f_standard(obj, state, u, noise)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     state_new = f_standard(obj, state, u, noise)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
%    noise - [m,n] size,[double] type,Description
% 
% Outputs:
%    state_new - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            if(nargin==4)
            if noise
                w = mvnrnd(zeros(1, 3), obj.Q);
            else
                w = zeros(1, 3);
            end
            else
                w = zeros(1, 3);
            end
            u = (u + w([1, 3])) * obj.deltaT;
            x_new = state(1) + u(1) * cos(state(3));
            y_new = state(2) + u(1) * sin(state(3));
            theta_new = state(3) + u(2);
            
            state_new = [x_new; y_new; theta_new];
        end
        
        function z = h(obj, state, noise)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     z = h(obj, state, noise)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
%    noise - [m,n] size,[double] type,Description
% 
% Outputs:
%    z - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           

            if size(state, 1) == 3 && size(state, 2) == 3
               
                z = (state * obj.b)';
            else
                
                z = state(1:2);
            end
            if(nargin==3)
                if noise
                    z(1:2) = z(1:2) + mvnrnd(zeros(1, 2), obj.R);
                end
            else
            end
        end
        
        function z = h1(obj, state, noise)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     z = h1(obj, state, noise)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
%    noise - [m,n] size,[double] type,Description
% 
% Outputs:
%    z - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            

            if size(state, 1) == 3 && size(state, 2) == 3
               
                z = (state * obj.b)';
            else
                
                z = state(1:2);
            end
            
            if noise
                if(abs(randn(1))<1.96) 
                ns=mvnrnd(zeros(1, 2), obj.R);
                else
                ns=mvnrnd(zeros(1, 2), 50*obj.R);
                end
                z(1:2) = z(1:2) +ns;
            end
        end

        function F = F(obj, state, u)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     F = F(obj, state, u)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
% 
% Outputs:
%    F - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            
            F = [1, 0, -u(1) * sin(state(3));
                 0, 1, u(1) * cos(state(3));
                 0, 0, 1];
        end
        
        function F_u = F_u(obj, state, u)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     F_u = F_u(obj, state, u)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
% 
% Outputs:
%    F_u - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            F_u = [cos(state(3)), 0;
                   sin(state(3)), 0;
                   0, 1];
        end
        
        function H = H(obj, state)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     H = H(obj, state)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    state - [m,n] size,[double] type,Description
% 
% Outputs:
%    H - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            H = [1, 0, 0;
                 0, 1, 0];
        end
        
        function xi_hat = carat(~, xi)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     xi_hat = carat(~, xi)
% 
% Inputs:
%    ~ - [m,n] size,[double] type,Description
%    xi - [m,n] size,[double] type,Description
% 
% Outputs:
%    xi_hat - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
       
            xi_hat = [0, -xi(3), xi(1);
                      xi(3), 0, xi(2);
                      0, 0, 0];
        end
        
        function Ad_xi = adjoint(~, xi)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     Ad_xi = adjoint(~, xi)
% 
% Inputs:
%    ~ - [m,n] size,[double] type,Description
%    xi - [m,n] size,[double] type,Description
% 
% Outputs:
%    Ad_xi - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:12:55
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            xi([1, 2], 3) = xi([2, 1], 3);
            xi(2, 3) = -xi(2, 3);  % see (140) of 
            Ad_xi = xi;
        end
    end
end
