classdef ExtendedKalmanFilter
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     ExtendedKalmanFilter
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
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
    properties
        sys
        mus
        sigmas
    end
    
    methods
        function obj = ExtendedKalmanFilter(system, mu_0, sigma_0)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     obj = ExtendedKalmanFilter(system, mu_0, sigma_0)
% 
% Inputs:
%    system - [m,n] size,[double] type,Description
%    mu_0 - [m,n] size,[double] type,Description
%    sigma_0 - [m,n] size,[double] type,Description
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
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            
            obj.sys = system;
            obj.mus = {mu_0};  
            obj.sigmas = {sigma_0};  
        end
        
      
        function mu = getmu(obj)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     mu = getmu(obj)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
% 
% Outputs:
%    mu - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            mu = obj.mus{end};
        end

       
        function sigma = getsigma(obj)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     sigma = getsigma(obj)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
% 
% Outputs:
%    sigma - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            sigma = obj.sigmas{end};
        end

       
        function [mu_bar, sigma_bar,obj] = predict(obj, u)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mu_bar, sigma_bar,obj] = predict(obj, u)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    u - [m,n] size,[double] type,Description
% 
% Outputs:
%    mu_bar - [m,n] size,[double] type,Description
%    sigma_bar - [m,n] size,[double] type,Description
%    obj - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            mu_bar = obj.sys.f_standard(obj.getmu, u);
            F = obj.sys.F(mu_bar, u);
            F_u = obj.sys.F_u(mu_bar, u);
            sigma_bar = F * obj.getsigma * F' + F_u * obj.sys.Q([1,3], [1,3]) * F_u' * obj.sys.deltaT^2;

            
            obj.mus{end+1} = mu_bar;
            obj.sigmas{end+1} = sigma_bar;
        end
        
        
        function [mu, sigma,obj] = update(obj, z)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mu, sigma,obj] = update(obj, z)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    z - [m,n] size,[double] type,Description
% 
% Outputs:
%    mu - [m,n] size,[double] type,Description
%    sigma - [m,n] size,[double] type,Description
%    obj - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            H = obj.sys.H(obj.getmu);
            zbar = obj.sys.h(obj.getmu);
            
            K = obj.getsigma * H' / (H * obj.getsigma * H' + obj.sys.R);
            obj.mus{end} = obj.getmu + K * (z' - zbar);
            obj.sigmas{end} = (eye(size(K,1)) - K * H) * obj.getsigma;
            
            mu = obj.getmu;
            sigma = obj.getsigma;
        end
        
        
        function [mus, sigmas] = iterate(obj, us, zs)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mus, sigmas] = iterate(obj, us, zs)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    us - [m,n] size,[double] type,Description
%    zs - [m,n] size,[double] type,Description
% 
% Outputs:
%    mus - [m,n] size,[double] type,Description
%    sigmas - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:13:35
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            for i = 1:length(us)
                [mu_bar, sigma_bar,obj]=obj.predict(us(i,:));
                [mu_bar, sigma_bar,obj]=obj.update(zs(i,:));
            end
            
            
             for k=1:size(us, 1)
                mus(k,:,:)=cell2mat(obj.mus(k+1));
                sigmas(k,:,:) = cell2mat(obj.sigmas(2:end));
             end

        end
    end
end

