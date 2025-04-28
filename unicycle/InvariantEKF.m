classdef InvariantEKF
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     InvariantEKF
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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
    
    properties (Access = public)
        sys 
        mus 
        sigmas
        xe
        scalex
        scaley
        iteration_counts = [];
    end
    
    methods
   
        function obj = InvariantEKF(system, mu_0, sigma_0)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     obj = InvariantEKF(system, mu_0, sigma_0)
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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            obj.sys = system;
            
            if numel(mu_0) == 3
                R = [cos(mu_0(3)), -sin(mu_0(3)), mu_0(1);
                     sin(mu_0(3)), cos(mu_0(3)), mu_0(2);
                     0, 0, 1];
                mu_0 = R;
            end
            
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
% Created:                         26-Apr-2025 15:11:09
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
% Created:                         26-Apr-2025 15:11:09
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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
            
            mu_bar = obj.sys.f_lie(obj.getmu, u);
           
            adj_u = obj.sys.adjoint( inv(expm(obj.sys.carat([u(1), 0, u(2)] * obj.sys.deltaT))) );
            sigma_bar = adj_u * obj.getsigma * adj_u' + obj.sys.Q * obj.sys.deltaT^2;
            
           
            obj.mus{end + 1} = mu_bar;
            obj.sigmas{end + 1} = sigma_bar;
            
           
            return;
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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            H = [1, 0, 0;
                 0, 1, 0];
            
            V = inv(obj.getmu) * z' - obj.sys.b;
            V = V(1:end-1);  
            
            invmu = inv(obj.getmu);
            invmu=invmu(1:2, 1:2);
           
             [bp,flag] = chol(obj.getsigma,'lower') ;
            if(flag~=0)
                flag
            end
            K = obj.getsigma * H' / (H * obj.getsigma * H' + invmu * obj.sys.R * invmu');
            
            obj.mus{end} = obj.getmu * expm(obj.sys.carat(K * V));
            obj.sigmas{end} = (eye(3) - K * H) * obj.getsigma;
            
            mu = obj.getmu;
            sigma = obj.getsigma;
        end
        
        function [mu, sigma,obj] = update_mkc_new(obj, z,i,kk)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mu, sigma,obj] = update_mkc_new(obj, z,i,kk)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    z - [m,n] size,[double] type,Description
%    i - [m,n] size,[double] type,Description
%    kk - [m,n] size,[double] type,Description
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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            H = [1, 0, 0;
                 0, 1, 0];
             
            
            V = inv(obj.getmu) * z' - obj.sys.b;
            V = V(1:end-1);  
            
            invmu = inv(obj.getmu);
            invmu=invmu(1:2, 1:2);
           
            % 
            sigma_r=[5,5]';
            sigma_p=[5,5,5]';
            [bp,flag] = chol(obj.getsigma,'lower') ;
            br = chol(invmu * obj.sys.R * invmu','lower') ;
            if(flag~=0)
                flag
            end
            cnt=5;
            num=5;
            m=2;
            n=3;
            xe=zeros(1,cnt);
            scalex=ones(1,n);
            scaley=ones(1,m);
           
            sigma_r=[kk,kk]';
            sigma_p=[500,500,500]';        
          
            iter_count = 0;
            while(num>0)
                %  
                if(num==cnt)
                  x_tlast=zeros(n,1); 
                else  
                  x_tlast=x_t; 
                end
                iter_count = iter_count + 1;
                num=num-1;
                z_=V;
                dp= zeros(3,1);
                wp= bp\x_tlast;
                ep=dp-wp;
                scalex=exp(-ep.*ep./(2*sigma_p.*sigma_p));
                Cx=diag(exp(-ep.*ep./(2*sigma_p.*sigma_p)));
                for kk=1:n
                    if(Cx(kk,kk)<0.01)
                        Cx(kk,kk)=0.01;
                    end   
                end
                dr= br\z_;
                wr= br\H*x_tlast;
                er=dr-wr;
                scaley=exp(-er.*er./(2*sigma_r.*sigma_r));
                Cy=diag(exp(-er.*er./(2*sigma_r.*sigma_r)));
                for kk=1:m
                    if(Cy(kk,kk)<0.01)
                        Cy(kk,kk)=0.01;
                    end
                end
                R_1=br/Cy*br';
                Sigma_1=bp/Cx*bp';
                Sn=H * Sigma_1 * H' + R_1;
                K_1=Sigma_1*H'/Sn;
                x_t=K_1*V;
                xe(cnt-num)=norm(x_t-x_tlast)/(norm(x_tlast)+0.1);
               
                if(xe(cnt-num)<0.6)
                    break
                end
            end 
            obj.iteration_counts = [obj.iteration_counts, iter_count];
            
            
            obj.mus{end} = obj.getmu * expm(obj.sys.carat(K_1 * V));
            obj.sigmas{end} = (eye(3) - K_1 * H) * obj.getsigma;
           
            mu = obj.getmu;
            sigma = obj.getsigma;
            
            obj.xe{end+1}=xe;
            obj.scalex{end+1}=scalex;
            obj.scaley{end+1}=scaley;


        end


function [mu, sigma,obj] = update_mck_new(obj, z,i,kk)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mu, sigma,obj] = update_mck_new(obj, z,i,kk)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    z - [m,n] size,[double] type,Description
%    i - [m,n] size,[double] type,Description
%    kk - [m,n] size,[double] type,Description
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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            H = [1, 0, 0;
                 0, 1, 0];
             
            
            V = inv(obj.getmu) * z' - obj.sys.b;
            V = V(1:end-1); 
            
            invmu = inv(obj.getmu);
            invmu=invmu(1:2, 1:2);
           
           
            sigma_r=[5,5]';
            sigma_p=[5,5,5]';
            [bp,flag] = chol(obj.getsigma,'lower') ;
            br = chol(invmu * obj.sys.R * invmu','lower') ;
            if(flag~=0)
                flag
            end
            cnt=5;
            num=5;
            m=2;
            n=3;
            xe=zeros(1,cnt);
            scalex=ones(1,n);
            scaley=ones(1,m);
            
            sigma_r=[kk,kk]';
            sigma_p=[kk,kk,kk]';        
           
            iter_count = 0;
            while(num>0)
                 
                if(num==cnt)
                  x_tlast=zeros(n,1); 
                else  
                  x_tlast=x_t; 
                end
                iter_count = iter_count + 1;
                num=num-1;
                z_=V;
                dp= zeros(3,1);
                wp= bp\x_tlast;
                ep=dp-wp;
                scalex=exp(-ep.*ep./(2*sigma_p.*sigma_p));
                Cx=diag(exp(-ep.*ep./(2*sigma_p.*sigma_p)));
                for kk=1:n
                    if(Cx(kk,kk)<0.01)
                        Cx(kk,kk)=0.01;
                    end   
                end
                dr= br\z_;
                wr= br\H*x_tlast;
                er=dr-wr;
                scaley=exp(-er.*er./(2*sigma_r.*sigma_r));
                Cy=diag(exp(-er.*er./(2*sigma_r.*sigma_r)));
                for kk=1:m
                    if(Cy(kk,kk)<0.01)
                        Cy(kk,kk)=0.01;
                    end
                end
                R_1=br/Cy*br';
                Sigma_1=bp/Cx*bp';
                Sn=H * Sigma_1 * H' + R_1;
                K_1=Sigma_1*H'/Sn;
                x_t=K_1*V;
                xe(cnt-num)=norm(x_t-x_tlast)/(norm(x_tlast)+0.1);
                
                if(xe(cnt-num)<0.6)
                    break
                end
            end 
            obj.iteration_counts = [obj.iteration_counts, iter_count];
           
            obj.mus{end} = obj.getmu * expm(obj.sys.carat(K_1 * V));
            obj.sigmas{end} = (eye(3) - K_1 * H) * obj.getsigma;
            
            mu = obj.getmu;
            sigma = obj.getsigma;
            
            obj.xe{end+1}=xe;
            obj.scalex{end+1}=scalex;
            obj.scaley{end+1}=scaley;


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
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            for i = 1:size(us, 1)
                u = us(i, :);
                z = zs(i, :);
                
                [mu_bar, sigma_bar,obj]=obj.predict(u);
                [mu_bar, sigma_bar,obj]=obj.update(z);
            end
            
           
            for k=1:size(us, 1)
                mus(k,:,:)=cell2mat(obj.mus(k+1));
                sigmas(k,:,:) = cell2mat(obj.sigmas(2:end));
            end
        end

        function [mus, sigmas,objnew] = iterate_mkc(obj, us, zs)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mus, sigmas,objnew] = iterate_mkc(obj, us, zs)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    us - [m,n] size,[double] type,Description
%    zs - [m,n] size,[double] type,Description
% 
% Outputs:
%    mus - [m,n] size,[double] type,Description
%    sigmas - [m,n] size,[double] type,Description
%    objnew - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            
            for i = 1:size(us, 1)
                u = us(i, :);
                z = zs(i, :);
                
                [mu_bar, sigma_bar,obj]=obj.predict(u);
                [mu_bar, sigma_bar,obj]=obj.update_mkc(z);
            end
          

            for k=1:length(us)
                mus(k,:,:)=cell2mat(obj.mus(k+1));
                sigmas(k,:,:) = cell2mat(obj.sigmas(k+1));
                xee(k,:)=cell2mat(obj.xe(k));
            end

            objnew.mus=mus;
            objnew.sigmas=sigmas;
            objnew.xe=xee;
        end
        function [mus, sigmas,objnew,iter] = iterate_mkc_new(obj, us, zs,kk)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mus, sigmas,objnew,iter] = iterate_mkc_new(obj, us, zs,kk)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    us - [m,n] size,[double] type,Description
%    zs - [m,n] size,[double] type,Description
%    kk - [m,n] size,[double] type,Description
% 
% Outputs:
%    mus - [m,n] size,[double] type,Description
%    sigmas - [m,n] size,[double] type,Description
%    objnew - [m,n] size,[double] type,Description
%    iter - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
           
            obj.iteration_counts = []; 
            for i = 1:size(us, 1)
                u = us(i, :);
                z = zs(i, :);
                
                [mu_bar, sigma_bar,obj]=obj.predict(u);
                [mu_bar, sigma_bar,obj]=obj.update_mkc_new(z,i,kk);
            end
           
            for k=1:length(us)
                mus(k,:,:)=cell2mat(obj.mus(k+1));
                sigmas(k,:,:) = cell2mat(obj.sigmas(k+1));
                xee(k,:)=cell2mat(obj.xe(k));
                scalex(k,:)=cell2mat(obj.scalex(k));
            end
            avg_iter = mean(obj.iteration_counts);
            iter=avg_iter;
            objnew.mus=mus;
            objnew.sigmas=sigmas;
            objnew.xe=xee;
            objnew.scalex=scalex;
        end

function [mus, sigmas,objnew,iter] = iterate_mck_new(obj, us, zs,kk)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     [mus, sigmas,objnew,iter] = iterate_mck_new(obj, us, zs,kk)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    us - [m,n] size,[double] type,Description
%    zs - [m,n] size,[double] type,Description
%    kk - [m,n] size,[double] type,Description
% 
% Outputs:
%    mus - [m,n] size,[double] type,Description
%    sigmas - [m,n] size,[double] type,Description
%    objnew - [m,n] size,[double] type,Description
%    iter - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%
          
            obj.iteration_counts = []; 
            for i = 1:size(us, 1)
                u = us(i, :);
                z = zs(i, :);
                
                [mu_bar, sigma_bar,obj]=obj.predict(u);
                [mu_bar, sigma_bar,obj]=obj.update_mck_new(z,i,kk);
            end
            
            
            for k=1:length(us)
                mus(k,:,:)=cell2mat(obj.mus(k+1));
                sigmas(k,:,:) = cell2mat(obj.sigmas(k+1));
                xee(k,:)=cell2mat(obj.xe(k));
                scalex(k,:)=cell2mat(obj.scalex(k));
            end
            avg_iter = mean(obj.iteration_counts);
            iter=avg_iter;
            objnew.mus=mus;
            objnew.sigmas=sigmas;
            objnew.xe=xee;
            objnew.scalex=scalex;
        end

function mse = calculate_mse(obj,musn,xn)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     mse = calculate_mse(obj,musn,xn)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    musn - [m,n] size,[double] type,Description
%    xn - [m,n] size,[double] type,Description
% 
% Outputs:
%    mse - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%

            mse_x = 0;
            mse_y = 0;
            mse_theta = 0;
            
            for k = 1:100
                
                mu = squeeze(musn(k,:,:)); 
                x_est = mu(1, 3);
                y_est = mu(2, 3);
                theta_est = atan2(mu(2, 1), mu(1, 1)); 
                true_state = squeeze(xn(k, :,:));
                x_true = true_state(1,3);
                y_true = true_state(2,3);
                theta_true = atan2(true_state(2, 1), true_state(1, 1));
                
                
                error_x = x_est - x_true;
                error_y = y_est - y_true;
                error_theta = angdiff(theta_true, theta_est); 
                mse_x = mse_x + error_x^2;
                mse_y = mse_y + error_y^2;
                mse_theta = mse_theta + error_theta^2;
            end
            
            
            mse_x = sqrt(mse_x / 100);
            mse_y =sqrt( mse_y / 100);
            mse_theta = sqrt(mse_theta / 100);
            
            
            mse = struct('x', mse_x, 'y', mse_y, 'theta', mse_theta);
        end

        function mse = calculate_mse_ekf(obj,musn,xn)
% Brief: One line description of what the function or class performs
% Details:
%    None
% 
% Syntax:  
%     mse = calculate_mse_ekf(obj,musn,xn)
% 
% Inputs:
%    obj - [m,n] size,[double] type,Description
%    musn - [m,n] size,[double] type,Description
%    xn - [m,n] size,[double] type,Description
% 
% Outputs:
%    mse - [m,n] size,[double] type,Description
% 
% Example: 
%    None
% 
% See also: None

% Author:                          zhongchongyi
% Email:                           chongyizhong@bit.edu.com
% Created:                         26-Apr-2025 15:11:09
% Version history revision notes:
%                                  None
% Implementation In Matlab R2022a
% Copyright © 2025 .All Rights Reserved.
%

            mse_x = 0;
            mse_y = 0;
            mse_theta = 0;
            
            for k = 1:100
                
                mu = squeeze(musn(k,:)); 
                x_est = mu(1);
                y_est = mu(2);
                theta_est = mu(3); 
                true_state = squeeze(xn(k, :,:));
                x_true = true_state(1,3);
                y_true = true_state(2,3);
                theta_true = atan2(true_state(2, 1), true_state(1, 1));
                
               
                error_x = x_est - x_true;
                error_y = y_est - y_true;
                error_theta = angdiff(theta_true, theta_est);
                mse_x = mse_x + error_x^2;
                mse_y = mse_y + error_y^2;
                mse_theta = mse_theta + error_theta^2;
            end
            
            
           mse_x = sqrt(mse_x / 100);
            mse_y =sqrt( mse_y / 100);
            mse_theta = sqrt(mse_theta / 100);
            
           
            mse = struct('x', mse_x, 'y', mse_y, 'theta', mse_theta);
        end

    end
end
