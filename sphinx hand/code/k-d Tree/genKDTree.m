%% Function to generate KD tree of contact locations for each finger and lookup table of corresponding alpha & beta
% Training data matrix consists of contact positions for a range of alpha & beta
% Separate KD tree created for each finger
% Search for nearest XYZ contact position and lookup corresponding alpha & beta

function [] = genKDTree()

% Training data input range of alpha & beta
alpha_L = deg2rad(0);
alpha_H = deg2rad(90);

beta_L = deg2rad(-125);
beta_H = deg2rad(50);

alpha_arr = alpha_L:deg2rad(0.2):alpha_H;
beta_arr = beta_L:deg2rad(0.2):beta_H;

% Initializing Training Data matrix for contact positions
X = cell(3,1);
X{1} = zeros(length(alpha_arr)*length(beta_arr), 3);
X{2} = X{1};
X{3} = X{1};

% Initializing lookup table of alpha & beta
Y = cell(3,1);
Y{1} = zeros(length(alpha_arr)*length(beta_arr), 2);
Y{2} = Y{1};
Y{3} = Y{1};

% Initializing Hand Parameters
Rsph = 100;
L1 = 0.7*Rsph;
L2 = 0.7*Rsph;
L_R2 = sqrt(Rsph^2 - L2^2);
L_R1 = sqrt(L_R2^2 - L1^2);
R1ang = deg2rad(55);
contactAng = [pi/2, 7*pi/6, 11*pi/6];
PP = zeros(3,3);
NN = zeros(3,3);
for f = 1:3
    PP(:,f) = [L_R1*sin(R1ang)*cos(contactAng(f)) ; L_R1*sin(R1ang)*sin(contactAng(f)); -L_R1*cos(R1ang)];
end

% Generating the contact positions for the alpha & beta range
ctr = 0;
for aa = 1:length(alpha_arr)
    for bb = 1:length(beta_arr)
        ctr = ctr + 1;
        RR = zeros(3,3);
        for f = 1:3
            ey = [sin(contactAng(f)); -cos(contactAng(f)); 0];
            ex = cross(ey, PP(:,f));
            ex = ex./norm(ex);
            
            RR(:,f) = PP(:,f) + L1.*cos(alpha_arr(aa)).*ex + L1.*sin(alpha_arr(aa)).*ey;
            
            ey2 = cross(PP(:,f), RR(:,f));
            ey2 = ey2./norm(ey2);
            
            ex2 = cross(ey2, RR(:,f));
            ex2 = ex2./norm(ex2);
            
            NN(:,f) = RR(:,f) + L2.*cos(beta_arr(bb)).*ex2 + L2.*sin(beta_arr(bb)).*ey2;
            
            X{f}(ctr,:) = NN(:,f)';
            Y{f}(ctr,1) = alpha_arr(aa);
            Y{f}(ctr,2) = beta_arr(bb);
        end
    end
end


% Generating the KDtree model object
Mdl = cell(3,1);
Mdl{1} = KDTreeSearcher(X{1});
Mdl{2} = KDTreeSearcher(X{2});
Mdl{3} = KDTreeSearcher(X{3});


% Saving the generated training data in matrix
save('kd-data.mat', 'Mdl', 'X', 'Y', 'alpha_arr', 'beta_arr')

end