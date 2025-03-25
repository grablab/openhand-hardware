%% Function to search NN in KD tree and lookup alpha & beta

function [alpha, beta, D] = searchKDTree(NNquery, a_curr, b_curr)
    load('kd-data.mat', 'Mdl', 'Y');
    
    alpha = zeros(1,3);
    beta = zeros(1,3);
    D = zeros(1,3);

    ang_delta_thresh = deg2rad(50);
    n_knn = 5;
    
    for f = 1:3
        Q = NNquery(:,f)';
        [Idx, dist] = knnsearch(Mdl{f}, Q, 'K', n_knn);
        
        alpha(f) = Y{f}(Idx(1), 1);
        beta(f) = Y{f}(Idx(1), 2);
        D(f) = dist(1);
        
        itr = 2;
        while (norm(alpha - a_curr) > ang_delta_thresh) || (norm(beta - b_curr) > ang_delta_thresh)
            alpha(f) = Y{f}(Idx(itr), 1);
            beta(f) = Y{f}(Idx(itr), 2);
            D(f) = dist(itr);
            itr = itr + 1;
            if itr == n_knn+1
                break
            end
        end
        
        disp(dist);
    end

end