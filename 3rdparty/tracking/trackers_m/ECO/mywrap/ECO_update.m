function [params, region] = ECO_update(params_in, im)

params = params_in;
% Set conjugate gradient uptions
init_CG_opts.CG_use_FR = true;
init_CG_opts.tol = 1e-6;
init_CG_opts.CG_standard_alpha = true;
init_CG_opts.debug = params.debug;
CG_opts.CG_use_FR = params.CG_use_FR;
CG_opts.tol = 1e-6;
CG_opts.CG_standard_alpha = params.CG_standard_alpha;
CG_opts.debug = params.debug;
if params.CG_forgetting_rate == Inf || params.learning_rate >= 1
    CG_opts.init_forget_factor = 0;
else
    CG_opts.init_forget_factor = (1-params.learning_rate)^params.CG_forgetting_rate;
end

% Set feature info
img_support_sz = params.feature_info.img_support_sz;
feature_sz = params.feature_info.data_sz;
feature_dim = params.feature_info.dim;
num_feature_blocks = length(feature_dim);

% Allocate
scores_fs_feat = cell(1,1,num_feature_blocks);

% Number of Fourier coefficients to save for each filter layer. This will
% be an odd number.
filter_sz = feature_sz + mod(feature_sz+1, 2);
filter_sz_cell = permute(mat2cell(filter_sz, ones(1,num_feature_blocks), 2), [2 3 1]);

% The size of the label function DFT. Equal to the maximum filter size.
[output_sz, k1] = max(filter_sz, [], 1);
k1 = k1(1);

% Get the remaining block indices
block_inds = 1:num_feature_blocks;
block_inds(k1) = [];

% How much each feature block has to be padded to the obtain output_sz
pad_sz = cellfun(@(filter_sz) (output_sz - filter_sz) / 2, filter_sz_cell, 'uniformoutput', false);

res_norms = [];
residuals_pcg = [];

params.frame = params.frame+1;

tic();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Target localization step
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Do not estimate translation and scaling on the first frame, since we 
    % just want to initialize the tracker there
    if params.frame > 1
        old_pos = inf(size(params.pos));
        iter = 1;
        
        %translation search
        while iter <= params.refinement_iterations && any(old_pos ~= params.pos)
            % Extract features at multiple resolutions
            sample_pos = round(params.pos);
            det_sample_pos = sample_pos;
            sample_scale = params.currentScaleFactor*params.scaleFactors;
            xt = extract_features(im, sample_pos, sample_scale, params.features, params.global_fparams, params.feature_extract_info);
                        
            % Project sample
            xt_proj = project_sample(xt, params.projection_matrix);
            
            % Do windowing of features
            xt_proj = cellfun(@(feat_map, cos_window) bsxfun(@times, feat_map, cos_window), xt_proj, params.cos_win, 'uniformoutput', false);
            
            % Compute the fourier series
            xtf_proj = cellfun(@cfft2, xt_proj, 'uniformoutput', false);
            
            % Interpolate features to the continuous domain
            xtf_proj = interpolate_dft(xtf_proj, params.interp1_fs, params.interp2_fs);
            
            % Compute convolution for each feature block in the Fourier domain
            % and the sum over all blocks.
            scores_fs_feat{k1} = sum(bsxfun(@times, params.hf_full{k1}, xtf_proj{k1}), 3);
            scores_fs_sum = scores_fs_feat{k1};
            for k = block_inds
                scores_fs_feat{k} = sum(bsxfun(@times, params.hf_full{k}, xtf_proj{k}), 3);
                scores_fs_sum(1+pad_sz{k}(1):end-pad_sz{k}(1), 1+pad_sz{k}(2):end-pad_sz{k}(2),1,:) = ...
                    scores_fs_sum(1+pad_sz{k}(1):end-pad_sz{k}(1), 1+pad_sz{k}(2):end-pad_sz{k}(2),1,:) + ...
                    scores_fs_feat{k};
            end
            
            % Also sum over all feature blocks.
            % Gives the fourier coefficients of the convolution response.
            scores_fs = permute(gather(scores_fs_sum), [1 2 4 3]);
            
            % Optimize the continuous score function with Newton's method.
            [trans_row, trans_col, scale_ind] = optimize_scores(scores_fs, params.newton_iterations);
            
            % Compute the translation vector in pixel-coordinates and round
            % to the closest integer pixel.
            translation_vec = [trans_row, trans_col] .* (img_support_sz./output_sz) * params.currentScaleFactor * params.scaleFactors(scale_ind);
            scale_change_factor = params.scaleFactors(scale_ind);
            
            % update position
            old_pos = params.pos;
            params.pos = sample_pos + translation_vec;
            
            if params.clamp_position
                params.pos = max([1 1], min([size(im,1) size(im,2)], params.pos));
            end
            
            % Do scale tracking with the scale filter
            if params.nScales > 0 && params.use_scale_filter
                scale_change_factor = scale_filter_track(im, params.pos, params.base_target_sz, params.currentScaleFactor, params.scale_filter, params);
            end 
            
            % Update the scale
            params.currentScaleFactor = params.currentScaleFactor * scale_change_factor;
            
            % Adjust to make sure we are not to large or to small
            if params.currentScaleFactor < params.min_scale_factor
                params.currentScaleFactor = params.min_scale_factor;
            elseif params.currentScaleFactor > params.max_scale_factor
                params.currentScaleFactor = params.max_scale_factor;
            end
            
            iter = iter + 1;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Model update step
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

         % Extract sample and init projection matrix
    if params.frame == 1
        % Extract image region for training sample
        sample_pos = round(params.pos);
        sample_scale = params.currentScaleFactor;
        xl = extract_features(im, sample_pos, params.currentScaleFactor, params.features, params.global_fparams, params.feature_extract_info);
        
        % Do windowing of features
        xlw = cellfun(@(feat_map, cos_window) bsxfun(@times, feat_map, cos_window), xl, params.cos_win, 'uniformoutput', false);
        
        % Compute the fourier series
        xlf = cellfun(@cfft2, xlw, 'uniformoutput', false);
        
        % Interpolate features to the continuous domain
        xlf = interpolate_dft(xlf, params.interp1_fs, params.interp2_fs);
        
        % New sample to be added
        xlf = compact_fourier_coeff(xlf);
        
        % Shift sample
        shift_samp = 2*pi * (params.pos - sample_pos) ./ (sample_scale * img_support_sz);
        xlf = shift_sample(xlf, shift_samp, params.kx, params.ky);
        
        % Init the projection matrix
        params.projection_matrix = init_projection_matrix(xl, params.sample_dim, params);
        
        % Project sample
        params.xlf_proj = project_sample(xlf, params.projection_matrix);
        
        clear xlw
    elseif params.learning_rate > 0
        if ~params.use_detection_sample
            % Extract image region for training sample
            sample_pos = round(params.pos);
            sample_scale = params.currentScaleFactor;
            xl = extract_features(im, sample_pos, params.currentScaleFactor, params.features, params.global_fparams, params.feature_extract_info);
            
            % Project sample
            xl_proj = project_sample(xl, params.projection_matrix);
            
            % Do windowing of features
            xl_proj = cellfun(@(feat_map, cos_window) bsxfun(@times, feat_map, cos_window), xl_proj, params.cos_win, 'uniformoutput', false);
            
            % Compute the fourier series
            xlf1_proj = cellfun(@cfft2, xl_proj, 'uniformoutput', false);
            
            % Interpolate features to the continuous domain
            xlf1_proj = interpolate_dft(xlf1_proj, params.interp1_fs, params.interp2_fs);
            
            % New sample to be added
            params.xlf_proj = compact_fourier_coeff(xlf1_proj);
        else
            if params.debug
                % Only for visualization
                xl = cellfun(@(xt) xt(:,:,:,scale_ind), xt, 'uniformoutput', false);
            end
            
            % Use the sample that was used for detection
            sample_scale = sample_scale(scale_ind);
            params.xlf_proj = cellfun(@(xf) xf(:,1:(size(xf,2)+1)/2,:,scale_ind), xtf_proj, 'uniformoutput', false);
        end
        
        % Shift the sample so that the target is centered
        shift_samp = 2*pi * (params.pos - sample_pos) ./ (sample_scale * img_support_sz);
        params.xlf_proj = shift_sample(params.xlf_proj, shift_samp, params.kx, params.ky);
    end
    
    % The permuted sample is only needed for the CPU implementation
    if ~params.use_gpu
        xlf_proj_perm = cellfun(@(xf) permute(xf, [4 3 1 2]), params.xlf_proj, 'uniformoutput', false);
    end
        
    if params.use_sample_merge
        % Update the samplesf to include the new sample. The distance
        % matrix, kernel matrix and prior weight are also updated
        if params.use_gpu
            [merged_sample, new_sample, merged_sample_id, new_sample_id, params.distance_matrix, params.gram_matrix, params.prior_weights] = ...
                update_sample_space_model_gpu(params.samplesf, params.xlf_proj, params.distance_matrix, params.gram_matrix, params.prior_weights,...
                params.num_training_samples,params);
        else
            [merged_sample, new_sample, merged_sample_id, new_sample_id, params.distance_matrix, params.gram_matrix, params.prior_weights] = ...
                update_sample_space_model(params.samplesf, xlf_proj_perm, params.distance_matrix, params.gram_matrix, params.prior_weights,...
                params.num_training_samples,params);
        end
        
        if params.num_training_samples < params.nSamples
            params.num_training_samples = params.num_training_samples + 1;
        end
    else
        % Do the traditional adding of a training sample and weight update
        % of C-COT
        [params.prior_weights, replace_ind] = update_prior_weights(params.prior_weights, gather(params.sample_weights), params.latest_ind, params.frame, params);
        params.latest_ind = replace_ind;
        
        merged_sample_id = 0;
        new_sample_id = replace_ind;
        if params.use_gpu
            new_sample = params.xlf_proj;
        else
            new_sample = xlf_proj_perm;
        end
    end
    
    if params.frame > 1 && params.learning_rate > 0 || params.frame == 1 && ~params.update_projection_matrix
        % Insert the new training sample
        for k = 1:num_feature_blocks
            if params.use_gpu
                if merged_sample_id > 0
                    params.samplesf{k}(:,:,:,merged_sample_id) = merged_sample{k};
                end
                if new_sample_id > 0
                    params.samplesf{k}(:,:,:,new_sample_id) = new_sample{k};
                end
            else
                if merged_sample_id > 0
                    params.samplesf{k}(merged_sample_id,:,:,:) = merged_sample{k};
                end
                if new_sample_id > 0
                    params.samplesf{k}(new_sample_id,:,:,:) = new_sample{k};
                end
            end
        end
    end

    params.sample_weights = cast(params.prior_weights, 'like', params.data_type);
           
    train_tracker = (params.frame < params.skip_after_frame) || (params.frames_since_last_train >= params.train_gap);
    
    if train_tracker     
        % Used for preconditioning
        new_sample_energy = cellfun(@(xlf) abs(xlf .* conj(xlf)), params.xlf_proj, 'uniformoutput', false);
        
        CG_state = [];
        if params.frame == 1
            % Initialize stuff for the filter learning
            
            % Initialize Conjugate Gradient parameters
            params.sample_energy = new_sample_energy;
           
            if params.update_projection_matrix
                % Number of CG iterations per GN iteration 
                init_CG_opts.maxit = ceil(params.init_CG_iter / params.init_GN_iter);
            
                params.hf = cell(2,1,num_feature_blocks);
                proj_energy = cellfun(@(P, yf) 2*sum(abs(yf(:)).^2) / sum(feature_dim) * ones(size(P), 'like', params.data_type), params.projection_matrix, params.yf, 'uniformoutput', false);
            else
                CG_opts.maxit = params.init_CG_iter; % Number of initial iterations if projection matrix is not updated
            
                params.hf = cell(1,1,num_feature_blocks);
            end
            
            % Initialize the filter with zeros
            for k = 1:num_feature_blocks
                params.hf{1,1,k} = zeros([filter_sz(k,1) (filter_sz(k,2)+1)/2 params.sample_dim(k)], 'like', params.data_type_complex);
            end
        else
            CG_opts.maxit = params.CG_iter;
            
            % Update the approximate average sample energy using the learning
            % rate. This is only used to construct the preconditioner.
            params.sample_energy = cellfun(@(se, nse) (1 - params.learning_rate) * se + params.learning_rate * nse, params.sample_energy, new_sample_energy, 'uniformoutput', false);
        end
        
        % Do training
        if params.frame == 1 && params.update_projection_matrix
            if params.debug
                projection_matrix_init = params.projection_matrix;
            end
            
            % Initial Gauss-Newton optimization of the filter and
            % projection matrix.
            if params.use_gpu
                [params.hf, params.projection_matrix, res_norms] = train_joint_gpu(params.hf, params.projection_matrix, xlf, params.yf, params.reg_filter, params.sample_energy, params.reg_energy, proj_energy, params, init_CG_opts);
            else
                [params.hf, params.projection_matrix, res_norms] = train_joint(params.hf, params.projection_matrix, xlf, params.yf, params.reg_filter, params.sample_energy, params.reg_energy, proj_energy, params, init_CG_opts);
            end
            
            % Re-project and insert training sample
            params.xlf_proj = project_sample(xlf, params.projection_matrix);
            for k = 1:num_feature_blocks
                if params.use_gpu
                    params.samplesf{k}(:,:,:,1) = params.xlf_proj{k};
                else
                    params.samplesf{k}(1,:,:,:) = permute(params.xlf_proj{k}, [4 3 1 2]);
                end
            end
            
            % Update the gram matrix since the sample has changed
            if strcmp(params.distance_matrix_update_type, 'exact')
                % Find the norm of the reprojected sample
                new_train_sample_norm =  0;
                
                for k = 1:num_feature_blocks
                    new_train_sample_norm = new_train_sample_norm + real(gather(2*(params.xlf_proj{k}(:)' * params.xlf_proj{k}(:))));% - reshape(xlf_proj{k}(:,end,:,:), [], 1, 1)' * reshape(xlf_proj{k}(:,end,:,:), [], 1, 1));
                end
                
                params.gram_matrix(1,1) = new_train_sample_norm;
            end
            
            if params.debug
                norm_proj_mat_init = sqrt(sum(cellfun(@(P) gather(norm(P(:))^2), projection_matrix_init)));
                norm_proj_mat = sqrt(sum(cellfun(@(P) gather(norm(P(:))^2), params.projection_matrix)));
                norm_proj_mat_change = sqrt(sum(cellfun(@(P,P2) gather(norm(P(:) - P2(:))^2), projection_matrix_init, params.projection_matrix)));
                fprintf('Norm init: %f, Norm final: %f, Matrix change: %f\n', norm_proj_mat_init, norm_proj_mat, norm_proj_mat_change / norm_proj_mat_init);
            end
        else
            % Do Conjugate gradient optimization of the filter
            if params.use_gpu
                [params.hf, res_norms, CG_state] = train_filter_gpu(params.hf, params.samplesf, params.yf, params.reg_filter, params.sample_weights, params.sample_energy, params.reg_energy, params, CG_opts, CG_state);
            else
                [params.hf, res_norms, CG_state] = train_filter(params.hf, params.samplesf, params.yf, params.reg_filter, params.sample_weights, params.sample_energy, params.reg_energy, params, CG_opts, CG_state);
            end
        end
        
        % Reconstruct the full Fourier series
        params.hf_full = full_fourier_coeff(params.hf);
        
        params.frames_since_last_train = 0;
    else
        params.frames_since_last_train = params.frames_since_last_train+1;
    end
    
    % Update the scale filter
    if params.nScales > 0 && params.use_scale_filter
        params.scale_filter = scale_filter_update(im, params.pos, params.base_target_sz, params.currentScaleFactor, params.scale_filter, params);
    end
    
    % Update the target size (only used for computing output box)
    params.target_sz = params.base_target_sz * params.currentScaleFactor;
    
    %save position and calculate FPS
%     tracking_result.center_pos = double(params.pos);
%     tracking_result.target_size = double(params.target_sz);
%     seq = report_tracking_result(seq, tracking_result);

    region = [params.pos([2,1]) - (params.target_sz([2,1]) - 1) / 2, params.target_sz([2,1])];
       
    params.time = params.time + toc();
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Visualization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % debug visualization
    if params.debug
        figure(20)
%         set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        subplot_cols = num_feature_blocks;
        subplot_rows = 3;%ceil(feature_dim/subplot_cols);
        for disp_layer = 1:num_feature_blocks;
            subplot(subplot_rows,subplot_cols,disp_layer);
            imagesc(mean(abs(sample_fs(conj(params.hf_full{disp_layer}))), 3)); 
            colorbar;
            axis image;
            subplot(subplot_rows,subplot_cols,disp_layer+subplot_cols);
            imagesc(mean(abs(xl{disp_layer}), 3)); 
            colorbar;
            axis image;
            if params.frame > 1
                subplot(subplot_rows,subplot_cols,disp_layer+2*subplot_cols);
                imagesc(fftshift(sample_fs(scores_fs_feat{disp_layer}(:,:,1,scale_ind))));
                colorbar;
                axis image;
            end
        end
        
        if train_tracker
            residuals_pcg = [residuals_pcg; res_norms];
            res_start_ind = max(1, length(residuals_pcg)-300);
            figure(99);plot(res_start_ind:length(residuals_pcg), residuals_pcg(res_start_ind:end));
            axis([res_start_ind, length(residuals_pcg), 0, min(max(residuals_pcg(res_start_ind:end)), 0.2)]);
        end
    end
    
    % visualization
    if params.visualization
        rect_position_vis = [params.pos([2,1]) - (params.target_sz([2,1]) - 1)/2, params.target_sz([2,1])];
        im_to_show = double(im)/255;
        if size(im_to_show,3) == 1
            im_to_show = repmat(im_to_show, [1 1 3]);
        end
        if params.frame == 1,  %first frame, create GUI
            params.fig_handle = figure('Name', 'Tracking');
%             set(fig_handle, 'Position', [100, 100, size(im,2), size(im,1)]);
            imagesc(im_to_show);
            hold on;
            rectangle('Position',rect_position_vis, 'EdgeColor','g', 'LineWidth',2);
            text(10, 10, int2str(params.frame), 'color', [0 1 1]);
            hold off;
            axis off;axis image;set(gca, 'Units', 'normalized', 'Position', [0 0 1 1])
            
%             output_name = 'Video_name';
%             opengl software;
%             writer = VideoWriter(output_name, 'MPEG-4');
%             writer.FrameRate = 5;
%             open(writer);
        else
            % Do visualization of the sampled confidence scores overlayed
            resp_sz = round(img_support_sz*params.currentScaleFactor*params.scaleFactors(scale_ind));
            xs = floor(det_sample_pos(2)) + (1:resp_sz(2)) - floor(resp_sz(2)/2);
            ys = floor(det_sample_pos(1)) + (1:resp_sz(1)) - floor(resp_sz(1)/2);
            
            % To visualize the continuous scores, sample them 10 times more
            % dense than output_sz. 
            sampled_scores_display = fftshift(sample_fs(scores_fs(:,:,scale_ind), 10*output_sz));
            
            figure(params.fig_handle);
%                 set(fig_handle, 'Position', [100, 100, 100+size(im,2), 100+size(im,1)]);
            imagesc(im_to_show);
            hold on;
            resp_handle = imagesc(xs, ys, sampled_scores_display); colormap hsv;
            alpha(resp_handle, 0.5);
            rectangle('Position',rect_position_vis, 'EdgeColor','g', 'LineWidth',2);
            text(10, 10, int2str(params.frame), 'color', [0 1 1]);
            hold off;
            
%                 axis off;axis image;set(gca, 'Units', 'normalized', 'Position', [0 0 1 1])
        end
        
        drawnow
%         if frame > 1
%             if frame < inf
%                 writeVideo(writer, getframe(gcf));
%             else
%                 close(writer);
%             end
%         end
%          pause
    end

% close(writer);

end