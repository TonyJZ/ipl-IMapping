function [location, params] = ECO_init(im, region)

% Feature specific parameters
hog_params.cell_size = 4;
hog_params.compressed_dim = 10;

% grayscale_params.colorspace='gray';
% grayscale_params.cell_size = 1;
% 
% cn_params.tablename = 'CNnorm';
% cn_params.useForGray = false;
% cn_params.cell_size = 4;
% cn_params.compressed_dim = 3;
% 
% ic_params.tablename = 'intensityChannelNorm6';
% ic_params.useForColor = false;
% ic_params.cell_size = 4;
% ic_params.compressed_dim = 3;

cnn_params.nn_name = 'imagenet-vgg-m-2048.mat'; % Name of the network
cnn_params.output_layer = [3 14];               % Which layers to use
cnn_params.downsample_factor = [2 1];           % How much to downsample each output layer
cnn_params.compressed_dim = [16 64];            % Compressed dimensionality of each output layer
cnn_params.input_size_mode = 'adaptive';        % How to choose the sample size
cnn_params.input_size_scale = 1;                % Extra scale factor of the input samples to the network (1 is no scaling)

% Which features to include
params.t_features = {
    struct('getFeature',@get_cnn_layers, 'fparams',cnn_params),...
    struct('getFeature',@get_fhog,'fparams',hog_params),...
    ...struct('getFeature',@get_colorspace, 'fparams',grayscale_params),...
    ...struct('getFeature',@get_table_feature, 'fparams',cn_params),...
    ...struct('getFeature',@get_table_feature, 'fparams',ic_params),...
};

% Global feature parameters1s
params.t_global.normalize_power = 2;    % Lp normalization with this p
params.t_global.normalize_size = true;  % Also normalize with respect to the spatial size of the feature
params.t_global.normalize_dim = true;   % Also normalize with respect to the dimensionality of the feature

% Image sample parameters
params.search_area_shape = 'square';    % The shape of the samples
params.search_area_scale = 4.5;         % The scaling of the target size to get the search area
params.min_image_sample_size = 200^2;   % Minimum area of image samples
params.max_image_sample_size = 250^2;   % Maximum area of image samples

% Detection parameters
params.refinement_iterations = 1;       % Number of iterations used to refine the resulting position in a frame
params.newton_iterations = 5;           % The number of Newton iterations used for optimizing the detection score
params.clamp_position = false;          % Clamp the target position to be inside the image

% Learning parameters
params.output_sigma_factor = 1/12;		% Label function sigma
params.learning_rate = 0.009;	 	 	% Learning rate
params.nSamples = 50;                   % Maximum number of stored training samples
params.sample_replace_strategy = 'lowest_prior';    % Which sample to replace when the memory is full
params.lt_size = 0;                     % The size of the long-term memory (where all samples have equal weight)
params.train_gap = 5;                   % The number of intermediate frames with no training (0 corresponds to training every frame)
params.skip_after_frame = 1;            % After which frame number the sparse update scheme should start (1 is directly)
params.use_detection_sample = true;     % Use the sample that was extracted at the detection stage also for learning

% Factorized convolution parameters
params.use_projection_matrix = true;    % Use projection matrix, i.e. use the factorized convolution formulation
params.update_projection_matrix = true; % Whether the projection matrix should be optimized or not
params.proj_init_method = 'pca';        % Method for initializing the projection matrix
params.projection_reg = 5e-8;	 	 	% Regularization paremeter of the projection matrix

% Generative sample space model parameters
params.use_sample_merge = true;                 % Use the generative sample space model to merge samples
params.sample_merge_type = 'Merge';             % Strategy for updating the samples
params.distance_matrix_update_type = 'exact';   % Strategy for updating the distance matrix

% Conjugate Gradient parameters
params.CG_iter = 5;                     % The number of Conjugate Gradient iterations in each update after the first frame
params.init_CG_iter = 10*15;            % The total number of Conjugate Gradient iterations used in the first frame
params.init_GN_iter = 10;               % The number of Gauss-Newton iterations used in the first frame (only if the projection matrix is updated)
params.CG_use_FR = false;               % Use the Fletcher-Reeves (true) or Polak-Ribiere (false) formula in the Conjugate Gradient
params.CG_standard_alpha = true;        % Use the standard formula for computing the step length in Conjugate Gradient
params.CG_forgetting_rate = 75;	 	 	% Forgetting rate of the last conjugate direction
params.precond_data_param = 0.3;	 	% Weight of the data term in the preconditioner
params.precond_reg_param = 0.015;	 	% Weight of the regularization term in the preconditioner 
params.precond_proj_param = 35;	 	 	% Weight of the projection matrix part in the preconditioner

% Regularization window parameters
params.use_reg_window = true;           % Use spatial regularization or not
params.reg_window_min = 1e-4;			% The minimum value of the regularization window
params.reg_window_edge = 10e-3;         % The impact of the spatial regularization
params.reg_window_power = 2;            % The degree of the polynomial to use (e.g. 2 is a quadratic window)
params.reg_sparsity_threshold = 0.05;   % A relative threshold of which DFT coefficients that should be set to zero

% Interpolation parameters
params.interpolation_method = 'bicubic';    % The kind of interpolation kernel
params.interpolation_bicubic_a = -0.75;     % The parameter for the bicubic interpolation kernel
params.interpolation_centering = true;      % Center the kernel at the feature sample
params.interpolation_windowing = false;     % Do additional windowing on the Fourier coefficients of the kernel

% Scale parameters for the translation model
% Only used if: params.use_scale_filter = false
params.number_of_scales = 5;            % Number of scales to run the detector
params.scale_step = 1.02;               % The scale factor

% Scale filter parameters
% Only used if: params.use_scale_filter = true
params.use_scale_filter = false;          % Use the fDSST scale filter or not (for speed)
% params.scale_sigma_factor = 1/16;       % Scale label function sigma
% params.scale_learning_rate = 0.025;     % Scale filter learning rate
% params.number_of_scales_filter = 17;    % Number of scales
% params.number_of_interp_scales = 33;    % Number of interpolated scales
% params.scale_model_factor = 1.0;        % Scaling of the scale model
% params.scale_step_filter = 1.02;        % The scale factor for the scale filter
% params.scale_model_max_area = 32*16;    % Maximume area for the scale sample patch
% params.scale_feature = 'HOG4';          % Features for the scale filter (only HOG4 supported)
% params.s_num_compressed_dim = 'MAX';    % Number of compressed feature dimensions in the scale filter
% params.lambda = 1e-2;					  % Scale filter regularization
% params.do_poly_interp = true;           % Do 2nd order polynomial interpolation to obtain more accurate scale

% Visualization
params.visualization = 1;               % Visualiza tracking and detection scores
params.debug = 0;                       % Do full debug visualization

if ~isdeployed
   params.visualization = 1;
end

% GPU
params.use_gpu = false;                 % Enable GPU or not
params.gpu_id = [];                     % Set the GPU id, or leave empty to use default

% Initialize
% params.seq = seq;


% initialize ECO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get sequence info
% [seq, im] = get_sequence_info(params.seq);
% params = rmfield(params, 'seq');
% if isempty(im)
%    seq.rect_position = [];
%    [seq, results] = get_sequence_results(seq);
%    return;
% end

% Init position
% If the provided region is a polygon ...
if numel(region) > 4
% Init with an axis aligned bounding box with correct area and center
% coordinate
    cx = mean(region(1:2:end));
    cy = mean(region(2:2:end));
    x1 = min(region(1:2:end));
    x2 = max(region(1:2:end));
    y1 = min(region(2:2:end));
    y2 = max(region(2:2:end));
    A1 = norm(region(1:2) - region(3:4)) * norm(region(3:4) - region(5:6));
    A2 = (x2 - x1) * (y2 - y1);
    s = sqrt(A1/A2);
    w = s * (x2 - x1) + 1;
    h = s * (y2 - y1) + 1;
else
    cx = region(1) + (region(3) - 1)/2;
    cy = region(2) + (region(4) - 1)/2;
    w = region(3);
    h = region(4);
end
    
init_c = [cy cx];
init_sz = [h w];
% im_size = size(im);

params.init_pos = init_c;
params.pos = init_c;
params.init_sz = init_sz; %min(max(round(init_sz), [1 1]), im_size(1:2));
params.region = region;
params.target_sz = params.init_sz;

% output location
location = [init_c([2,1]) - (params.init_sz([2,1])-1)/2, params.init_sz([2,1])];

% Feature settings
params.features = params.t_features;

% Set default parameters
params = init_default_params(params);

% Global feature parameters
if isfield(params, 't_global')
    params.global_fparams = params.t_global;
else
    params.global_fparams = [];
end
params.global_fparams.use_gpu = params.use_gpu;
params.global_fparams.gpu_id = params.gpu_id;

% Correct max number of samples
% params.nSamples = min(params.nSamples, seq.num_frames);
params.nSamples = min(params.nSamples, Inf);

% Define data types
if params.use_gpu
    params.data_type = zeros(1, 'single', 'gpuArray');
else
    params.data_type = zeros(1, 'single');
end
params.data_type_complex = complex(params.data_type);
params.global_fparams.data_type = params.data_type;

init_target_sz = params.target_sz;

% Check if color image
if size(im,3) == 3
    if all(all(im(:,:,1) == im(:,:,2)))
        is_color_image = false;
    else
        is_color_image = true;
    end
else
    is_color_image = false;
end

if size(im,3) > 1 && is_color_image == false
    im = im(:,:,1);
end

% Check if mexResize is available and show warning otherwise.
params.use_mexResize = true;
params.global_fparams.use_mexResize = true;
try
    [~] = mexResize(ones(5,5,3,'uint8'), [3 3], 'auto');
catch err
    warning('ECO:tracker', 'Error when using the mexResize function. Using Matlab''s interpolation function instead, which is slower.\nTry to run the compile script in "external_libs/mexResize/".\n\nThe error was:\n%s', getReport(err));
    params.use_mexResize = false;
    params.global_fparams.use_mexResize = false;
end

% Calculate search area and initial scale factor
search_area = prod(init_target_sz * params.search_area_scale);
if search_area > params.max_image_sample_size
    params.currentScaleFactor = sqrt(search_area / params.max_image_sample_size);
elseif search_area < params.min_image_sample_size
    params.currentScaleFactor = sqrt(search_area / params.min_image_sample_size);
else
    params.currentScaleFactor = 1.0;
end

% target size at the initial scale
params.base_target_sz = params.target_sz / params.currentScaleFactor;

% window size, taking padding into account
switch params.search_area_shape
    case 'proportional'
        img_sample_sz = floor( params.base_target_sz * params.search_area_scale);     % proportional area, same aspect ratio as the target
    case 'square'
        img_sample_sz = repmat(sqrt(prod(params.base_target_sz * params.search_area_scale)), 1, 2); % square area, ignores the target aspect ratio
    case 'fix_padding'
        img_sample_sz = params.base_target_sz + sqrt(prod(params.base_target_sz * params.search_area_scale) + (params.base_target_sz(1) - params.base_target_sz(2))/4) - sum(params.base_target_sz)/2; % const padding
    case 'custom'
        img_sample_sz = [params.base_target_sz(1)*2 params.base_target_sz(2)*2]; % for testing
end

[params.features, params.global_fparams, params.feature_info] = init_features(params.features, params.global_fparams, is_color_image, img_sample_sz, 'odd_cells');

% Set feature info
img_support_sz = params.feature_info.img_support_sz;
feature_sz = params.feature_info.data_sz;
feature_dim = params.feature_info.dim;
num_feature_blocks = length(feature_dim);

% Get feature specific parameters
feature_params = init_feature_params(params.features, params.feature_info);
params.feature_extract_info = get_feature_extract_info(params.features);

% Set the sample feature dimension
if params.use_projection_matrix
    params.sample_dim = feature_params.compressed_dim;
else
    params.sample_dim = feature_dim;
end

% Size of the extracted feature maps
feature_sz_cell = permute(mat2cell(feature_sz, ones(1,num_feature_blocks), 2), [2 3 1]);

% Number of Fourier coefficients to save for each filter layer. This will
% be an odd number.
filter_sz = feature_sz + mod(feature_sz+1, 2);
filter_sz_cell = permute(mat2cell(filter_sz, ones(1,num_feature_blocks), 2), [2 3 1]);

% The size of the label function DFT. Equal to the maximum filter size.
[output_sz, k1] = max(filter_sz, [], 1);
k1 = k1(1);

% Get the remaining block indices
% block_inds = 1:num_feature_blocks;
% block_inds(k1) = [];

% How much each feature block has to be padded to the obtain output_sz
% pad_sz = cellfun(@(filter_sz) (output_sz - filter_sz) / 2, filter_sz_cell, 'uniformoutput', false);

% Compute the Fourier series indices and their transposes
ky = cellfun(@(sz) (-ceil((sz(1) - 1)/2) : floor((sz(1) - 1)/2))', filter_sz_cell, 'uniformoutput', false);
params.ky = ky;
kx = cellfun(@(sz) -ceil((sz(2) - 1)/2) : 0, filter_sz_cell, 'uniformoutput', false);
params.kx = kx;

% construct the Gaussian label function using Poisson formula
sig_y = sqrt(prod(floor(params.base_target_sz))) * params.output_sigma_factor * (output_sz ./ img_support_sz);
yf_y = cellfun(@(ky) single(sqrt(2*pi) * sig_y(1) / output_sz(1) * exp(-2 * (pi * sig_y(1) * ky / output_sz(1)).^2)), ky, 'uniformoutput', false);
yf_x = cellfun(@(kx) single(sqrt(2*pi) * sig_y(2) / output_sz(2) * exp(-2 * (pi * sig_y(2) * kx / output_sz(2)).^2)), kx, 'uniformoutput', false);
yf = cellfun(@(yf_y, yf_x) cast(yf_y * yf_x, 'like', params.data_type), yf_y, yf_x, 'uniformoutput', false);
params.yf = yf;

% construct cosine window
cos_window = cellfun(@(sz) hann(sz(1)+2)*hann(sz(2)+2)', feature_sz_cell, 'uniformoutput', false);
cos_window = cellfun(@(cos_window) cast(cos_window(2:end-1,2:end-1), 'like', params.data_type), cos_window, 'uniformoutput', false);
params.cos_win = cos_window;

% Compute Fourier series of interpolation function
[params.interp1_fs, params.interp2_fs] = cellfun(@(sz) get_interp_fourier(sz, params), filter_sz_cell, 'uniformoutput', false);

% Get the reg_window_edge parameter
reg_window_edge = {};
for k = 1:length(params.features)
    if isfield(params.features{k}.fparams, 'reg_window_edge')
        reg_window_edge = cat(3, reg_window_edge, permute(num2cell(params.features{k}.fparams.reg_window_edge(:)), [2 3 1]));
    else
        reg_window_edge = cat(3, reg_window_edge, cell(1, 1, length(params.features{k}.fparams.nDim)));
    end
end

% Construct spatial regularization filter
reg_filter = cellfun(@(reg_window_edge) get_reg_filter(img_support_sz, params.base_target_sz, params, reg_window_edge), reg_window_edge, 'uniformoutput', false);
params.reg_filter = reg_filter;

% Compute the energy of the filter (used for preconditioner)
params.reg_energy = cellfun(@(reg_filter) real(reg_filter(:)' * reg_filter(:)), reg_filter, 'uniformoutput', false);

if params.use_scale_filter
    [params.nScales, params.scale_step, params.scaleFactors, params.scale_filter, params] = init_scale_filter(params);
else
    % Use the translation filter to estimate the scale.
    params.nScales = params.number_of_scales;
    params.scale_step = params.scale_step;
    scale_exp = (-floor((params.nScales-1)/2):ceil((params.nScales-1)/2));
    params.scaleFactors = params.scale_step .^ scale_exp;
end

if params.nScales > 0
    %force reasonable scale changes
    params.min_scale_factor = params.scale_step ^ ceil(log(max(5 ./ img_support_sz)) / log(params.scale_step));
    params.max_scale_factor = params.scale_step ^ floor(log(min([size(im,1) size(im,2)] ./ params.base_target_sz)) / log(params.scale_step));
end

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

% seq.time = 0;
params.time = 0;

% Initialize and allocate
params.prior_weights = zeros(params.nSamples,1, 'single');
params.sample_weights = cast(params.prior_weights, 'like', params.data_type);
params.samplesf = cell(1, 1, num_feature_blocks);
if params.use_gpu
    % In the GPU version, the data is stored in a more normal way since we
    % dont have to use mtimesx.
    for k = 1:num_feature_blocks
        params.samplesf{k} = zeros(filter_sz(k,1),(filter_sz(k,2)+1)/2,params.sample_dim(k),params.nSamples, 'like', params.data_type_complex);
    end
else
    for k = 1:num_feature_blocks
        params.samplesf{k} = zeros(params.nSamples,params.sample_dim(k),filter_sz(k,1),(filter_sz(k,2)+1)/2, 'like', params.data_type_complex);
    end
end

% Allocate
% scores_fs_feat = cell(1,1,num_feature_blocks);

% Distance matrix stores the square of the euclidean distance between each pair of
% samples. Initialise it to inf
params.distance_matrix = inf(params.nSamples, 'single');

% Kernel matrix, used to update distance matrix
params.gram_matrix = inf(params.nSamples, 'single');

params.latest_ind = [];
params.frames_since_last_train = inf;
params.num_training_samples = 0;

% Find the minimum allowed sample weight. Samples are discarded if their weights become lower 
params.minimum_sample_weight = params.learning_rate*(1-params.learning_rate)^(2*params.nSamples);

% res_norms = [];
% residuals_pcg = [];

params.frame = 1;

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
        
        if params.frame == 1
            % Initialize stuff for the filter learning
            
            % Initialize Conjugate Gradient parameters
            params.sample_energy = new_sample_energy;
            CG_state = [];
            
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
            
            figure(fig_handle);
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
    
end