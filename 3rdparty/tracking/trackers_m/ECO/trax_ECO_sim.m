function trax_ECO(do_cleanup)

if ~isdeployed
    setup_paths();
end

if nargin < 3
    do_cleanup = true;
end

% Load initalization data
video_path = 'D:/code_ipl/3rdparty/tracking/source/ECO-master/sequences/Human7';
[seq, ground_truth] = load_video_info(video_path);


% Initialize the tracker
im = imread(seq.s_frames{1});
[res, parameters] = ECO_init(im, seq.init_rect);

%figure
LineStyle = '-';
cl = [0,1,0];
LineWidth = 2;
% imshow(im);
% rectangle('Position', res, 'EdgeColor', cl, 'LineWidth', LineWidth,'LineStyle',LineStyle);

 % Iterate through images
for i = 2:length(seq.s_frames)
    im = imread(seq.s_frames{i});
%     parameters.frame = i;
    [parameters, region] = ECO_update(parameters, im);

    %figure
%     imshow(im);
%     rectangle('Position', region, 'EdgeColor', cl, 'LineWidth', LineWidth,'LineStyle',LineStyle);
    
end;

if isfield(parameters, 'time')
    parameters.fps = (parameters.frame-1) / parameters.time;
else
    parameters.fps = NaN;
end

end