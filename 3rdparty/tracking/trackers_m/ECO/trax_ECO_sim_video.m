function trax_ECO(do_cleanup)

if ~isdeployed
    setup_paths();
end

if nargin < 3
    do_cleanup = true;
end

% Load initalization data
video_path = 'D:\data_tracking\MOT16\MOT16-07.mp4';
% [seq, ground_truth] = load_video_info(video_path);

v = VideoReader(video_path);
frame = readFrame(v);
figure, imshow(frame);  
[A,rect] = imcrop(frame);

% Initialize the tracker
im = frame;
[res, parameters] = ECO_init(im, rect);

%figure
LineStyle = '-';
cl = [0,1,0];
LineWidth = 2;
% imshow(im);
% rectangle('Position', res, 'EdgeColor', cl, 'LineWidth', LineWidth,'LineStyle',LineStyle);

 % Iterate through images
while hasFrame(v)
    im = readFrame(v);
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