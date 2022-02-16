function [xs, ys, xt, yt, row] = readpolygon(filename)
data = load(filename);

[row,col]=size(data);
xs = data(:,1);
ys = data(:,2);
xt = data(:,3);
yt = data(:,4);