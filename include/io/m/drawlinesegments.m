figure
hold on;

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\lineSegs_beforeArr.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
%    plot(x,y,'k-o');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\lineArr_beformerge.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
    plot(x,y,'k--*');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_123.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
    plot(x,y,'r--*');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_048.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
    plot(x,y,'r--*');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_070.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
    plot(x,y,'g-*');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_093.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
%    plot(x,y,'g--o');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_009_hole_00.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
%    plot(x,y,'r--*');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_009_hole_01.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
%    plot(x,y,'r--*');
end

[xs, ys, xt, yt, row] = readpolygon('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_009_hole_02.txt');
for i=1:row
    x = [xs(i), xt(i)];
    y = [ys(i), yt(i)];
%    plot(x,y,'r--*');
end