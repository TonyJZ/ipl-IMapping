data = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\lineSegs_beforeArr.txt')
X=data(:,1)
Y=data(:,2)

data0 = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_136_hole_00.txt')
X0=data0(:,1)
Y0=data0(:,2)

data1 = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_136_hole_01.txt')
X1=data1(:,1)
Y1=data1(:,2)

data2 = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_136_hole_02.txt')
X2=data2(:,1)
Y2=data2(:,2)

data3 = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_136_hole_03.txt')
X3=data3(:,1)
Y3=data3(:,2)

data4 = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\polygons\face_136_hole_04.txt')
X4=data4(:,1)
Y4=data4(:,2)

%data5 = load('D:\code_indoor_location\sample_data\sample1\ref_dataset\result\face_007_hole_05.txt')
%X5=data5(:,1)
%Y5=data5(:,2)

figure
plot(X,Y,'b-o')
%plot(X,Y,'b--o',X0,Y0,'r--*',X1,Y1,'r--*',X2,Y2,'r--*')
%plot(X,Y,'b--o',X0,Y0,'r--*',X1,Y1,'r--*',X2,Y2,'r--*',X3,Y3,'r--*',X4,Y4,'r--*')
axis equal
hold on