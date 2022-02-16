cd ..\..\bin\vs2015\debug
echo point cloud classification...
utest_iplclassifier.exe D:\data_indoor\parking_lot\raw.ply D:\data_indoor\parking_lot\result_test
echo classification done!

echo wall detection...
utest_model_fitting.exe D:\data_indoor\parking_lot\result_test\wall_pts.pcd D:\data_indoor\parking_lot\result_test\eRansac --doRANSAC
echo wall detection done!

echo wall filtering...
utest_wall_refinement.exe D:\data_indoor\parking_lot\result_test\eRansac .pcd D:\data_indoor\parking_lot\result_test\eRansac\filtered
echo wall filtering done!

echo 2D reconstruction...
utest_floormapRec.exe D:\data_indoor\parking_lot\result_test\eRansac\filtered .pcd D:\data_indoor\parking_lot\dummy.pcd D:\data_indoor\parking_lot\result_test\polygons
echo 2D reconstruction done!

echo polygon simplification...
utest_polygon_simplification.exe D:\data_indoor\parking_lot\result_test\polygons\all_polygons.shp D:\data_indoor\parking_lot\result_test\polygons\all_polygons_simplified.shp 0.5
echo polygon simplification done!

echo all done!