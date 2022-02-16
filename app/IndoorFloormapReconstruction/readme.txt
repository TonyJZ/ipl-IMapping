test data: ref_pointcloud_outlierRemoval.pcd

working flow:
1. classification
   ./utest_iplclassifier.exe ../iplTestData/IndoorMapping/office/ref_pointcloud_outlierRemoval.pcd ../iplTestData/IndoorMapping/office/result

2. wall detection
  ./utest_model_fitting.exe ../iplTestData/IndoorMapping/office/result/wall_pts.pcd ../iplTestData/IndoorMapping/office/result/eRansac --doRANSAC

3. wall refinement
  ./utest_wall_refinement.exe ../iplTestData/IndoorMapping/office/result/eRansac .pcd ../iplTestData/IndoorMapping/office/result/filtered

4. floormap reconstruction
  ./utest_floormapRec.exe ../iplTestData/IndoorMapping/office/result/filtered .pcd ../iplTestData/IndoorMapping/office/result/nonwall_pts.pcd ../iplTestData/IndoorMapping/office/result/polygons


5. polygon simplification
  ./utest_polygon_simplification.exe ../iplTestData/IndoorMapping/office/result/polygons/all_polygons.shp ../iplTestData/IndoorMapping/office/result/polygons/all_polygons_simplified.shp 0.5