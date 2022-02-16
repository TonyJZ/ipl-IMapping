test dataset: 
	(1) ref. data:            /iplTestData/TargetLocalization/ref_dataset/
        (2) target survey data:   /iplTestData/TargetLocalization/target_dataset1/

1. ref. map generating
   (1) point cloud classification
       utest_iplclassifier D:\iplTestData\TargetLocalization\ref_dataset\ref_pointcloud_outlierRemoval.pcd D:\iplTestData\TargetLocalization\ref_dataset\result

   (2) wall extraction
     utest_wall_extraction D:\iplTestData\TargetLocalization\target_dataset1\result\vertical_pts.pcd D:\iplTestData\TargetLocalization\target_dataset1\result\eRansac\wall --doRANSAC

   (3) ceiling extraction
     utest_ceiling_extraction D:\iplTestData\TargetLocalization\target_dataset1\result\horizontal_pts.pcd D:\iplTestData\TargetLocalization\target_dataset1\result\eRansac\ceiling --doRANSAC           

########delete, 需要测试intersection结果和2D registration 结果是否有影响
   (3) wall refinement
     utest_wall_refinement D:\iplTestData\TargetLocalization\target_dataset1\result\eRansac .pcd D:\iplTestData\TargetLocalization\target_dataset1\result\filtered
#########

   (4) intersection graph extraction
     utest_intersection_graph D:\iplTestData\TargetLocalization\target_dataset1\result\filtered .pcd D:\iplTestData\TargetLocalization\target_dataset1\result\merged_planes.ip2d --outputforcheck
     

2. target localizaiton
   (1) point cloud classification
   (2) wall extraction
   (3) ceiling extraction
   (4) intersection graph extraction
   
   (5) 2D registration
   utest_2Dregist D:\iplTestData\TargetLocalization\ref_dataset\result\merged_planes.ip2d D:\iplTestData\TargetLocalization\target_dataset1\result\merged_planes.ip2d D:\iplTestData\TargetLocalization\target_dataset1\result\inti_trans.txt --search_radius 10.0

   (6) wall filtering
utest_plane_filtering.exe D:\iplTestData\TargetLocalization\hard_reg_test\T3\result\eRansac\wall .pcd  D:\iplTestData\TargetLocalization\hard_reg_test\T3\result\filtered\wall --occupied_area 2.0  --sparse_degree 0.25



   (7) 3D registration
   utest_3Dreigst D:\iplTestData\TargetLocalization\ref_dataset\result\eRansac\wall D:\iplTestData\TargetLocalization\ref_dataset\result\eRansac\ceiling D:\iplTestData\TargetLocalization\target_dataset1\result\filtered\wall D:\iplTestData\TargetLocalization\target_dataset1\result\eRansac\ceiling .pcd D:\iplTestData\TargetLocalization\target_dataset1\result\inti_trans.txt D:\iplTestData\TargetLocalization\target_dataset1\result\final_trans.txt --search_radius 2.0 --resampling_rate 0.1

  (8) data transformation
  utest_pointcloud_geoTrans D:\iplTestData\TargetLocalization\target_dataset1\result\wall_pts.pcd D:\iplTestData\TargetLocalization\target_dataset1\result\wall_pts_trans3D.pcd D:\iplTestData\TargetLocalization\target_dataset1\result\final_trans.txt

3. map update
  (1) candidate change extraction
  utest_CandidateChangeExtraction D:\iplTestData\TargetLocalization\mapupdate\ref\eRansac\ceiling D:\iplTestData\TargetLocalization\hard_reg_test\T2\result\filtered\ceiling .pcd D:\iplTestData\TargetLocalization\hard_reg_test\T2\result\final_trans.txt D:\iplTestData\TargetLocalization\hard_reg_test\CCL\T2\ceiling --distance_th 0.2


  (2) Global change detection
  utest_GlobalChangeDetection --create_gcm D:\iplTestData\TargetLocalization\hard_reg_test\changes\ceilings.gcm --origin 1 1 0 --voxel_size 0.3 0.3 0.3 --reliability_Th 3 --save_csm D:\iplTestData\TargetLocalization\hard_reg_test\changes\ceilings.csm --iccl D:\iplTestData\TargetLocalization\hard_reg_test\CCL\T2\ceiling --iccl D:\iplTestData\TargetLocalization\hard_reg_test\CCL\T3\ceiling --iccl D:\iplTestData\TargetLocalization\hard_reg_test\CCL\T4\ceiling --iccl D:\iplTestData\TargetLocalization\hard_reg_test\CCL\T5\ceiling --iccl D:\iplTestData\TargetLocalization\hard_reg_test\CCL\T6\ceiling

  
  (3) map update
  utest_map_update --iCSM D:\iplTestData\TargetLocalization\hard_reg_test\changes\ceilings.csm --oDir D:\iplTestData\TargetLocalization\hard_reg_test\changes\mapupdate\ceiling

