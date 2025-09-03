# lidar_camera_projection

LiDAR 포인트를 Cam 이미지 위에 projection하는 코드.
LiDAR to IMU extrinsic, Cam to IMU extrinsic으로 Lidar to Cam extrinsic을 구해서 projection한다.

# 설정 (config/extrinsics.yaml)
아래 코드 config 알맞게 수정

  # Camera -> IMU (4x4)
  cam_to_imu:
    T: [[-0.0052379815, -0.9999803988, -0.0034301065, -0.0086779584],
        [-0.0398738980,  0.0036362859, -0.9991981033, -0.0288010439],
        [ 0.9991909907, -0.0050970095, -0.0398921632, -0.1167463763],
        [ 0.0,           0.0,           0.0,           1.0]]
  
  # LiDAR -> IMU (4x4)
  lidar_to_imu:
    T: [[ 0.688536,  0.004840,  0.725186,  0.110294],
        [-0.010386,  0.999941,  0.003187, -0.002712],
        [-0.725128, -0.009726,  0.688545,  0.057693],
        [ 0.0,       0.0,       0.0,       1.0]]
  
  # 카메라 파라미터
  camera:
    # CameraInfo 대신 아래 값을 강제 사용하려면 false
    use_camera_info: false
  
    # pinhole intrinsics = [fx, fy, cx, cy]
    intrinsics: [634.2358506386614, 634.2909384110677, 631.4370690331413, 374.41418400883066]
  
    # radtan 왜곡: [k1, k2, p1, p2]  (OpenCV에서는 k3=0으로 자동 패딩)
    distortion: [-0.049879140602722534, 0.04189503052568046, -0.0002581078253831348, -0.00036332730935522637]
  
    # 이미지 해상도 (raw 이미지와 동일해야 정확)
    resolution: [1280, 720]
  
  # 시각화 옵션
  max_range: 60.0
  point_size: 2
  draw_alpha: 0.9
