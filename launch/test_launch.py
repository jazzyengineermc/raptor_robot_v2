<launch>

	<!--<node pkg="rosserial_python" type="serial_node.py" name="rosserial" output="screen" />-->

	<!-- taken from intel realsense doco -->
	<node pkg="imu_filter_madgwick" type="imu_filter_node" name="ImuFilter">
		<param name="use_mag" type="bool" value="false" />
		<param name="_publish_tf" type="bool" value="false" />
		<param name="_world_frame" type="string" value="enu" />
		<remap from="/imu/data_raw" to="/d435i/camera/imu"/>
		<remap from="/imu/data" to="/rtabmap/imu"/>
	</node>

	<!-- taken from intel realsense doco -->
	<include file="$(find realsense2_camera)/launch/rs_launch.py">
		<arg name="align_depth" value="true"/>
		<arg name="enable_gyro" value="true"/>
		<arg name="enable_accel" value="true"/>
		<arg name="unite_imu_method" value="copy"/>
	</include>

  <group ns="rtabmap">
    <node pkg="nodelet" type="nodelet" name="rgbd_sync" args="standalone rtabmap_ros/rgbd_sync" output="screen">
      <remap from="rgb/image"        to="/d435i/camera/color/image_raw"/>
      <remap from="depth/image"      to="/d435i/camera/aligned_depth_to_color/image_raw"/>
      <remap from="rgb/camera_info"  to="/d435i/camera/color/camera_info"/>
      <remap from="rgbd_image"       to="/d435i/camera/rgbd"/> <!-- output -->
      
      <!-- Should be true for not synchronized camera topics 
           (e.g., false for kinectv2, zed, realsense, true for xtion, kinect360)-->
      <param name="approx_sync"       value="false"/> 
    </node>

    <!-- Odometry -->
    <node pkg="rtabmap_ros" type="rgbd_odometry" name="rgbd_odometry" output="screen">
      <param name="subscribe_rgbd" type="bool"   value="true"/>
      <param name="frame_id"       type="string" value="base_link"/>
      <remap from="rgbd_image" to="/d435i/camera/rgbd"/>
    </node>

    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">
          <param name="frame_id"        type="string" value="base_link"/>
          <param name="subscribe_depth" type="bool"   value="false"/>
          <param name="subscribe_rgbd"  type="bool"   value="true"/>

          <remap from="odom"       to="odom"/>
          <remap from="rgbd_image" to="/d435i/camera/rgbd"/>

          <param name="queue_size"  type="int"  value="10"/>
          <param name="approx_sync" type="bool" value="false"/>

          <!-- RTAB-Map's parameters -->
          <param name="RGBD/AngularUpdate"        type="string" value="0.01"/>
          <param name="RGBD/LinearUpdate"         type="string" value="0.01"/>
          <param name="RGBD/OptimizeFromGraphEnd" type="string" value="false"/>
    </node>
  </group>
  
</launch>