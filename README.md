# 重要参数修改
~~~
./fast_lio/src/laserMapping.cpp
~~~
~~~
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br,const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubFastLioYaw_)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "base_link";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.header.stamp = odomAftMapped.header.stamp; 
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x + 0.175;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);

    tf2::Quaternion quat;
    quat.setX(trans.transform.rotation.x);
    quat.setY(trans.transform.rotation.y);
    quat.setZ(trans.transform.rotation.z);
    quat.setW(trans.transform.rotation.w);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto yaw_message = std_msgs::msg::Float64();
    yaw_message.data=yaw;
    pubFastLioYaw_->publish(yaw_message);

    geometry_msgs::msg::TransformStamped trans1;
    trans1.header.frame_id = "odom";
    trans1.child_frame_id = "base_footprint";
    trans1.header.stamp = odomAftMapped.header.stamp; 
    trans1.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans1.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans1.transform.translation.z = odomAftMapped.pose.pose.position.z;
    tf_br->sendTransform(trans1);
}
~~~