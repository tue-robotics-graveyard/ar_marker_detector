#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <tf/transform_datatypes.h>

#include <ar_marker_detector/Toggle.h>
#include <ar_marker_detector/object.h>

#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/arMulti.h>

//! Global stuff
const double AR_TO_ROS = 0.001;

// Parameters
bool publishTf_;
int threshold_;

//
tf::TransformBroadcaster broadcaster_;
bool active_;
rgbd::Client client_;

int objectnum_;
ar_object::ObjectData_T * object;

bool startStopCallback(ar_marker_detector::Toggle::Request &req, ar_marker_detector::Toggle::Response &res) {

    active_ = req.enable;
    res.success = true;
    return res.success;

}

void arInit (const std::string& pattern_filename)
{
    // ToDo:
    /*
    arInitCparam (&cam_param_);
    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load in the object data - trained markers and associated bitmap files
    if ((object = ar_object::read_ObjData (pattern_filename_, &objectnum_)) == NULL) {
        ROS_BREAK ();
    }
    ROS_DEBUG ("Objectfile num = %d", objectnum_);

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
    */
}

bool findMarkers(){

    rgbd::Image image;
    if (!client_.nextImage(image)) {
        return false;
    }

    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k, j;

    /* Get the image from the rgbd server
     * NOTE: the dataPtr format is BGR because the ARToolKit library was
     * build with V4L, dataPtr format change according to the
     * ARToolKit configure option (see config.h).*/
    dataPtr = (ARUint8 *) image.getRGBImage().data;

    // detect the markers in the video frame
    if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
        argCleanup ();
        ROS_BREAK ();
    }

    // check for known patterns
    for (i = 0; i < objectnum_; i++)
    {
        k = -1;
        for (j = 0; j < marker_num; j++)
        {
            if (object[i].id == marker_info[j].id)
            {
                if (k == -1)
                    k = j;
                else                  // make sure you have the best pattern (highest confidence factor)
                    if (marker_info[k].cf < marker_info[j].cf)
                        k = j;
            }
        }
        if (k == -1)
        {
            object[i].visible = 0;
            continue;
        }

        // calculate the transform for each marker
        if (object[i].visible == 0)
        {
            arGetTransMat (&marker_info[k], object[i].marker_center, object[i].marker_width, object[i].trans);
        }
        else
        {
            arGetTransMatCont (&marker_info[k], object[i].trans,
                               object[i].marker_center, object[i].marker_width, object[i].trans);
        }
        object[i].visible = 1;

        double arQuat[4], arPos[3];

        //arUtilMatInv (object[i].trans, cam_trans);
        arUtilMat2QuatPos (object[i].trans, arQuat, arPos);

        // **** convert to ROS frame

        //! Transform depth image
        // Convert depth image
        // ToDo:
        const cv::Mat& depth_img_ptr = image.getDepthImage();
        //cv::Mat depth_img = depth_img_ptr.data;

        //! Get 3D position using depth image
        bool found_pos_based_on_depth = false;

        //Calculate median from 5x5 pixel center
        cv::Point2d pos_2d(marker_info[k].pos[0], marker_info[k].pos[1]);
        std::vector<float> pixelDistances;
        const int half_rows = pos_2d.y;
        const int half_cols = pos_2d.x;

        for(int row = half_rows - 4; row < half_rows + 4; row++)
        {
            for(int col = half_cols - 4; col < half_cols + 4; col++)
            {
                // ToDo:
                /*
                if (col > 0 && col < depth_img.cols-1 && row > 0 && row < depth_img.rows)
                {
                    float depth = depth_img.at<float>(row, col);
                    // Ignore NaNs
                    if (depth == depth) {
                        pixelDistances.push_back(depth);
                    }
                }*/
            }
        }

        // Determine 3D position using camera model and distance
        cv::Point3d pos_3d;
        // ToDo:
        /*
        if (!pixelDistances.empty()) {
            std::sort(pixelDistances.begin(), pixelDistances.end());
            pos_3d = cam_model_.projectPixelTo3dRay(pos_2d) * pixelDistances[pixelDistances.size() / 2];
            found_pos_based_on_depth = true;

        }*/

        double quat[4], pos[3];

        // Orientation based on 2D information only
        quat[0] = -arQuat[0];
        quat[1] = -arQuat[1];
        quat[2] = -arQuat[2];
        quat[3] = arQuat[3];

        if (found_pos_based_on_depth)
        {
            // Position based on depth image
            pos[0] = pos_3d.x;
            pos[1] = pos_3d.y;
            pos[2] = pos_3d.z;

            ROS_INFO(" Marker %i has position (%f,%f,%f) and orientation (%f,%f,%f,%f)",
                     i, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]);
        }
        else
        {
            // Original position estimate based on 2D information only
            pos[0] = arPos[0] * AR_TO_ROS;
            pos[1] = arPos[1] * AR_TO_ROS;
            pos[2] = arPos[2] * AR_TO_ROS;

            ROS_WARN("Found marker but no depth information available at estimated position with distance %f!", pos[2]);
        }


        // **** publish the marker
/*
        pein_ar_pose::ARMarker ar_pose_marker;
        ar_pose_marker.header.frame_id = rgb_img_msg->header.frame_id;
        ar_pose_marker.header.stamp = rgb_img_msg->header.stamp;
        ar_pose_marker.id = object[i].id;

        ar_pose_marker.pose.pose.position.x = pos[0];
        ar_pose_marker.pose.pose.position.y = pos[1];
        ar_pose_marker.pose.pose.position.z = pos[2];

        ar_pose_marker.pose.pose.orientation.x = quat[0];
        ar_pose_marker.pose.pose.orientation.y = quat[1];
        ar_pose_marker.pose.pose.orientation.z = quat[2];
        ar_pose_marker.pose.pose.orientation.w = quat[3];

        ar_pose_marker.confidence = round(marker_info->cf * 100);
        arPoseMarkers_.markers.push_back (ar_pose_marker);
*/
        // **** publish transform between camera and marker

        tf::Quaternion rotation (quat[0], quat[1], quat[2], quat[3]);
        tf::Vector3 origin (pos[0], pos[1], pos[2]);
        tf::Transform t (rotation, origin);

        // Only publish results if position is based on depth information
        if (publishTf_ && found_pos_based_on_depth)
        {
            tf::StampedTransform camToMarker (t, ros::Time(image.getTimestamp()), image.getFrameId(), object[i].name);
            broadcaster_.sendTransform(camToMarker);
        }

        // **** publish visual marker
/*
        if (publishVisualMarkers_ && found_pos_based_on_depth)
        {
            tf::Vector3 markerOrigin (0, 0, 0.25 * object[i].marker_width * AR_TO_ROS);
            tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
            tf::Transform markerPose = t * m; // marker pose in the camera frame

            tf::poseTFToMsg (markerPose, rvizMarker_.pose);

            rvizMarker_.header.frame_id = rgb_img_msg->header.frame_id;
            rvizMarker_.header.stamp = rgb_img_msg->header.stamp;
            rvizMarker_.id = object[i].id;

            rvizMarker_.scale.x = 1.0 * object[i].marker_width * AR_TO_ROS;
            rvizMarker_.scale.y = 1.0 * object[i].marker_width * AR_TO_ROS;
            rvizMarker_.scale.z = 0.5 * object[i].marker_width * AR_TO_ROS;
            rvizMarker_.ns = "basic_shapes";
            rvizMarker_.type = visualization_msgs::Marker::CUBE;
            rvizMarker_.action = visualization_msgs::Marker::ADD;
            switch (i)
            {
            case 0:
                rvizMarker_.color.r = 0.0f;
                rvizMarker_.color.g = 0.0f;
                rvizMarker_.color.b = 1.0f;
                rvizMarker_.color.a = 1.0;
                break;
            case 1:
                rvizMarker_.color.r = 1.0f;
                rvizMarker_.color.g = 0.0f;
                rvizMarker_.color.b = 0.0f;
                rvizMarker_.color.a = 1.0;
                break;
            default:
                rvizMarker_.color.r = 0.0f;
                rvizMarker_.color.g = 1.0f;
                rvizMarker_.color.b = 0.0f;
                rvizMarker_.color.a = 1.0;
            }
            rvizMarker_.lifetime = ros::Duration (1.0);

            rvizMarkerPub_.publish (rvizMarker_);
            ROS_DEBUG ("Published visual marker");
        }*/
    }

    /*
    arMarkerPub_.publish (arPoseMarkers_);
    ROS_DEBUG ("Published ar_multi markers");
    */

    /*if (client.nextImage(image))
    {
        std::cout << "Image: t = " << std::fixed << image.getTimestamp() << ", frame = " << image.getFrameId() << std::endl;

        cv::imshow("rgb", image.getRGBImage());
        cv::imshow("depth", image.getDepthImage() / 8);
        cv::waitKey(3);
    }*/
}

int main(int argc, char **argv) {

    // Globals
    active_ = false;

    ros::init(argc, argv, "rgbd_transport_test_client");
    ros::NodeHandle nh;

    // Get paths
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    std::string default_path = "data/object_4x4";

    // Nodehandle
    ros::NodeHandle n_param ("~");

    //! Get ROS parameters
    bool publishVisualMarkers_;
    std::string rgbdTopic_;

    // Whether or not publishing tfs
    if (!n_param.getParam ("publish_tf", publishTf_)) {
        publishTf_ = true;
    }
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    // Whether or not publishing visual markers in RViz
    if (!n_param.getParam ("publish_visual_markers", publishVisualMarkers_)) {
        publishVisualMarkers_ = true;
    }
    ROS_INFO ("\tPublish visual markers: %d", publishVisualMarkers_);

    // Threshold for converting to binary image
    if (!n_param.getParam ("threshold", threshold_)) {
        threshold_ = 100;
    }
    ROS_INFO ("\tThreshold: %d", threshold_);

    // RGB image topic
    if (!n_param.getParam ("rgbd_topic", rgbdTopic_)) {
        rgbdTopic_ = "top_kinect/rgbd";
    }
    ROS_INFO ("\tRgbd topic: %s", rgbdTopic_.c_str());

    /*
    // Modifications to allow path list from outside the package
    n_param.param ("marker_pattern_list", local_path, default_path);
    if (local_path.compare(0,5,"data/") == 0){
        //according to previous implementations, check if first 5 chars equal "data/"
        sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
    }
    else{
        //for new implementations, can pass a path outside the package_path
        sprintf (pattern_filename_, "%s", local_path.c_str ());
    }
    ROS_INFO ("Marker Pattern Filename: %s", pattern_filename_);
    */

    /*
    // Advertise results
    arMarkerPub_ = n_.advertise < ar_marker_detector::ARMarkers > ("ar_pose_marker", 0);
    if(publishVisualMarkers_)
    {
        rvizMarkerPub_ = nh.advertise < visualization_msgs::Marker > ("/ar_marker", 0);
    }*/

    // Service for switching on/off
    ros::ServiceServer srv_on_off_ = nh.advertiseService("ar_marker_detector", &startStopCallback);

    // Initialize client
    client_.intialize(rgbdTopic_);

    ros::Rate r(10);
    while (ros::ok())
    {
        if (active_)
        {
            findMarkers();
        }

        r.sleep();
    }

    ros::spin();

    return 0;
}
