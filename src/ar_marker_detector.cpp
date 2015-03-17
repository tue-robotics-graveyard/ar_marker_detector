#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rgbd_transport_test_client");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("test");

    ros::Rate r(30);
    while (ros::ok())
    {
        rgbd::Image image;
        if (client.nextImage(image))
        {
            std::cout << "Image: t = " << std::fixed << image.getTimestamp() << ", frame = " << image.getFrameId() << std::endl;

            cv::imshow("rgb", image.getRGBImage());
            cv::imshow("depth", image.getDepthImage() / 8);
            cv::waitKey(3);
        }
        r.sleep();
    }

    ros::spin();

    return 0;
}
