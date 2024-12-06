#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "opencv2/opencv.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using namespace cv;

class ImageProcessorNode : public rclcpp::Node
{
public:
    ImageProcessorNode()
        : Node("image_processor_node"), count_(0)
    {
        // Subscriber to /videocamera topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/videocamera",
            10,
            std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));

        // Publisher to /processed_image topic
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

        RCLCPP_INFO(this->get_logger(), "Image Processor Node has been started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
       Mat im;
        try{
            im = cv_bridge::toCvCopy(*msg.get(), "bgr8")->image;
        }
        catch (const cv_bridge::Exception& e)  {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        }
        
          SimpleBlobDetector::Params params;

          // Change thresholds
          params.minThreshold = 40;
          params.maxThreshold = 200;

          // Filter by Area.
          params.filterByArea = false;
          params.minArea = 1500; //min (1500) number of pixels for blob

          // Filter by Circularity
          params.filterByCircularity = true;
          params.minCircularity = 0.85; //1 is for perfect circular objects

          // Filter by Convexity
          params.filterByConvexity = true;
          params.minConvexity = 0.8; //1 is for convex shapes

          // Filter by Inertia
          params.filterByInertia = true;
          params.minInertiaRatio = 0.8; //1 is for perfect circle

          //storage for blobs
          std::vector<KeyPoint> keypoints;

      #if CV_MAJOR_VERSION < 3 

          //create the Blob detector
          SimpleBlobDetector detector(params);
          
          //detects blobs on the mask
          detector.detect(im, keypoints); 
      #else 
           //create the Blob detector
          Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
          
          //detects blobs on the mask
          detector->detect(im, keypoints); 

      #endif

        std::cout << "# of keypoint found: " <<  keypoints.size()  << std::endl;
        for (size_t i = 0; i < keypoints.size(); ++i) {
        // Print keypoint properties
        std::cout << "Keypoint " << i + 1 << ":" << std::endl;
        std::cout << "Position: (" << keypoints[i].pt.x << ", " << keypoints[i].pt.y << ")" << std::endl;
    }

          Mat im_with_keypoints;

          drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          //drawKeypoints(gray, keypoints, im_with_keypoints, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          
          processed_image_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();
          //processed_image_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", gray).toImageMsg();

          publisher_->publish(*processed_image_.get());
            
          //Debug on console:
          if(1){
           RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
      	    count_++;
          }
        }
    
    
    sensor_msgs::msg::Image::SharedPtr processed_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
     size_t count_;   //Debug counter
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

 
