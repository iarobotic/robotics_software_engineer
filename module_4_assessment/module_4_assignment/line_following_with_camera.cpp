#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber()
      : Node("camera_subscriber_node"), _angularVel(0.3) {
    declareParameters();
    setupPublisher();
    setupSubscription();
    RCLCPP_INFO(this->get_logger(), "\n------ Node Started -----\n");
  }

private:
  // Variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
  double _angularVel;

  // Initialization Functions
  void declareParameters() {
    this->declare_parameter<int>("lower_threshold", 200);
    this->declare_parameter<int>("upper_threshold", 250);
  }

  void setupPublisher() {
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  void setupSubscription() {
    _subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&CameraSubscriber::cameraCallback, this, std::placeholders::_1));
  }

  // Callback Function
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr cameraMsg) {
    try {
      cv::Mat cannyImage = preprocessImage(cameraMsg);
      auto velocityMsg = processEdges(cannyImage);
      _publisher->publish(velocityMsg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
  }

  // Image Processing Functions
  cv::Mat preprocessImage(const sensor_msgs::msg::Image::SharedPtr &cameraMsg) {
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(cameraMsg, "bgr8");
    cv::Mat grayImage, cannyImage;
    cv::cvtColor(cvPtr->image, grayImage, cv::COLOR_BGR2GRAY);

    int lowerThreshold = this->get_parameter("lower_threshold").as_int();
    int upperThreshold = this->get_parameter("upper_threshold").as_int();
    cv::Canny(grayImage, cannyImage, lowerThreshold, upperThreshold);

    return cannyImage;
  }

  geometry_msgs::msg::Twist processEdges(const cv::Mat &cannyImage) {
    geometry_msgs::msg::Twist velocityMsg;
    cv::Mat roi = extractROI(cannyImage);

    std::vector<int> edge = findEdgesInRow(roi, 160);
    if (!edge.empty()) {
      int midPoint = calculateMidPoint(edge);
      int robotMidPoint = 640 / 2;
      double error = calculateError(robotMidPoint, midPoint);

      velocityMsg = generateVelocity(error);
      visualizeImage(roi, midPoint, robotMidPoint);
    }

    return velocityMsg;
  }

  // ROI Extraction
  cv::Mat extractROI(const cv::Mat &image) {
    int row = 150;
    return image(cv::Range(row, row + 240), cv::Range(0, 640));
  }

  // Edge Detection Functions
  std::vector<int> findEdgesInRow(const cv::Mat &roi, int row) {
    std::vector<int> edge;
    for (int i = 0; i < roi.cols; ++i) {
      if (roi.at<uchar>(row, i) == 255) {
        edge.push_back(i);
      }
    }
    return edge;
  }

  int calculateMidPoint(const std::vector<int> &edge) {
    int midArea = edge.back() - edge.front();
    return edge.front() + midArea / 2;
  }

  double calculateError(int robotMidPoint, int midPoint) {
    return robotMidPoint - midPoint;
  }

  // Velocity Generation
  geometry_msgs::msg::Twist generateVelocity(double error) {
    geometry_msgs::msg::Twist velocityMsg;
    velocityMsg.linear.x = 0.1;
    velocityMsg.angular.z = (error < 0) ? -_angularVel : _angularVel;
    return velocityMsg;
  }

  // Visualization
  void visualizeImage(cv::Mat &roi, int midPoint, int robotMidPoint) {
    cv::circle(roi, cv::Point(midPoint, 160), 2, cv::Scalar(255, 255, 255), -1);
    cv::circle(roi, cv::Point(robotMidPoint, 160), 5, cv::Scalar(255, 255, 255), -1);
    cv::imshow("Image", roi);
    cv::waitKey(1);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
