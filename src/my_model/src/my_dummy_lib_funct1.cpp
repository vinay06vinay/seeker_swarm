
// openCV stuff
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

// rclcpp stuff
#include <rclcpp/logging.hpp>


int function1(int input)
{
  cv::Mat img;
  img = cv::imread("test.png", cv::IMREAD_GRAYSCALE); 
  fprintf (stderr, "Calling OpenCV function\n");
  RCLCPP_INFO_STREAM (rclcpp::get_logger("my_model"), "Calling ROS function");
  
  return input + 100;
}
