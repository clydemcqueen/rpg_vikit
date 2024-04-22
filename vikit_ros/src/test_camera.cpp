#include <vikit/camera_loader.h>

class TestCamera : public rclcpp::Node
{
private:
  vk::AbstractCamera* cam_{};

public:
  TestCamera() : Node("test_camera") {}

  // Do not call shared_from_this() in the constructor
  void post_construction()
  {
    if(!vk::camera_loader::loadFromRosNode(shared_from_this(), cam_))
      throw std::runtime_error("Camera model not correctly specified.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto test_camera = std::make_shared<TestCamera>();
  test_camera->post_construction();
  rclcpp::spin(test_camera);
  rclcpp::shutdown();
  return 0;
}
