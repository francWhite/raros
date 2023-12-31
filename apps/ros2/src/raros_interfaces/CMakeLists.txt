 cmake_minimum_required(VERSION 3.8)
project(raros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Distance.msg"
  "msg/StepperInstruction.msg"
  "msg/StepperMovement.msg"
  "msg/StepperStatus.msg"
  "msg/StepperFeedback.msg"
  "msg/StepperParameters.msg"
  "msg/Direction.msg"
  "msg/Status.msg"
  "srv/RotateRangeSensor.srv"
  "srv/RotateCamera.srv"
  "srv/CaptureImage.srv"
  "srv/GetStatus.srv"
  "srv/ActionPlayTone.srv"
  "srv/ActionCompleted.srv"
  "srv/ActionMove.srv"
  "srv/ActionRotate.srv"
  "srv/ActionTurn.srv"
  "action/PlayTone.action"
  "action/Move.action"
  "action/Rotate.action"
  "action/Turn.action"
  DEPENDENCIES geometry_msgs
)

ament_package()