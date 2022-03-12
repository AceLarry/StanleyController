#include "stanley_controller/stanley_node.hpp"

StanleyController::StanleyController() : Node("stanley_controller") {
  // Parameters
  this->declare_parameter<double>("WB", 1.6);
  this->get_parameter("WB", WB);  // [m] wheel base of vehicle
  this->declare_parameter<double>("LD", 3.0);
  this->get_parameter("LD", LD);  // [m] lookahead distance
  this->declare_parameter<double>("K", 0.4);
  this->get_parameter("K", K);  // Look forward gain
  this->declare_parameter<double>("curv_prop", 0.5);
  this->get_parameter("curv_prop", curv_prop);
  this->declare_parameter<double>("accel_max", 5.0);
  this->get_parameter("accel_max", accel_max);
  this->declare_parameter<double>("accel_min", -5.0);
  this->get_parameter("accel_min", accel_min);
  this->declare_parameter<double>("vel_max", 20.0);
  this->get_parameter("vel_max", vel_max);
  this->declare_parameter<double>("accel_lat_max", 16.0);
  this->get_parameter("accel_lat_max", accel_lat_max);
  this->declare_parameter<bool>("pub_vel", false);
  this->get_parameter("pub_vel", pub_vel);
  this->declare_parameter<bool>("pub_accel", true);
  this->get_parameter("pub_accel", pub_accel);

  // Initialize state
  speed = 0.0;
  track = {};
  gotMessage = false;

  // Subscribers
  track_sub_ = this->create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
      "track_topic", 1, std::bind(&StanleyController::control_cb, this, std::placeholders::_1));

  car_sub_ = this->create_subscription<eufs_msgs::msg::CarState>(
      "car_topic", 1, std::bind(&StanleyController::car_state_cb, this, std::placeholders::_1));

  // Publish
  command_publisher_ =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("command_topic", 1);

  // Parameter changer
  parameter_changer = this->add_on_set_parameters_callback(
      bind(&StanleyController::onParameterChange, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult StanleyController::onParameterChange(
    std::vector<rclcpp::Parameter> parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  for (rclcpp::Parameter param : parameters) {
    // Lookahead
    if (param.get_name() == "LD") {
      LD = param.as_double();
    }
    if (param.get_name() == "K") {
      K = param.as_double();
    }

    // Vehicle dynamics
    if (param.get_name() == "accel_max") {
      accel_max = param.as_double();
    }
    if (param.get_name() == "accel_min") {
      accel_min = param.as_double();
    }
    if (param.get_name() == "vel_max") {
      vel_max = param.as_double();
    }
    if (param.get_name() == "accel_lat_max") {
      accel_lat_max = param.as_double();
    }

    RCLCPP_INFO(this->get_logger(), "%s changed to %f", param.get_name().c_str(),
                param.as_double());
  }
  return result;
}

void StanleyController::car_state_cb(const eufs_msgs::msg::CarState::SharedPtr msg) {
  speed = msg->twist.twist.linear.x;
}

void StanleyController::control_cb(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg) {
  // Initialise variables
  double speed_cmd = 0.0;
  double accel_cmd = 0.0;
  double steering_cmd = 0.0;
  // K = look forward gain
  double Lf = K * speed + LD;  // update lookahead distance based on curr speed
  int steering_target_idx = 0;
  int closest_idx = 0;
  double speed_preview_dist = 0;
  int speed_preview_idx = 0;  // Curvature point used for speed calc
  int speed_target_idx = 0;

  track.clear();
  if (msg->header.frame_id == "base_footprint") {
    for (int i = 0; i < int(msg->waypoints.size()); i++) {
      std::vector<double> temp = {(double)msg->waypoints[i].position.x - WB / 2,
                                  (double)msg->waypoints[i].position.y};
      track.push_back(temp);
    }
  }

  // Get index of lookahead point
  std::vector<double> targets = {static_cast<double>(steering_target_idx),
                                 static_cast<double>(closest_idx)};
  targets = search_target_index(track, Lf);
  steering_target_idx = static_cast<int>(targets[0]);
  closest_idx = targets[1];

  // Compute steering angle
  steering_cmd = steer_control();

  // The distance for the car to come to a complete stop
  speed_preview_dist = pow(speed, 2.0) / (2 * abs(accel_min));
  speed_target_idx = int(search_target_index(track, speed_preview_dist)[0]);

  // Compute speed and acceleration
  std::vector<double> controls = {accel_cmd, speed_cmd, (double)speed_preview_idx};
  controls = accel_control(closest_idx, speed_target_idx, speed_preview_idx);
  accel_cmd = controls[0];
  speed_cmd = controls[1];
  speed_preview_idx = int(controls[2]);

  // Publish controls
  ackermann_msgs::msg::AckermannDriveStamped drive_msg =
      createDriveMsg(steering_cmd, accel_cmd, speed_cmd);
  command_publisher_->publish(drive_msg);
}

double StanleyController::steer_control() {
  // find coordinates of the two nearest way-points
  double x1 = StanleyController::nearest_two_points(track)[0][0];
  double y1 = StanleyController::nearest_two_points(track)[0][1];
  double x2 = StanleyController::nearest_two_points(track)[1][0];
  double y2 = StanleyController::nearest_two_points(track)[1][1];

  // heading error
  double psi = atan2((y2 - y1), (x2 - x1));

  // cross-track error
  double e = abs(x1 * y2 - x2 * y1) / sqrt(pow((y1 - y2), 2.0) + pow((x2 - x1), 2.0));

  // gain parameter that needs to be tuned
  double k = 1.0;

  // softening constant ensuring denominator be zero
  double ks = 0.000000001;

  // computes steering angle
  double delta = psi + atan2((k * e), (ks + speed));

  return delta;
}

std::vector<std::vector<double>> StanleyController::nearest_two_points(
    std::vector<std::vector<double>> track) {
  // Find nearest 2 points on trajectory closest to front axle
  // Indices of the two lowest values
  if (track.size() <= 2) {
    return {track[0], track[1]};
  }

  int min_idx = 0;
  int next_min_idx = 1;

  if (hypotf(track[min_idx][0], track[min_idx][1]) >
      hypotf(track[next_min_idx][0], track[next_min_idx][1])) {
    next_min_idx = 0;
    min_idx = 1;
  }

  for (int i = 2; i < static_cast<int>(track.size()); i++) {
    if (hypotf(track[i][0], track[i][1]) < hypotf(track[next_min_idx][0], track[next_min_idx][1])) {
      next_min_idx = i;
        if (hypotf(track[min_idx][0], track[min_idx][1])
            > hypotf(track[next_min_idx][0], track[next_min_idx][1])){
        next_min_idx = min_idx;
        min_idx = i;
        }
    }
  }

  if (hypotf(track[min_idx][0], track[min_idx][1]) > 10) {
    RCLCPP_ERROR(this->get_logger(), "Closest point to car is greater than 10m");
  }

  return {track[min_idx], track[next_min_idx],
          {static_cast<double>(min_idx), static_cast<double>(next_min_idx)}};
}

// The same method used in PurePursuitController.
// Purely used to implement longitudinal control (ie speed and acceleration, instead of steering).
std::vector<double> StanleyController::search_target_index(std::vector<std::vector<double>> track,
                                                           double dist) {
  // Find nearest point on trajectory closest to rear
  // Index of lowest value
  int ind = 0;
  double value = -1.0;
  for (int i = 0; i < static_cast<int>(track.size()); i++) {
    double curr = hypotf(track[i][0], track[i][1]);
    if (value == -1) {
      value = curr;
    } else if (curr < value) {
      value = curr;
      ind = i;
    }
  }
  int closest = ind;

  if (value > 10) {
    RCLCPP_ERROR(this->get_logger(), "Closest point to car is greater than 10m");
  }

  // search lookahead target point index
  while (dist > hypotf(track[ind][0], track[ind][1])) {
    if ((ind + 1) >= static_cast<int>(track.size())) {
      break;  // not exceed goal
    }
    ind = ind + 1;
  }

  return {static_cast<double>(ind), static_cast<double>(closest)};
}

std::vector<double> StanleyController::accel_control(int closest_idx, int speed_target_idx,
                                                     int speed_preview_idx) {
  // Get max curvature point up to speed target index
  // Specify starting index (rather than using closest_idx)
  int start_idx = floor(closest_idx + (speed_target_idx - closest_idx) * curv_prop);

  double speed_preview_curvature = -1.0;
  for (int i = start_idx; i <= speed_target_idx; i++) {
    if (speed_preview_curvature == -1) {
      speed_preview_curvature = abs(track[i][2]);
      speed_preview_idx = i;
      continue;
    }
    if (abs(track[i][2]) > speed_preview_curvature) {
      speed_preview_curvature = abs(track[i][2]);
      speed_preview_idx = i;
    }
  }

  // If straight track, set speed to max
  double max_speed = 0.0;
  if (speed_preview_curvature == 0) {
    max_speed = vel_max;
  } else {
    max_speed = sqrt(accel_lat_max / speed_preview_curvature);
  }

  double speed_cmd = max_speed;
  // Cap speed
  speed_cmd = std::max(std::min(speed_cmd, vel_max), 0.0);

  // Calculate the acceleration
  double accel_cmd = (speed_cmd - speed);
  accel_cmd = std::max(std::min(accel_cmd, accel_max), accel_min);

  return {accel_cmd, speed_cmd, (double)speed_preview_idx};
}

ackermann_msgs::msg::AckermannDriveStamped StanleyController::createDriveMsg(double steering_angle,
                                                                             double acceleration,
                                                                             double speed) {
  ackermann_msgs::msg::AckermannDriveStamped drive_stamped;
  drive_stamped.header.frame_id = "stanley_controller";
  drive_stamped.header.stamp = this->now();
  if (pub_accel) {
    drive_stamped.drive.acceleration = acceleration;
  }
  if (pub_vel) {
    drive_stamped.drive.speed = speed;
  }
  drive_stamped.drive.steering_angle = steering_angle;
  return drive_stamped;
}

std::vector<double> StanleyController::diff(std::vector<double> input) {
  std::vector<double> output;
  if (input.size() <= 1) return input;
  for (int i = 1; i < int(input.size()); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StanleyController>());
  rclcpp::shutdown();
  return 0;
}
