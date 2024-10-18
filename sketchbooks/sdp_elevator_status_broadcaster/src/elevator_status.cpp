#include "elevator_status.h"

void print_elevator_config_vector(LGFX_Sprite &sprite, const std::vector<ElevatorConfig> &elevator_config, int32_t push_x, int32_t push_y) {
  sprite.fillScreen(0xFFFFFF);
  sprite.setCursor(0, 0);
  sprite.printf("Elevator Config\n");
  for (auto &entry : elevator_config) {
    sprite.printf("Floor: %d, Height: %.2f\n", entry.floor_num, entry.floor_height);
  }
  sprite.pushSprite(push_x, push_y);
}

/**
 * Calculate floor number from altitude
 *
 * @param altitude Altitude in meters
 * @param elevator_config Elevator configuration
 * @param threshold Threshold for floor detection
 * @return Floor number
 *
 * C++ version of code below
 *
 * altitude_diff_list = [
                {
                    'floor': key,
                    'altitude_diff':
                    (entry['altitude'] - self.elevator_config[self.param_anchor_floor]['altitude']) - (self.state_altitude - self.param_anchor_altitude)
                }
                for key, entry in self.elevator_config.items()]
            nearest_entry = min(
                altitude_diff_list,
                key=lambda x: math.fabs(x['altitude_diff'])
            )
            self.state_current_floor = nearest_entry['floor']
 */
std::optional<int32_t> calc_floor(float altitude, float reference_altitude, int32_t reference_floor, const std::vector<ElevatorConfig> &elevator_config) {
  if (elevator_config.empty()) {
    return std::nullopt;
  }
  int32_t candidate_floor = elevator_config[0].floor_num;
  int reference_floor_index = -100;
  for (int i = 0; i < elevator_config.size(); i++) {
    if (elevator_config[i].floor_num == reference_floor) {
      reference_floor_index = i;
      break;
    }
  }
  if (reference_floor_index == -100) {
    return std::nullopt;
  }
  float candidate_diff = fabs((elevator_config[0].floor_height - elevator_config[reference_floor_index].floor_height) - (altitude - reference_altitude));
  for (int i = 1; i < elevator_config.size(); i++) {
    float diff = fabs((elevator_config[i].floor_height - elevator_config[reference_floor_index].floor_height) - (altitude - reference_altitude));
    if (diff < candidate_diff) {
      candidate_diff = diff;
      candidate_floor = elevator_config[i].floor_num;
    }
  }
  return candidate_floor;
}

/**
 * Calculate acceleration on gravity direction
 *
 * @param accels_x Acceleration in x-axis
 * @param accels_y Acceleration in y-axis
 * @param accels_z Acceleration in z-axis
 * @param gravity_x Gravity in x-axis
 * @param gravity_y Gravity in y-axis
 * @param gravity_z Gravity in z-axis
 * @return Acceleration on gravity direction
 */
float calc_accel_on_gravity(float accels_x, float accels_y, float accels_z, float gravity_x, float gravity_y, float gravity_z) {
  return (accels_x * gravity_x + accels_y * gravity_y + accels_z * gravity_z) / sqrt(gravity_x * gravity_x + gravity_y * gravity_y + gravity_z * gravity_z);
}

/**
 * WIthdraw gravity from acceleration
 *
 * @param accels_x Acceleration in x-axis
 * @param accels_y Acceleration in y-axis
 * @param accels_z Acceleration in z-axis
 * @param gravity_x Gravity in x-axis
 * @param gravity_y Gravity in y-axis
 * @param gravity_z Gravity in z-axis
 * @return Acceleration without gravity
 */
std::tuple<float, float, float> withdraw_gravity(float accels_x, float accels_y, float accels_z, float gravity_x, float gravity_y, float gravity_z) {
  float accels_x_without_gravity = accels_x - gravity_x;
  float accels_y_without_gravity = accels_y - gravity_y;
  float accels_z_without_gravity = accels_z - gravity_z;
  return std::make_tuple(accels_x_without_gravity, accels_y_without_gravity, accels_z_without_gravity);
}

/**
 * Calculate next step moving status from acceleration
 *
 * @param accels_x Acceleration in x-axis
 * @param accels_y Acceleration in y-axis
 * @param accels_z Acceleration in z-axis
 * @param gravity_x Gravity in x-axis
 * @param gravity_y Gravity in y-axis
 * @param gravity_z Gravity in z-axis
 * @param current_status Current moving status
 * @param moving_threshold Threshold for moving detection
 * @return Moving status
 */
ElevatorMovingStatus calc_moving_status(float accel_on_gravity, ElevatorMovingStatus current_status, float moving_threshold) {
  switch (current_status) {
    case HALT:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      }
      break;
    case UP_ACCEL:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      } else {
        return UP_STABLE;
      }
      break;
    case UP_STABLE:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      } else {
        return UP_STABLE;
      }
      break;
    case UP_DECEL:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      } else {
        return UP_STABLE;
      }
      break;
    case DOWN_ACCEL:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      } else {
        return DOWN_STABLE;
      }
      break;
    case DOWN_STABLE:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      } else {
        return DOWN_STABLE;
      }
      break;
    case DOWN_DECEL:
      if (accel_on_gravity > moving_threshold) {
        return UP_ACCEL;
      } else if (accel_on_gravity < -moving_threshold) {
        return DOWN_ACCEL;
      } else {
        return DOWN_STABLE;
      }
      break;
  }
  return HALT;
}

std::string moving_status_to_string(ElevatorMovingStatus status) {
  switch (status) {
    case HALT:
      return "HALT";
    case UP_ACCEL:
      return "UP_ACCEL";
    case UP_STABLE:
      return "UP_STABLE";
    case UP_DECEL:
      return "UP_DECEL";
    case DOWN_ACCEL:
      return "DOWN_ACCEL";
    case DOWN_STABLE:
      return "DOWN_STABLE";
    case DOWN_DECEL:
      return "DOWN_DECEL";
    default:
      return "UNKNOWN";
  }
}