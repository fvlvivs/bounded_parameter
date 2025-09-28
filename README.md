# bounded_parameter

A simple ROS 2 package that provides template functions for declaring and validating parameters with bounds checking.

## Usage

### Include the Header

```cpp
#include "bounded_parameter/bounded_parameter.hpp"
```

### Example

```cpp
// Declare a bounded integer parameter.
declareBoundedParameter<int>(
    this,               // Node pointer
    "frequency",        // Parameter name
    10,                 // Default value
    10,                 // Lower bound
    50,                 // Upper bound
    42                  // Initial value
);

// Get and validate the parameter
int value;
if (getBoundedParameter(this, "frequency", value)) {
    RCLCPP_INFO(get_logger(), "Valid value: %d.", value);
} else {
    RCLCPP_WARN(get_logger(), "Value out of bounds, using default.");
}
```

### Parameter YAML Configuration

```yaml
example_node:
  ros__parameters:
    frequency:
      default: 10
      lower_bound: 10
      upper_bound: 50
      value: 42

    frame_id:
      default: "map"
      allowed_values: ["map", "odom", "base_link"]
      value: "base_link"
```


## License

This package is licensed under the Apache License 2.0. See LICENSE for details.
