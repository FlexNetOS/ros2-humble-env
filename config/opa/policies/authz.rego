package authz

# Default deny
default allow = false

# Allow if user has admin role
allow {
    input.user.roles[_] == "admin"
}

# Allow read access to authenticated users
allow {
    input.method == "GET"
    input.user.authenticated == true
}

# Allow ROS2 topic access based on namespace
allow {
    input.resource.type == "ros2_topic"
    input.user.namespace == input.resource.namespace
}
