# Fire decision published by point_to_cv_target.py on /sentry/fire_command
# and relayed by mcb_relay onto /dji_serial_bridge/fire_command.
# ROS-internal today: there is no UART message ID for it and this package
# subscribes to no such topic, so the decision currently stops at that topic.
# See UART_PROTOCOL.md ("Not on the wire").
std_msgs/Header header  # header.stamp = decision time

bool fire             # true = fire
uint32 delay_ms       # milliseconds after header.stamp to actually fire (0 = immediate)
