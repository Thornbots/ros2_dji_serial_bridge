# Fire decision published by point_to_cv_target.py on /sentry/fire_command
# and relayed by mcb_relay onto /dji_serial_bridge/fire_command.
# ROS-internal today: there is no UART message ID for it and this package
# subscribes to no such topic, so the decision currently stops at that topic.
# PLANNED TO GO AWAY: merge into CVTarget as a fire-delay field, so aim and
# fire cross the wire as one decision that cannot pair up wrongly.
# See UART_PROTOCOL.md ("Not on the wire") for the rationale.
std_msgs/Header header  # header.stamp = decision time

bool fire             # true = fire
uint32 delay_ms       # milliseconds after header.stamp to actually fire (0 = immediate)
