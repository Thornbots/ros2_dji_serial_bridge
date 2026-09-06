# Fire decision published by thornbots_pkg's firing logic.
std_msgs/Header header  # header.stamp = decision time

bool fire             # true = fire
uint32 delay_ms       # milliseconds after header.stamp to actually fire (0 = immediate)
