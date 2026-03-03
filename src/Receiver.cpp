#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

/*
void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  UNIMPLEMENTED(__PRETTY_FUNCTION__);

  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
}
*/

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  // UNIMPLEMENTED jsme smazali

  // 1. Rozbalení (deserializace) dat z rámce
  if (Utils::Message::deserialize(frame, data)) {
    // 2. Vypsání přijatých dat (pokud se to povede)
    RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
    RCLCPP_INFO(logger, "Prijato: x=%f, y=%f, z=%f", data.x, data.y, data.z);
  } else {
    RCLCPP_WARN(logger, "Nepodarilo se deserializovat data!");
  }
}
