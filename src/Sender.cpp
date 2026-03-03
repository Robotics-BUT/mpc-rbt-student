#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

/*
void Sender::Node::onDataTimerTick()
{
  UNIMPLEMENTED(__PRETTY_FUNCTION__);

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };
  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
}
*/

void Sender::Node::onDataTimerTick()
{
  // UNIMPLEMENTED jsme smazali

  // 1. Vygenerování timestampu (to už jsi tu měl)
  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
    
  // 2. Změna hodnot x, y, z (inkrementace, aby se data měnila)
  data.x += 1.1;
  data.y += 2.2;
  data.z += 3.3;

  // 3. Příprava rámce (to už jsi tu měl)
  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };
  
  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);
  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);

  // 4. Serializace a samotné odeslání
  if (Utils::Message::serialize(frame, data)) {
    this->send(frame);
    RCLCPP_INFO(logger, "Odeslano: x=%f, y=%f, z=%f", data.x, data.y, data.z);
  } else {
    RCLCPP_ERROR(logger, "Chyba pri serializaci dat!");
  }
}
