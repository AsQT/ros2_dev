#include "robot_hardware_interface/tcp_client.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_hardware_interface
{
namespace
{

std::string errno_string(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

uint16_t unpack_u16_le_raw(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

void ensure_success_ack(
  const std::optional<std::vector<uint8_t>> & payload,
  const std::string & command_name)
{
  if (!payload.has_value() || payload->empty()) {
    throw std::runtime_error(command_name + " ACK payload is empty");
  }
  if ((*payload)[0] != 0x00) {
    std::ostringstream oss;
    oss << command_name << " failed with ACK code=0x"
        << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>((*payload)[0]);
    throw std::runtime_error(oss.str());
  }
}

}  // namespace

RobotTcpClient::~RobotTcpClient()
{
  disconnect();
}

bool RobotTcpClient::connect(const std::string & ip, int port, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  close_locked();
  timeout_ms_ = sanitize_timeout_ms(timeout_ms);
  last_error_.clear();

  if (ip.empty()) {
    last_error_ = "robot_ip is empty";
    return false;
  }
  if (port <= 0 || port > 65535) {
    last_error_ = "robot_port out of range";
    return false;
  }

  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) {
    last_error_ = errno_string("socket() failed");
    return false;
  }

  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    last_error_ = errno_string("fcntl(O_NONBLOCK) failed");
    close_locked();
    return false;
  }

  int opt = 1;
  (void)::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
  (void)::setsockopt(fd_, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    last_error_ = "invalid robot IPv4 address: " + ip;
    close_locked();
    return false;
  }

  const int rc = ::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  if (rc == 0) {
    return true;
  }

  if (errno != EINPROGRESS) {
    last_error_ = errno_string("connect() failed");
    close_locked();
    return false;
  }

  if (!wait_fd_locked(true, timeout_ms_)) {
    last_error_ = "TCP connect timeout";
    close_locked();
    return false;
  }

  int so_error = 0;
  socklen_t so_error_len = sizeof(so_error);
  if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) < 0) {
    last_error_ = errno_string("getsockopt(SO_ERROR) failed");
    close_locked();
    return false;
  }
  if (so_error != 0) {
    last_error_ = "TCP connect failed: " + std::string(std::strerror(so_error));
    close_locked();
    return false;
  }

  return true;
}

void RobotTcpClient::disconnect()
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  close_locked();
}

bool RobotTcpClient::is_connected() const
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return fd_ >= 0;
}

std::string RobotTcpClient::last_error() const
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return last_error_;
}

size_t RobotTcpClient::last_state_payload_length() const
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return last_state_payload_length_;
}

size_t RobotTcpClient::last_state_payload_offset() const
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return last_state_payload_offset_;
}

size_t RobotTcpClient::last_state_axis_bytes() const
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return last_state_axis_bytes_;
}

bool RobotTcpClient::send_frame(uint8_t cmd, const std::vector<uint8_t> & payload, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return send_frame_locked(cmd, payload, timeout_ms);
}

Frame RobotTcpClient::read_frame(std::optional<uint8_t> expected_cmd, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  return read_frame_locked(expected_cmd, timeout_ms);
}

std::optional<std::vector<uint8_t>> RobotTcpClient::exchange(
  uint8_t cmd,
  const std::vector<uint8_t> & payload,
  bool wait_reply,
  std::optional<uint8_t> expected_cmd,
  int timeout_ms)
{
  std::lock_guard<std::mutex> lock(io_mtx_);
  if (fd_ < 0) {
    throw std::runtime_error("TCP not connected");
  }

  drain_input_locked();
  if (!send_frame_locked(cmd, payload, timeout_ms)) {
    throw std::runtime_error("TCP send timeout");
  }

  if (!wait_reply) {
    return std::nullopt;
  }

  const uint8_t exp_cmd = expected_cmd.has_value() ? expected_cmd.value() : cmd;
  return read_frame_locked(exp_cmd, timeout_ms).payload;
}

std::vector<uint8_t> RobotTcpClient::build_frame_locked(
  uint8_t cmd,
  const std::vector<uint8_t> & payload) const
{
  std::vector<uint8_t> frame;
  if (payload.size() > 0xFFFFu) {
    throw std::runtime_error("payload too large for robot TCP frame");
  }

  frame.reserve(ROBOT_TCP_HEADER_SIZE + payload.size());
  append_u16_le(frame, ROBOT_TCP_MAGIC);  // Wire bytes AA 55 for little-endian 0x55AA.
  frame.push_back(cmd);
  append_u16_le(frame, seq_);
  append_u16_le(frame, static_cast<uint16_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  return frame;
}

bool RobotTcpClient::send_frame_locked(uint8_t cmd, const std::vector<uint8_t> & payload, int timeout_ms)
{
  if (fd_ < 0) {
    throw std::runtime_error("TCP not connected");
  }

  const int timeout = timeout_ms < 0 ? timeout_ms_ : sanitize_timeout_ms(timeout_ms);
  const auto frame = build_frame_locked(cmd, payload);
  seq_ = static_cast<uint16_t>(seq_ + 1);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
  size_t offset = 0;

  while (offset < frame.size()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    const auto remaining =
      static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

    if (!wait_fd_locked(true, std::max(0, remaining))) {
      return false;
    }

#ifdef MSG_NOSIGNAL
    constexpr int send_flags = MSG_NOSIGNAL;
#else
    constexpr int send_flags = 0;
#endif
    const ssize_t n =
      ::send(fd_, frame.data() + offset, frame.size() - offset, send_flags);
    if (n > 0) {
      offset += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      continue;
    }

    last_error_ = n == 0 ? "TCP send returned 0" : errno_string("TCP send failed");
    close_locked();
    throw std::runtime_error(last_error_);
  }

  return true;
}

Frame RobotTcpClient::read_frame_locked(std::optional<uint8_t> expected_cmd, int timeout_ms)
{
  if (fd_ < 0) {
    throw std::runtime_error("TCP not connected");
  }

  const int timeout = timeout_ms < 0 ? timeout_ms_ : sanitize_timeout_ms(timeout_ms);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);

  auto remaining_ms = [&]() -> int {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return 0;
    }
    return static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
  };

  uint8_t header[ROBOT_TCP_HEADER_SIZE]{};
  const int header_remaining = remaining_ms();
  if (header_remaining <= 0 || !read_exact_locked(header, sizeof(header), header_remaining)) {
    throw std::runtime_error("receive timeout waiting robot TCP header");
  }

  const uint16_t magic = unpack_u16_le_raw(header);
  if (magic != ROBOT_TCP_MAGIC) {
    std::ostringstream oss;
    oss << "robot TCP magic mismatch: expected=0x" << std::hex << std::setw(4)
        << std::setfill('0') << ROBOT_TCP_MAGIC << " got=0x" << std::setw(4) << magic;
    throw std::runtime_error(oss.str());
  }

  const uint8_t cmd = header[2];
  const uint16_t length = unpack_u16_le_raw(header + 5);

  std::vector<uint8_t> payload(length);
  if (length > 0) {
    const int payload_remaining = remaining_ms();
    if (payload_remaining <= 0 ||
      !read_exact_locked(payload.data(), payload.size(), payload_remaining))
    {
      throw std::runtime_error("receive timeout waiting robot TCP payload");
    }
  }

  if (expected_cmd.has_value() && cmd != expected_cmd.value()) {
    std::ostringstream oss;
    oss << "CMD mismatch: expected=0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(expected_cmd.value()) << " got=0x" << std::setw(2)
        << static_cast<int>(cmd);
    throw std::runtime_error(oss.str());
  }

  return Frame{cmd, payload};
}

bool RobotTcpClient::read_byte_locked(uint8_t & out, int timeout_ms)
{
  if (fd_ < 0) {
    throw std::runtime_error("TCP not connected");
  }

  if (!wait_fd_locked(false, timeout_ms)) {
    return false;
  }

  ssize_t n = 0;
  do {
    n = ::recv(fd_, &out, 1, 0);
  } while (n < 0 && errno == EINTR);

  if (n == 1) {
    return true;
  }
  if (n == 0) {
    last_error_ = "TCP peer disconnected";
    close_locked();
    throw std::runtime_error(last_error_);
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK) {
    return false;
  }

  last_error_ = errno_string("TCP receive failed");
  close_locked();
  throw std::runtime_error(last_error_);
}

bool RobotTcpClient::read_exact_locked(uint8_t * out, size_t len, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  size_t offset = 0;

  while (offset < len) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    const auto remaining =
      static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

    if (!wait_fd_locked(false, std::max(0, remaining))) {
      return false;
    }

    ssize_t n = 0;
    do {
      n = ::recv(fd_, out + offset, len - offset, 0);
    } while (n < 0 && errno == EINTR);

    if (n > 0) {
      offset += static_cast<size_t>(n);
      continue;
    }
    if (n == 0) {
      last_error_ = "TCP peer disconnected";
      close_locked();
      throw std::runtime_error(last_error_);
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      continue;
    }

    last_error_ = errno_string("TCP receive failed");
    close_locked();
    throw std::runtime_error(last_error_);
  }

  return true;
}

bool RobotTcpClient::wait_fd_locked(bool want_write, int timeout_ms) const
{
  if (fd_ < 0) {
    return false;
  }

  fd_set read_set;
  fd_set write_set;
  FD_ZERO(&read_set);
  FD_ZERO(&write_set);
  if (want_write) {
    FD_SET(fd_, &write_set);
  } else {
    FD_SET(fd_, &read_set);
  }

  timeval timeout{};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int rc = 0;
  do {
    rc = ::select(
      fd_ + 1,
      want_write ? nullptr : &read_set,
      want_write ? &write_set : nullptr,
      nullptr,
      &timeout);
  } while (rc < 0 && errno == EINTR);

  return rc > 0;
}

void RobotTcpClient::drain_input_locked()
{
  if (fd_ < 0) {
    return;
  }

  uint8_t buffer[256];
  while (true) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);
    timeval timeout{};
    const int ready = ::select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return;
    }

    const ssize_t n = ::recv(fd_, buffer, sizeof(buffer), 0);
    if (n > 0) {
      continue;
    }
    if (n == 0) {
      last_error_ = "TCP peer disconnected";
      close_locked();
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return;
    }
    last_error_ = errno_string("TCP drain failed");
    close_locked();
    return;
  }
}

void RobotTcpClient::close_locked()
{
  if (fd_ >= 0) {
    (void)::shutdown(fd_, SHUT_RDWR);
    (void)::close(fd_);
    fd_ = -1;
  }
}

void RobotTcpClient::append_u16_le(std::vector<uint8_t> & out, uint16_t v)
{
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

std::vector<uint8_t> RobotTcpClient::pack_i32_le(int32_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFF),
    static_cast<uint8_t>((v >> 8) & 0xFF),
    static_cast<uint8_t>((v >> 16) & 0xFF),
    static_cast<uint8_t>((v >> 24) & 0xFF)};
}

std::vector<uint8_t> RobotTcpClient::pack_u32_le(uint32_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFF),
    static_cast<uint8_t>((v >> 8) & 0xFF),
    static_cast<uint8_t>((v >> 16) & 0xFF),
    static_cast<uint8_t>((v >> 24) & 0xFF)};
}

int32_t RobotTcpClient::unpack_i32_le(const uint8_t * p)
{
  int32_t v = 0;
  v |= static_cast<int32_t>(p[0]);
  v |= static_cast<int32_t>(p[1]) << 8;
  v |= static_cast<int32_t>(p[2]) << 16;
  v |= static_cast<int32_t>(p[3]) << 24;
  return v;
}

uint32_t RobotTcpClient::unpack_u32_le(const uint8_t * p)
{
  uint32_t v = 0;
  v |= static_cast<uint32_t>(p[0]);
  v |= static_cast<uint32_t>(p[1]) << 8;
  v |= static_cast<uint32_t>(p[2]) << 16;
  v |= static_cast<uint32_t>(p[3]) << 24;
  return v;
}

double RobotTcpClient::clamp(double v, double lo, double hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

int RobotTcpClient::sanitize_timeout_ms(int timeout_ms)
{
  return std::max(0, timeout_ms);
}

void RobotTcpClient::servo_on_axis(uint8_t axis_id)
{
  ensure_success_ack(
    exchange(cmd::SERVO_ON_AXIS, {axis_id, 0x01}, true, 0xA1),
    "SERVO_ON_AXIS");
}

void RobotTcpClient::servo_off_axis(uint8_t axis_id)
{
  ensure_success_ack(
    exchange(cmd::SERVO_ON_AXIS, {axis_id, 0x00}, true, 0xA1),
    "SERVO_OFF_AXIS");
}

void RobotTcpClient::servo_home_axis(uint8_t axis_id)
{
  exchange(cmd::SERVO_HOME_AX, {axis_id}, false);
}

void RobotTcpClient::servo_all(bool on)
{
  const uint8_t state = on ? 0x01 : 0x00;
  ensure_success_ack(
    exchange(cmd::SERVO_ON_ALL, {state}, true, cmd::SERVO_ON_ALL),
    on ? "SERVO_ALL_ON" : "SERVO_ALL_OFF");
}

double RobotTcpClient::get_pos_axis_deg(uint8_t axis_id, int timeout_ms)
{
  const auto rx = exchange(cmd::GET_POS_AXIS, {axis_id}, true, cmd::GET_POS_AXIS, timeout_ms);
  if (!rx.has_value() || rx->size() < sizeof(int32_t)) {
    throw std::runtime_error("Position payload too short");
  }
  const size_t offset = rx->size() >= sizeof(int32_t) + 2 ? 2 : 0;
  const int32_t pos_i32 = unpack_i32_le(rx->data() + offset);
  return static_cast<double>(pos_i32) / 1000.0;
}

void RobotTcpClient::run_axis(uint8_t axis_id, double pos_deg, double vel_deg_s)
{
  pos_deg = clamp(pos_deg, -90.0, 90.0);
  vel_deg_s = clamp(vel_deg_s, 0.001, 89.999);

  const int32_t pos_i = static_cast<int32_t>(std::llround(pos_deg * 1000.0));
  const uint32_t vel_u = static_cast<uint32_t>(std::llround(vel_deg_s * 1000.0));

  std::vector<uint8_t> payload;
  payload.reserve(1 + 4 + 4);
  payload.push_back(axis_id);
  const auto pos = pack_i32_le(pos_i);
  const auto vel = pack_u32_le(vel_u);
  payload.insert(payload.end(), pos.begin(), pos.end());
  payload.insert(payload.end(), vel.begin(), vel.end());
  exchange(cmd::RUN_AXIS, payload, false);
}

void RobotTcpClient::jog(uint8_t axis_id, bool direction_plus, double vel_deg_s)
{
  vel_deg_s = clamp(vel_deg_s, 0.0, 89.999);
  const uint32_t vel_u = static_cast<uint32_t>(std::llround(vel_deg_s * 1000.0));
  const uint8_t dir_b = direction_plus ? 0x01 : 0x00;

  std::vector<uint8_t> payload;
  payload.reserve(1 + 1 + 4);
  payload.push_back(axis_id);
  payload.push_back(dir_b);
  const auto vel = pack_u32_le(vel_u);
  payload.insert(payload.end(), vel.begin(), vel.end());
  exchange(cmd::JOG, payload, false);
}

std::vector<uint32_t> RobotTcpClient::status_all(int timeout_ms)
{
  auto [pos, vel, flags] = get_all_state(timeout_ms);
  (void)pos;
  (void)vel;
  return flags;
}

void RobotTcpClient::run_all(
  const std::vector<double> & pos_deg,
  const std::vector<double> & vel_deg_s)
{
  constexpr size_t kAxes = 8;
  if (pos_deg.size() != kAxes || vel_deg_s.size() != kAxes) {
    throw std::runtime_error("run_all needs 8 pos + 8 vel");
  }

  std::vector<uint8_t> payload;
  payload.reserve(kAxes * (4 + 4));
  for (size_t i = 0; i < kAxes; ++i) {
    const double pos = clamp(pos_deg[i], -280.0, 280.0);
    const double vel = clamp(vel_deg_s[i], -100.0, 100.0);
    const int32_t pos_i = static_cast<int32_t>(std::llround(pos * 1000.0));
    const int32_t vel_i = static_cast<int32_t>(std::llround(vel * 1000.0));
    const auto pos_bytes = pack_i32_le(pos_i);
    const auto vel_bytes = pack_u32_le(static_cast<uint32_t>(vel_i));
    payload.insert(payload.end(), pos_bytes.begin(), pos_bytes.end());
    payload.insert(payload.end(), vel_bytes.begin(), vel_bytes.end());
  }
  exchange(cmd::RUN_ALL, payload, false);
}

std::pair<std::vector<double>, std::vector<double>> RobotTcpClient::get_pos_all(int timeout_ms)
{
  auto [pos, vel, flags] = get_all_state(timeout_ms);
  (void)flags;
  return {pos, vel};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<uint32_t>> RobotTcpClient::get_all_state(
  int timeout_ms)
{
  // STATUS_ALL / CMD_GET_ALL response format:
  // [CMD_OK][int32 pos_mdeg][uint32 vel_mdeg_s][uint32 flag] * 8 axes.
  const auto rx = exchange(cmd::STATUS_ALL, {}, true, cmd::STATUS_ALL, timeout_ms);
  if (!rx.has_value()) {
    throw std::runtime_error("No reply for STATUS_ALL");
  }

  auto parsed = parse_status_all_payload(*rx);

  {
    std::lock_guard<std::mutex> lock(io_mtx_);
    last_state_payload_length_ = rx->size();
    last_state_payload_offset_ = ROBOT_STATUS_ALL_STATUS_BYTES;
    last_state_axis_bytes_ = ROBOT_STATUS_ALL_AXIS_BYTES;
  }

  return parsed;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<uint32_t>>
RobotTcpClient::parse_status_all_payload(const std::vector<uint8_t> & payload)
{
  if (payload.size() != ROBOT_STATUS_ALL_PAYLOAD_SIZE) {
    std::ostringstream oss;
    oss << "CMD_GET_ALL payload length mismatch: expected "
        << ROBOT_STATUS_ALL_PAYLOAD_SIZE
        << " bytes (1 CMD_OK + " << ROBOT_STATUS_FRAME_AXIS_COUNT
        << " axes * " << ROBOT_STATUS_ALL_AXIS_BYTES
        << " bytes), got " << payload.size();
    throw std::runtime_error(oss.str());
  }

  if (payload[0] != ROBOT_CMD_OK) {
    std::ostringstream oss;
    oss << "CMD_GET_ALL response status not OK: payload[0]=0x"
        << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(payload[0]);
    throw std::runtime_error(oss.str());
  }

  constexpr size_t axis_offset = ROBOT_STATUS_ALL_STATUS_BYTES;
  std::vector<double> pos_deg(ROBOT_STATUS_FRAME_AXIS_COUNT, 0.0);
  std::vector<double> vel_deg_s(ROBOT_STATUS_FRAME_AXIS_COUNT, 0.0);
  std::vector<uint32_t> flags(ROBOT_STATUS_FRAME_AXIS_COUNT, 0);

  for (size_t i = 0; i < ROBOT_STATUS_FRAME_AXIS_COUNT; ++i) {
    const uint8_t * base = payload.data() + axis_offset + i * ROBOT_STATUS_ALL_AXIS_BYTES;
    const int32_t pos_i32 = unpack_i32_le(base + 0);
    const uint32_t vel_u32 = unpack_u32_le(base + 4);
    const uint32_t flag_u32 = unpack_u32_le(base + 8);

    pos_deg[i] = static_cast<double>(pos_i32) / 1000.0;
    vel_deg_s[i] = static_cast<double>(vel_u32) / 1000.0;
    flags[i] = flag_u32;
  }

  return {pos_deg, vel_deg_s, flags};
}

}  // namespace robot_hardware_interface
