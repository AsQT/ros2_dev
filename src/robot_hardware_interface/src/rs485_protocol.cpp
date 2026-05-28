#include "robot_hardware_interface/rs485_protocol.hpp"

#include <stdexcept>
#include <cstring>
#include <cmath>

// Linux serial (Ubuntu)
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace robot_hardware_interface {
// Header/Tail/Stuff bytes are configurable via Rs485Client::set_protocol().
// Defaults are defined in ProtocolBytes.

uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}
/*__________________________________________________________________________________*/
std::vector<uint8_t> stuff_bytes(const std::vector<uint8_t>& raw, uint8_t stuff_byte) {
  std::vector<uint8_t> out;
  out.reserve(raw.size() * 2);
  for (auto b : raw) {
    out.push_back(b);
    if (b == stuff_byte) out.push_back(stuff_byte);
  }
  return out;
}
/*__________________________________________________________________________________*/
std::vector<uint8_t> destuff_bytes(const std::vector<uint8_t>& stuffed, uint8_t stuff_byte) {
  std::vector<uint8_t> out;
  out.reserve(stuffed.size());
  for (size_t i = 0; i < stuffed.size();) {
    uint8_t b = stuffed[i];
    out.push_back(b);
    if (b == stuff_byte && (i + 1) < stuffed.size() && stuffed[i + 1] == stuff_byte) {
      i += 2;
    } else {
      i += 1;
    }
  }
  return out;
}
/*__________________________________________________________________________________*/
static speed_t baud_to_termios(int baudrate) {
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}
/*________________  SerialPort ________________*/
SerialPort::~SerialPort() { close(); }

void SerialPort::open(
                      const   std::string& port, 
                      int     baudrate, 
                      double  timeout_s) 
{
  close();
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) throw std::runtime_error("open() serial failed: " + port);
  timeout_s_ = timeout_s;

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr() failed");
  }
  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  speed_t spd = baud_to_termios(baudrate);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr() failed");
  }
  flush_input();
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const { return fd_ >= 0; }

void SerialPort::set_timeout(double timeout_s) { timeout_s_ = timeout_s; }
/*__________________________________________________________________________________*/
void SerialPort::flush_input() {
  if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}
/*__________________________________________________________________________________*/
void SerialPort::write_all(const uint8_t* data, size_t len) {
  
  if (fd_ < 0) throw std::runtime_error("Serial not open");
  size_t off = 0;
  while (off < len) {
    ssize_t n = ::write(fd_, data + off, len - off);
    if (n < 0) throw std::runtime_error("Serial write failed");
    off += static_cast<size_t>(n);
  }
  ::tcdrain(fd_);
}
/*__________________________________________________________________________________*/
bool SerialPort::read_byte(uint8_t& out) 
{
  if (fd_ < 0) throw std::runtime_error("Serial not open");
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd_, &set);

  timeval tv{};
  tv.tv_sec = static_cast<int>(timeout_s_);
  tv.tv_usec = static_cast<int>((timeout_s_ - tv.tv_sec) * 1e6);

  int rv = ::select(fd_ + 1, &set, nullptr, nullptr, &tv);
  if (rv <= 0) return false;

  uint8_t b = 0;
  ssize_t n = ::read(fd_, &b, 1);
  if (n == 1) { out = b; return true; }
  return false;
}

/*____________ Rs485Client ____________*/
void Rs485Client::set_protocol(const ProtocolBytes & p) 
{
  std::lock_guard<std::mutex> lk(io_mtx_);
  proto_ = p;

  if (proto_.header0 == 0) proto_.header0 = 0xAA;
  if (proto_.tail0 == 0) proto_.tail0 = 0xAA;
  if (proto_.stuff == 0) proto_.stuff = 0xAA;
}

ProtocolBytes Rs485Client::protocol() const 
{
  std::lock_guard<std::mutex> lk(io_mtx_);
  return proto_;
}
/*__________________________________________________________________________________*/
void Rs485Client::connect(
                          const std::string& port, 
                          int baudrate, 
                          double timeout_s) {
  std::lock_guard<std::mutex> lk(io_mtx_);
  sp_.open(port, baudrate, timeout_s);
}
/*__________________________________________________________________________________*/
void Rs485Client::disconnect() {
  std::lock_guard<std::mutex> lk(io_mtx_);
  sp_.close();
}
/*__________________________________________________________________________________*/
bool Rs485Client::is_connected() const { return sp_.is_open(); }

// --------------------- Cac ham tien ich --------------------------------
std::vector<uint8_t> Rs485Client::pack_i32_le(int32_t v) {
  return {static_cast<uint8_t>(v & 0xFF),
          static_cast<uint8_t>((v >> 8) & 0xFF),
          static_cast<uint8_t>((v >> 16) & 0xFF),
          static_cast<uint8_t>((v >> 24) & 0xFF)};
}
/*__________________________________________________________________________________*/
std::vector<uint8_t> Rs485Client::pack_u32_le(uint32_t v) {
  return {static_cast<uint8_t>(v & 0xFF),
          static_cast<uint8_t>((v >> 8) & 0xFF),
          static_cast<uint8_t>((v >> 16) & 0xFF),
          static_cast<uint8_t>((v >> 24) & 0xFF)};
}
/*__________________________________________________________________________________*/
int32_t Rs485Client::unpack_i32_le(const uint8_t* p) {
  int32_t v = 0;
  v |= (int32_t)p[0];
  v |= (int32_t)p[1] << 8;
  v |= (int32_t)p[2] << 16;
  v |= (int32_t)p[3] << 24;
  return v;
}
/*__________________________________________________________________________________*/
uint32_t Rs485Client::unpack_u32_le(const uint8_t* p) {
  uint32_t v = 0;
  v |= (uint32_t)p[0];
  v |= (uint32_t)p[1] << 8;
  v |= (uint32_t)p[2] << 16;
  v |= (uint32_t)p[3] << 24;
  return v;
}
/*__________________________________________________________________________________*/
uint16_t Rs485Client::unpack_u16_le(const uint8_t* p) {
  uint16_t v = 0;
  v |= (uint16_t)p[0];
  v |= (uint16_t)p[1] << 8;
  return v;
}
/*__________________________________________________________________________________*/
double Rs485Client::clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
/*__________________________________________________________________________________*/
static inline const char* frame_err_str(uint8_t err_cmd)
{
  switch (err_cmd) {
    case 0xE1: return "Frame_ERR_LEN";
    case 0xE2: return "Frame_ERR_HEADER";
    case 0xE3: return "Frame_ERR_TAIL";
    case 0xE4: return "Frame_ERR_PAYLOAD";
    case 0xE5: return "Frame_ERR_CRC";
    default:   return "Frame_ERR_UNKNOWN";
  }
}
/*___________________________________________________________________*/
static inline std::string hex_dump(const std::vector<uint8_t>& v)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < v.size(); ++i) {
    oss << "0x" << std::setw(2) << static_cast<int>(v[i]);
    if (i + 1 < v.size()) oss << ' ';
  }
  return oss.str();
}
/*_______________ build_frame _______________*/
std::vector<uint8_t> Rs485Client::build_frame(
                                              uint8_t cmd, 
                                              const std::vector<uint8_t>& payload) {
  const auto p = proto_;
  std::vector<uint8_t> body;
  body.reserve(1 + payload.size() + 2);
  body.push_back(cmd);
  body.insert(body.end(), payload.begin(), payload.end());

  uint16_t crc = crc16_modbus(body.data(), body.size());
  body.push_back(static_cast<uint8_t>(crc & 0xFF));
  body.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  auto stuffed = stuff_bytes(body, p.stuff);

  std::vector<uint8_t> frame;
  frame.reserve(2 + stuffed.size() + 2);
  frame.push_back(p.header0);
  frame.push_back(p.header1);
  frame.insert(frame.end(), stuffed.begin(), stuffed.end());
  frame.push_back(p.tail0);
  frame.push_back(p.tail1);
  return frame;
}
// --------------------- recv_frame --------------------------------
Frame Rs485Client::recv_frame(std::optional<uint8_t> expected_cmd, double timeout_s)
{
  if (!sp_.is_open()) throw std::runtime_error("Serial not connected");
  sp_.set_timeout(timeout_s);

  const auto p = proto_;

  // ---- find HEADER ----
  uint8_t b = 0, prev = 0;
  bool have_prev = false;
  while (true) {
    if (!sp_.read_byte(b)) throw std::runtime_error("Timeout waiting HEADER");
    if (have_prev && prev == p.header0 && b == p.header1) break;
    prev = b;
    have_prev = true;
  }

  // ---- read until TAIL ----
  std::vector<uint8_t> body_stuffed;
  while (true) {
    if (!sp_.read_byte(b)) throw std::runtime_error("Timeout waiting TAIL");
    body_stuffed.push_back(b);
    size_t n = body_stuffed.size();
    if (n >= 2 && body_stuffed[n - 2] == p.tail0 && body_stuffed[n - 1] == p.tail1) {
      body_stuffed.resize(n - 2);
      break;
    }
  }

  auto body = destuff_bytes(body_stuffed, p.stuff);

  // [status][cmd][payload...][crcL][crcH] => tối thiểu 4 byte
  if (body.size() < 4) {
    std::ostringstream oss;
    oss << "Frame too short: size=" << body.size();
    std::cerr << "[RS485][ERROR] " << oss.str() << "\n";
    throw std::runtime_error(oss.str());
  }

  const uint8_t cmd_status = body[0];
  const uint8_t cmd        = body[1];

  const uint16_t crc_recv =
      static_cast<uint16_t>(body[body.size() - 2]) |
      (static_cast<uint16_t>(body[body.size() - 1]) << 8);

  std::vector<uint8_t> payload(body.begin() + 2, body.end() - 2);

  // CRC on [cmd_status][cmd][payload]
  std::vector<uint8_t> calc_in;
  calc_in.reserve(2 + payload.size());
  calc_in.push_back(cmd_status);
  calc_in.push_back(cmd);
  calc_in.insert(calc_in.end(), payload.begin(), payload.end());

  const uint16_t crc_calc = crc16_modbus(calc_in.data(), calc_in.size());
  if (crc_calc != crc_recv) {
    std::ostringstream oss;
    oss << "CRC error: calc=0x" << std::hex << std::setw(4) << std::setfill('0') << crc_calc
        << " recv=0x" << std::setw(4) << crc_recv
        << " | status=0x" << std::setw(2) << static_cast<int>(cmd_status)
        << " cmd=0x" << std::setw(2) << static_cast<int>(cmd)
        << " payload=[" << hex_dump(payload) << "]";
    std::cerr << "[RS485][ERROR] " << oss.str() << "\n";
    throw std::runtime_error(oss.str());
  }
  
  if (cmd_status == 0xEE) {
    std::ostringstream oss;
    oss << "FRAME_ERR: " << frame_err_str(cmd)
        << " (err_cmd=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd) << ")"
        << " payload=[" << hex_dump(payload) << "]";
    std::cerr << "[RS485][ERROR] " << oss.str() << "\n";
    throw std::runtime_error(oss.str());
  }

  if (expected_cmd.has_value() && cmd != expected_cmd.value()) {
    std::ostringstream oss;
    oss << "CMD mismatch: expected=0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(expected_cmd.value())
        << " got=0x" << std::setw(2) << static_cast<int>(cmd)
        << " status=0x" << std::setw(2) << static_cast<int>(cmd_status);
    std::cerr << "[RS485][ERROR] " << oss.str() << "\n";
    throw std::runtime_error(oss.str());
  }
  return Frame{cmd, payload};
}
/*__________________________________________________________________________________*/
std::optional<std::vector<uint8_t>> Rs485Client::exchange(
                                                          uint8_t cmd,
                                                          const std::vector<uint8_t>& payload,
                                                          bool wait_reply,
                                                          std::optional<uint8_t> expected_cmd,
                                                          double timeout_s) {
  std::lock_guard<std::mutex> lk(io_mtx_);
  if (!sp_.is_open()) throw std::runtime_error("Serial not connected");

  sp_.flush_input();

  auto frame = build_frame(cmd, payload);
  sp_.write_all(frame.data(), frame.size());

  if (!wait_reply) return std::nullopt;

  uint8_t exp = expected_cmd.has_value() ? expected_cmd.value() : cmd;
  auto fr = recv_frame(exp, timeout_s);
  return fr.payload;
}
/*___________________ api high level ___________________________*/
// ---  servo_on_axis ---
void Rs485Client::servo_on_axis(uint8_t axis_id) {
  exchange(cmd::SERVO_ON_AXIS, {axis_id, 0x01}, false);
}
// ---  servo_off_axis ---
void Rs485Client::servo_off_axis(uint8_t axis_id) {
  exchange(cmd::SERVO_ON_AXIS, {axis_id, 0x00}, false);
}
// ---  servo_all ---
void Rs485Client::servo_all(bool on) {
  const uint8_t state = on ? 0x01 : 0x00;
  exchange(cmd::SERVO_ON_ALL, {state}, false);
}
// ---  servo_home_axis ---
void Rs485Client::servo_home_axis(uint8_t axis_id) {
  exchange(cmd::SERVO_HOME_AX, {axis_id}, false);
}
// --- get_pos_axis_deg ---
double Rs485Client::get_pos_axis_deg(uint8_t axis_id, double timeout_s) {
  auto rx = exchange(cmd::GET_POS_AXIS, {axis_id}, true, cmd::GET_POS_AXIS, timeout_s);
  if (!rx.has_value() || rx->size() < 6) throw std::runtime_error("Position payload too short");
  int32_t pos_i32 = unpack_i32_le(rx->data() + 2);
  return static_cast<double>(pos_i32) / 1000.0;
}
// --- get_all_sate ---
std::tuple<
            std::vector<double>, 
            std::vector<double>, 
            std::vector<uint16_t>
            >
Rs485Client::get_all_state(double timeout_s)
{
  constexpr size_t AXES = 8;
  constexpr size_t BYTES_PER_AXIS = 10; // 4 pos + 4 vel +2
  const size_t need_bytes = AXES * BYTES_PER_AXIS; // 64
  std::vector<double> pos_deg(AXES, 0.0);
  std::vector<double> vel_deg_s(AXES, 0.0);
  std::vector<uint16_t> flags(AXES, 0);

  auto rx = exchange(cmd::STATUS_ALL, {}, true, cmd::STATUS_ALL, timeout_s);
  if (!rx.has_value()) throw std::runtime_error("No reply for STATUS_ALL");
  size_t offset = 0;
  if (rx->size() == need_bytes) {
    offset = 0;
  } else if (rx->size() >= need_bytes + 2) {
    offset = 2; // nếu firmware có prefix 2 byte
  } else {
    throw std::runtime_error("STATUS_ALL payload too short");
  }

  if (rx->size() < offset + need_bytes) {
    throw std::runtime_error("STATUS_ALL payload truncated");
  }

  for (size_t i = 0; i < AXES; ++i) {
    const uint8_t* base = rx->data() + offset + i * BYTES_PER_AXIS;  // i*8

    const int32_t  pos_i32 = unpack_i32_le(base + 0);
    const int32_t vel_u32 = unpack_u32_le(base + 4);
    const uint16_t flag_i   = unpack_u16_le(base + 8);

    pos_deg[i]    = static_cast<double>(pos_i32) / 1000.0; // mdeg -> deg
    vel_deg_s[i]  = static_cast<double>(vel_u32) / 1000.0; // mdeg/s -> deg/s
    flags[i]      =  flag_i;
  }

  return {pos_deg, vel_deg_s, flags};
}
// --- run_axis ---
  void Rs485Client::run_axis(uint8_t axis_id, double pos_deg, double vel_deg_s) {
    pos_deg = clamp(pos_deg, -90.0, 90.0);
    vel_deg_s = clamp(vel_deg_s, 0.001, 89.999);
    int32_t pos_i = (int32_t)llround(pos_deg * 1000.0);
    uint32_t vel_u = (uint32_t)llround(vel_deg_s * 1000.0);

    std::vector<uint8_t> payload;
    payload.reserve(1 + 4 + 4);
    payload.push_back(axis_id);
    auto p = pack_i32_le(pos_i);
    auto v = pack_u32_le(vel_u);
    payload.insert(payload.end(), p.begin(), p.end());
    payload.insert(payload.end(), v.begin(), v.end());
    exchange(cmd::RUN_AXIS, payload, false);
  }
// --- jog  ---
  void Rs485Client::jog(uint8_t axis_id, bool direction_plus, double vel_deg_s) {
    if (vel_deg_s < 0) vel_deg_s = 0.0;
    if (vel_deg_s > 89.999) vel_deg_s = 89.999;
    uint32_t vel_u = (uint32_t)llround(vel_deg_s * 1000.0);
    uint8_t dir_b = direction_plus ? 0x01 : 0x00;

    auto v = pack_u32_le(vel_u);
    std::vector<uint8_t> payload;
    payload.reserve(1 + 1 + 4);
    payload.push_back(axis_id);
    payload.push_back(dir_b);
    payload.insert(payload.end(), v.begin(), v.end());
    exchange(cmd::JOG, payload, false);
  }

// --- run_all ---
  void Rs485Client::run_all(const std::vector<double>& pos6_deg, const std::vector<double>& vel6_deg_s) 
  {
    if (pos6_deg.size() != 8 || vel6_deg_s.size() != 8) throw std::runtime_error("run_all needs 8 pos + 8 vel");
    std::vector<uint8_t> payload;
    payload.reserve(8 * (4 + 4));
    for (int i = 0; i < 7; ++i) {
      double p = clamp(pos6_deg[i], -280.0, 280.0);
      double v = clamp(vel6_deg_s[i], -100, 100);
      int32_t pi = (int32_t)llround(p * 1000.0);
      int32_t vu = (int32_t)llround(v * 1000.0);
      auto pb = pack_i32_le(pi);
      auto vb = pack_u32_le(vu);
      payload.insert(payload.end(), pb.begin(), pb.end());
      payload.insert(payload.end(), vb.begin(), vb.end());
    }
    exchange(cmd::RUN_ALL, payload, false);
  }
}  // namespace robot_hardware_interface
