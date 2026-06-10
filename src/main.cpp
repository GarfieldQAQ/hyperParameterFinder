#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#error "The first implementation targets Windows serial ports. Add a POSIX backend before building on other systems."
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

constexpr uint8_t kJustFloatTail[] = {0x00U, 0x00U, 0x80U, 0x7FU};
constexpr double kDefaultSamplePeriod = 0.01;

struct Args {
  std::string port;
  int baud = 115200;
  std::string loop = "current";
  std::string kp = "0.005:0.05:0.005";
  std::string ki = "0.5:5.0:0.5";
  double targetIq = 0.2;
  double limit = 0.3;
  double settle = 0.5;
  double capture = 2.0;
  double samplePeriod = kDefaultSamplePeriod;
  fs::path output = "output";
};

struct CandidateResult {
  double kp = 0.0;
  double ki = 0.0;
  double score = 0.0;
  double overshoot = 0.0;
  double settleError = 0.0;
  double ripple = 0.0;
  double riseTime = 0.0;
  double saturation = 0.0;
  size_t sampleCount = 0U;
  fs::path csvPath;
};

struct ScoreResult {
  double score = 1.0e9;
  double overshoot = 1.0e9;
  double settleError = 1.0e9;
  double ripple = 1.0e9;
  double riseTime = 0.0;
  double saturation = 1.0e9;
};

class SerialPort {
public:
  SerialPort(const std::string &port, int baud)
  {
    const std::string device = makeDeviceName(port);
    handle_ = CreateFileA(device.c_str(),
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          nullptr,
                          OPEN_EXISTING,
                          FILE_ATTRIBUTE_NORMAL,
                          nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
      throw std::runtime_error("failed to open serial port " + device);
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (GetCommState(handle_, &dcb) == 0) {
      close();
      throw std::runtime_error("GetCommState failed");
    }
    dcb.BaudRate = static_cast<DWORD>(baud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if (SetCommState(handle_, &dcb) == 0) {
      close();
      throw std::runtime_error("SetCommState failed");
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    if (SetCommTimeouts(handle_, &timeouts) == 0) {
      close();
      throw std::runtime_error("SetCommTimeouts failed");
    }

    PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
  }

  ~SerialPort()
  {
    close();
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;

  void writeLine(const std::string &line, bool quiet = false)
  {
    if (!quiet) {
      std::cout << "> " << line << "\n";
    }
    const std::string payload = line + "\r\n";
    DWORD written = 0;
    if (WriteFile(handle_, payload.data(), static_cast<DWORD>(payload.size()), &written, nullptr) == 0 ||
        written != payload.size()) {
      throw std::runtime_error("serial write failed");
    }
  }

  std::vector<uint8_t> readAvailable()
  {
    DWORD errors = 0;
    COMSTAT status{};
    if (ClearCommError(handle_, &errors, &status) == 0) {
      return {};
    }
    if (status.cbInQue == 0U) {
      return {};
    }

    std::vector<uint8_t> data(status.cbInQue);
    DWORD read = 0;
    if (ReadFile(handle_, data.data(), static_cast<DWORD>(data.size()), &read, nullptr) == 0) {
      return {};
    }
    data.resize(read);
    return data;
  }

  std::string drainText(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    std::string text;
    while (std::chrono::steady_clock::now() < end) {
      const auto chunk = readAvailable();
      if (!chunk.empty()) {
        text.append(reinterpret_cast<const char *>(chunk.data()), chunk.size());
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    return text;
  }

private:
  HANDLE handle_ = INVALID_HANDLE_VALUE;

  static std::string makeDeviceName(const std::string &port)
  {
    if (port.rfind("\\\\.\\", 0) == 0) {
      return port;
    }
    return "\\\\.\\" + port;
  }

  void close()
  {
    if (handle_ != INVALID_HANDLE_VALUE) {
      CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
    }
  }
};

class JustFloatParser {
public:
  explicit JustFloatParser(size_t channelCount)
      : channelCount_(channelCount),
        framePayloadSize_(channelCount * sizeof(float)),
        frameSize_(framePayloadSize_ + sizeof(kJustFloatTail))
  {
  }

  std::vector<std::vector<float>> feed(const std::vector<uint8_t> &data)
  {
    buffer_.insert(buffer_.end(), data.begin(), data.end());
    std::vector<std::vector<float>> frames;

    while (true) {
      auto tail = std::search(buffer_.begin(), buffer_.end(), std::begin(kJustFloatTail), std::end(kJustFloatTail));
      if (tail == buffer_.end()) {
        const size_t maxKeep = frameSize_ * 2U;
        if (buffer_.size() > maxKeep) {
          buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(maxKeep));
        }
        break;
      }

      const auto tailIndex = static_cast<size_t>(std::distance(buffer_.begin(), tail));
      if (tailIndex < framePayloadSize_) {
        buffer_.erase(buffer_.begin(), tail + static_cast<std::ptrdiff_t>(sizeof(kJustFloatTail)));
        continue;
      }

      const size_t start = tailIndex - framePayloadSize_;
      std::vector<float> frame(channelCount_);
      std::memcpy(frame.data(), buffer_.data() + start, framePayloadSize_);
      frames.push_back(std::move(frame));
      buffer_.erase(buffer_.begin(), tail + static_cast<std::ptrdiff_t>(sizeof(kJustFloatTail)));
    }

    return frames;
  }

private:
  size_t channelCount_;
  size_t framePayloadSize_;
  size_t frameSize_;
  std::vector<uint8_t> buffer_;
};

static void printUsage()
{
  std::cout
      << "Usage: hyperParameterFinder --port COM7 [options]\n\n"
      << "Options:\n"
      << "  --baud N                 Serial baud rate, default 115200\n"
      << "  --loop current           Loop to tune, current is implemented first\n"
      << "  --kp A:B:S or A,B,C      Current loop Kp search values\n"
      << "  --ki A:B:S or A,B,C      Current loop Ki search values\n"
      << "  --target-iq X            iq step target, default 0.2\n"
      << "  --limit X                FOC output limit, default 0.3\n"
      << "  --settle S               Seconds to settle before step, default 0.5\n"
      << "  --capture S              Seconds to capture after step, default 2.0\n"
      << "  --sample-period S        VOFA frame period, default 0.01\n"
      << "  --output DIR             Output directory, default output\n";
}

static double parseDouble(const std::string &value, const std::string &name)
{
  try {
    size_t used = 0;
    const double result = std::stod(value, &used);
    if (used != value.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error("invalid " + name + ": " + value);
  }
}

static int parseInt(const std::string &value, const std::string &name)
{
  try {
    size_t used = 0;
    const int result = std::stoi(value, &used);
    if (used != value.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error("invalid " + name + ": " + value);
  }
}

static Args parseArgs(int argc, char **argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "-h" || key == "--help") {
      printUsage();
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::runtime_error("missing value for " + key);
    }
    const std::string value = argv[++i];
    if (key == "--port") {
      args.port = value;
    } else if (key == "--baud") {
      args.baud = parseInt(value, key);
    } else if (key == "--loop") {
      args.loop = value;
    } else if (key == "--kp") {
      args.kp = value;
    } else if (key == "--ki") {
      args.ki = value;
    } else if (key == "--target-iq") {
      args.targetIq = parseDouble(value, key);
    } else if (key == "--limit") {
      args.limit = parseDouble(value, key);
    } else if (key == "--settle") {
      args.settle = parseDouble(value, key);
    } else if (key == "--capture") {
      args.capture = parseDouble(value, key);
    } else if (key == "--sample-period") {
      args.samplePeriod = parseDouble(value, key);
    } else if (key == "--output") {
      args.output = value;
    } else {
      throw std::runtime_error("unknown option: " + key);
    }
  }

  if (args.port.empty()) {
    throw std::runtime_error("--port is required");
  }
  if (args.loop != "current") {
    throw std::runtime_error("only --loop current is implemented");
  }
  return args;
}

static std::vector<std::string> split(const std::string &text, char sep)
{
  std::vector<std::string> parts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, sep)) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

static std::vector<double> parseFloatList(const std::string &spec)
{
  if (spec.find(':') != std::string::npos) {
    const auto parts = split(spec, ':');
    if (parts.size() != 3U) {
      throw std::runtime_error("range syntax must be start:stop:step");
    }
    const double start = parseDouble(parts[0], "range start");
    const double stop = parseDouble(parts[1], "range stop");
    const double step = parseDouble(parts[2], "range step");
    if (step == 0.0) {
      throw std::runtime_error("range step cannot be zero");
    }

    std::vector<double> values;
    for (double x = start; (step > 0.0) ? (x <= stop + std::abs(step) * 0.5e-6)
                                        : (x >= stop - std::abs(step) * 0.5e-6);
         x += step) {
      values.push_back(x);
    }
    return values;
  }

  std::vector<double> values;
  for (const auto &part : split(spec, ',')) {
    values.push_back(parseDouble(part, "value"));
  }
  return values;
}

static double standardDeviation(const std::vector<double> &values)
{
  if (values.size() < 2U) {
    return 0.0;
  }
  double mean = 0.0;
  for (double value : values) {
    mean += value;
  }
  mean /= static_cast<double>(values.size());

  double sum = 0.0;
  for (double value : values) {
    const double diff = value - mean;
    sum += diff * diff;
  }
  return std::sqrt(sum / static_cast<double>(values.size() - 1U));
}

static std::vector<std::vector<float>> collectFrames(SerialPort &serial,
                                                     JustFloatParser &parser,
                                                     double durationSeconds)
{
  const auto duration = std::chrono::duration<double>(durationSeconds);
  const auto end = std::chrono::steady_clock::now() + duration;
  std::vector<std::vector<float>> frames;
  while (std::chrono::steady_clock::now() < end) {
    const auto data = serial.readAvailable();
    if (!data.empty()) {
      auto parsed = parser.feed(data);
      frames.insert(frames.end(),
                    std::make_move_iterator(parsed.begin()),
                    std::make_move_iterator(parsed.end()));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  return frames;
}

static ScoreResult scoreCurrentResponse(const std::vector<std::vector<float>> &frames,
                                        double target,
                                        double captureSeconds,
                                        double samplePeriod,
                                        double outputLimit)
{
  ScoreResult result;
  if (frames.size() < 10U) {
    result.riseTime = captureSeconds;
    return result;
  }

  std::vector<double> iq;
  std::vector<double> vq;
  iq.reserve(frames.size());
  vq.reserve(frames.size());
  for (const auto &frame : frames) {
    iq.push_back(frame[3]);
    vq.push_back(frame[7]);
  }

  const double absTarget = std::max(std::abs(target), 1.0e-6);
  const size_t finalWindow = std::max<size_t>(5U, iq.size() / 5U);
  const size_t firstFinal = iq.size() - finalWindow;
  std::vector<double> finalSamples(iq.begin() + static_cast<std::ptrdiff_t>(firstFinal), iq.end());
  double finalMean = 0.0;
  for (double value : finalSamples) {
    finalMean += value;
  }
  finalMean /= static_cast<double>(finalSamples.size());

  const auto peakIt = (target >= 0.0) ? std::max_element(iq.begin(), iq.end())
                                      : std::min_element(iq.begin(), iq.end());
  const double peak = *peakIt;
  if (target >= 0.0) {
    result.overshoot = std::max(0.0, (peak - target) / absTarget);
  } else {
    result.overshoot = std::max(0.0, (target - peak) / absTarget);
  }
  result.settleError = std::abs(finalMean - target) / absTarget;
  result.ripple = standardDeviation(finalSamples) / absTarget;

  const double threshold = target * 0.9;
  result.riseTime = captureSeconds;
  for (size_t i = 0; i < iq.size(); ++i) {
    if ((target >= 0.0 && iq[i] >= threshold) ||
        (target < 0.0 && iq[i] <= threshold)) {
      result.riseTime = static_cast<double>(i) * samplePeriod;
      break;
    }
  }

  size_t saturationCount = 0U;
  for (double value : vq) {
    if (std::abs(value) >= outputLimit * 0.98) {
      saturationCount++;
    }
  }
  result.saturation = static_cast<double>(saturationCount) / static_cast<double>(vq.size());
  result.score = result.overshoot * 4.0 +
                 result.settleError * 5.0 +
                 result.ripple * 3.0 +
                 result.saturation * 2.0 +
                 std::min(result.riseTime / std::max(captureSeconds, samplePeriod), 2.0);
  return result;
}

static std::string candidateName(double kp, double ki)
{
  std::ostringstream oss;
  oss << std::setprecision(6) << "current_kp_" << kp << "_ki_" << ki;
  std::string name = oss.str();
  std::replace(name.begin(), name.end(), '.', 'p');
  std::replace(name.begin(), name.end(), '-', 'm');
  return name;
}

static void writeFramesCsv(const fs::path &path,
                           const std::vector<std::vector<float>> &frames,
                           double samplePeriod)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "t_s,target_id,target_iq,id,iq,id_raw,iq_raw,vd,vq,va,vb,id_i,iq_i\n";
  out << std::setprecision(9);
  for (size_t i = 0; i < frames.size(); ++i) {
    out << static_cast<double>(i) * samplePeriod;
    for (float value : frames[i]) {
      out << ',' << value;
    }
    out << '\n';
  }
}

static void writeSummaryCsv(const fs::path &path, std::vector<CandidateResult> results)
{
  fs::create_directories(path.parent_path());
  std::sort(results.begin(), results.end(), [](const CandidateResult &a, const CandidateResult &b) {
    return a.score < b.score;
  });

  std::ofstream out(path);
  out << "rank,kp,ki,score,overshoot,settle_error,ripple,rise_time_s,saturation,sample_count,csv_path\n";
  out << std::setprecision(9);
  size_t rank = 1U;
  for (const auto &result : results) {
    out << rank++ << ','
        << result.kp << ','
        << result.ki << ','
        << result.score << ','
        << result.overshoot << ','
        << result.settleError << ','
        << result.ripple << ','
        << result.riseTime << ','
        << result.saturation << ','
        << result.sampleCount << ','
        << result.csvPath.string() << '\n';
  }
}

static CandidateResult runCurrentCandidate(SerialPort &serial,
                                           double kp,
                                           double ki,
                                           const Args &args,
                                           const fs::path &outputDir)
{
  JustFloatParser parser(12U);

  serial.writeLine("vofa off");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("foc off");
  serial.drainText(std::chrono::milliseconds(100));
  serial.writeLine("foc prep current");
  serial.drainText(std::chrono::milliseconds(200));
  serial.writeLine("foc limit " + std::to_string(args.limit));
  serial.writeLine("foc ckp " + std::to_string(kp));
  serial.writeLine("foc cki " + std::to_string(ki));
  serial.writeLine("foc id 0");
  serial.writeLine("foc iq 0");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("vofa current on");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  serial.writeLine("foc on");
  std::this_thread::sleep_for(std::chrono::duration<double>(args.settle));
  (void)collectFrames(serial, parser, 0.2);

  serial.writeLine("foc iq " + std::to_string(args.targetIq), true);
  auto frames = collectFrames(serial, parser, args.capture);
  serial.writeLine("foc iq 0", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  serial.writeLine("foc off", true);
  serial.writeLine("vofa off", true);
  serial.drainText(std::chrono::milliseconds(200));

  const fs::path csvPath = outputDir / (candidateName(kp, ki) + ".csv");
  writeFramesCsv(csvPath, frames, args.samplePeriod);

  const auto score = scoreCurrentResponse(frames, args.targetIq, args.capture, args.samplePeriod, args.limit);
  CandidateResult result;
  result.kp = kp;
  result.ki = ki;
  result.score = score.score;
  result.overshoot = score.overshoot;
  result.settleError = score.settleError;
  result.ripple = score.ripple;
  result.riseTime = score.riseTime;
  result.saturation = score.saturation;
  result.sampleCount = frames.size();
  result.csvPath = csvPath;
  return result;
}

static std::string timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_s(&local, &t);
  std::ostringstream oss;
  oss << std::put_time(&local, "%Y%m%d_%H%M%S");
  return oss.str();
}

static int runCurrentSearch(const Args &args)
{
  const auto kpValues = parseFloatList(args.kp);
  const auto kiValues = parseFloatList(args.ki);
  const fs::path outputDir = args.output / ("current_" + timestamp());
  fs::create_directories(outputDir);

  std::cout << "Opening " << args.port << " at " << args.baud << " baud\n";
  SerialPort serial(args.port, args.baud);

  std::vector<CandidateResult> results;
  const size_t total = kpValues.size() * kiValues.size();
  size_t index = 0U;

  try {
    for (double kp : kpValues) {
      for (double ki : kiValues) {
        index++;
        std::cout << "\n[" << index << "/" << total
                  << "] current loop kp=" << kp
                  << ", ki=" << ki << "\n";
        auto result = runCurrentCandidate(serial, kp, ki, args, outputDir);
        results.push_back(result);
        std::cout << "score=" << result.score
                  << " overshoot=" << result.overshoot
                  << " settle=" << result.settleError
                  << " ripple=" << result.ripple
                  << " rise=" << result.riseTime << "s"
                  << " sat=" << result.saturation
                  << " samples=" << result.sampleCount << "\n";
      }
    }
  } catch (...) {
    try {
      serial.writeLine("foc iq 0", true);
      serial.writeLine("foc off", true);
      serial.writeLine("vofa off", true);
    } catch (...) {
    }
    throw;
  }

  serial.writeLine("foc iq 0", true);
  serial.writeLine("foc off", true);
  serial.writeLine("vofa off", true);

  if (results.empty()) {
    throw std::runtime_error("no candidate result");
  }

  const fs::path summaryPath = outputDir / "summary.csv";
  writeSummaryCsv(summaryPath, results);
  const auto best = std::min_element(results.begin(), results.end(), [](const auto &a, const auto &b) {
    return a.score < b.score;
  });

  std::cout << "\nBest current PI:\n"
            << "  ckp=" << best->kp << "\n"
            << "  cki=" << best->ki << "\n"
            << "  score=" << best->score << "\n"
            << "  summary=" << summaryPath.string() << "\n";
  return 0;
}

int main(int argc, char **argv)
{
  try {
    const auto args = parseArgs(argc, argv);
    return runCurrentSearch(args);
  } catch (const std::exception &ex) {
    std::cerr << "error: " << ex.what() << "\n";
    return 1;
  }
}
