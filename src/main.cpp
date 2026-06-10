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
#include <numeric>
#include <random>
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
  std::string optimizer = "grid";
  std::string kp = "0.005:0.05:0.005";
  std::string ki = "0.5:5.0:0.5";
  double targetIq = 0.2;
  double targetVelocity = 2.0;
  double currentKp = 0.0491197815;
  double currentKi = 7.99676751;
  double limit = 0.3;
  double closedLoopDuty = 0.18;
  double settle = 0.5;
  double capture = 2.0;
  double samplePeriod = kDefaultSamplePeriod;
  size_t boInitial = 5U;
  size_t boIterations = 20U;
  size_t boCandidates = 400U;
  double boXi = 0.01;
  double boNoise = 1.0e-6;
  double boLengthScale = 0.35;
  double initialKp = 0.02;
  double initialKi = 2.0;
  uint32_t seed = 42U;
  fs::path output = "output";
};

struct Range {
  double min = 0.0;
  double max = 0.0;
};

struct ParamPoint {
  double kp = 0.0;
  double ki = 0.0;
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
  double effectiveRatio = 0.0;
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
  double effectiveRatio = 0.0;
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
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;
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
    const std::string payload = line + "\r";
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
      << "  --loop current|velocity  Loop to tune, default current\n"
      << "  --optimizer grid|bo      Search strategy, default grid\n"
      << "  --kp A:B:S or A,B,C      Current loop Kp search values\n"
      << "  --ki A:B:S or A,B,C      Current loop Ki search values\n"
      << "  --bo-initial N           Initial BO design size, default 5\n"
      << "  --bo-iterations N        BO iterations after initial design, default 20\n"
      << "  --bo-candidates N        EI candidate pool size, default 400\n"
      << "  --bo-xi X                EI exploration offset, default 0.01\n"
      << "  --bo-noise X             GP diagonal noise, default 1e-6\n"
      << "  --bo-length-scale X      RBF length scale in normalized space, default 0.35\n"
      << "  --initial-kp X           Baseline Kp tested first in BO, default 0.02\n"
      << "  --initial-ki X           Baseline Ki tested first in BO, default 2.0\n"
      << "  --seed N                 Random seed, default 42\n"
      << "  --target-iq X            iq step target, default 0.2\n"
      << "  --target-vel X           velocity step target rad/s, default 2.0\n"
      << "  --current-kp X           Inner current loop Kp for velocity tests, default 0.0491197815\n"
      << "  --current-ki X           Inner current loop Ki for velocity tests, default 7.99676751\n"
      << "  --limit X                FOC output limit, default 0.3\n"
      << "  --closed-loop-duty X     Minimum duty considered closed-loop valid, default 0.18\n"
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
    } else if (key == "--optimizer") {
      args.optimizer = value;
    } else if (key == "--kp") {
      args.kp = value;
    } else if (key == "--ki") {
      args.ki = value;
    } else if (key == "--target-iq") {
      args.targetIq = parseDouble(value, key);
    } else if (key == "--target-vel") {
      args.targetVelocity = parseDouble(value, key);
    } else if (key == "--current-kp") {
      args.currentKp = parseDouble(value, key);
    } else if (key == "--current-ki") {
      args.currentKi = parseDouble(value, key);
    } else if (key == "--limit") {
      args.limit = parseDouble(value, key);
    } else if (key == "--closed-loop-duty") {
      args.closedLoopDuty = parseDouble(value, key);
    } else if (key == "--settle") {
      args.settle = parseDouble(value, key);
    } else if (key == "--capture") {
      args.capture = parseDouble(value, key);
    } else if (key == "--sample-period") {
      args.samplePeriod = parseDouble(value, key);
    } else if (key == "--bo-initial") {
      args.boInitial = static_cast<size_t>(parseInt(value, key));
    } else if (key == "--bo-iterations") {
      args.boIterations = static_cast<size_t>(parseInt(value, key));
    } else if (key == "--bo-candidates") {
      args.boCandidates = static_cast<size_t>(parseInt(value, key));
    } else if (key == "--bo-xi") {
      args.boXi = parseDouble(value, key);
    } else if (key == "--bo-noise") {
      args.boNoise = parseDouble(value, key);
    } else if (key == "--bo-length-scale") {
      args.boLengthScale = parseDouble(value, key);
    } else if (key == "--initial-kp") {
      args.initialKp = parseDouble(value, key);
    } else if (key == "--initial-ki") {
      args.initialKi = parseDouble(value, key);
    } else if (key == "--seed") {
      args.seed = static_cast<uint32_t>(parseInt(value, key));
    } else if (key == "--output") {
      args.output = value;
    } else {
      throw std::runtime_error("unknown option: " + key);
    }
  }

  if (args.port.empty()) {
    throw std::runtime_error("--port is required");
  }
  if (args.loop != "current" && args.loop != "velocity") {
    throw std::runtime_error("--loop must be current or velocity");
  }
  if (args.optimizer != "grid" && args.optimizer != "bo") {
    throw std::runtime_error("--optimizer must be grid or bo");
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

static Range parseRange(const std::string &spec)
{
  const auto values = parseFloatList(spec);
  if (values.empty()) {
    throw std::runtime_error("empty parameter range");
  }
  const auto [minIt, maxIt] = std::minmax_element(values.begin(), values.end());
  if (*minIt == *maxIt) {
    throw std::runtime_error("Bayesian optimization requires non-zero parameter range");
  }
  return {*minIt, *maxIt};
}

static double normalize(double value, Range range)
{
  return (value - range.min) / (range.max - range.min);
}

static double denormalize(double value, Range range)
{
  return range.min + value * (range.max - range.min);
}

static double standardDeviation(const std::vector<double> &values);

static bool nearlySamePoint(const ParamPoint &a, const ParamPoint &b)
{
  return std::abs(a.kp - b.kp) < 1.0e-12 && std::abs(a.ki - b.ki) < 1.0e-12;
}

static bool wasEvaluated(const std::vector<CandidateResult> &results, const ParamPoint &point)
{
  return std::any_of(results.begin(), results.end(), [&](const CandidateResult &result) {
    return nearlySamePoint({result.kp, result.ki}, point);
  });
}

static double normalPdf(double x)
{
  constexpr double kInvSqrt2Pi = 0.39894228040143267794;
  return kInvSqrt2Pi * std::exp(-0.5 * x * x);
}

static double normalCdf(double x)
{
  return 0.5 * (1.0 + std::erf(x / std::sqrt(2.0)));
}

static double rbfKernel(const ParamPoint &a, const ParamPoint &b, double lengthScale)
{
  const double dx = a.kp - b.kp;
  const double dy = a.ki - b.ki;
  const double r2 = dx * dx + dy * dy;
  const double ls = std::max(lengthScale, 1.0e-6);
  return std::exp(-0.5 * r2 / (ls * ls));
}

static std::vector<double> solveLinearSystem(std::vector<std::vector<double>> matrix,
                                             std::vector<double> rhs)
{
  const size_t n = rhs.size();
  for (size_t col = 0; col < n; ++col) {
    size_t pivot = col;
    double bestAbs = std::abs(matrix[col][col]);
    for (size_t row = col + 1U; row < n; ++row) {
      const double candidate = std::abs(matrix[row][col]);
      if (candidate > bestAbs) {
        bestAbs = candidate;
        pivot = row;
      }
    }
    if (bestAbs < 1.0e-12) {
      throw std::runtime_error("Gaussian process kernel matrix is singular");
    }
    if (pivot != col) {
      std::swap(matrix[pivot], matrix[col]);
      std::swap(rhs[pivot], rhs[col]);
    }

    const double divisor = matrix[col][col];
    for (size_t j = col; j < n; ++j) {
      matrix[col][j] /= divisor;
    }
    rhs[col] /= divisor;

    for (size_t row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = matrix[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (size_t j = col; j < n; ++j) {
        matrix[row][j] -= factor * matrix[col][j];
      }
      rhs[row] -= factor * rhs[col];
    }
  }
  return rhs;
}

static void addUniquePoint(std::vector<ParamPoint> &points, ParamPoint point)
{
  const auto it = std::find_if(points.begin(), points.end(), [&](const ParamPoint &existing) {
    return nearlySamePoint(existing, point);
  });
  if (it == points.end()) {
    points.push_back(point);
  }
}

static std::vector<ParamPoint> initialDesign(Range kpRange,
                                             Range kiRange,
                                             size_t requested,
                                             ParamPoint baseline)
{
  baseline.kp = std::clamp(baseline.kp, kpRange.min, kpRange.max);
  baseline.ki = std::clamp(baseline.ki, kiRange.min, kiRange.max);

  std::vector<ParamPoint> points;
  addUniquePoint(points, baseline);
  addUniquePoint(points, {kpRange.min, kiRange.min});
  addUniquePoint(points, {kpRange.max, kiRange.min});
  addUniquePoint(points, {kpRange.min, kiRange.max});
  addUniquePoint(points, {kpRange.max, kiRange.max});
  addUniquePoint(points, {denormalize(0.5, kpRange), denormalize(0.5, kiRange)});

  if (requested > points.size()) {
    for (size_t i = points.size(); i < requested; ++i) {
      const double t = static_cast<double>(i - points.size() + 1U) /
                       static_cast<double>(requested - points.size() + 1U);
      addUniquePoint(points, {denormalize(t, kpRange), denormalize(1.0 - t, kiRange)});
    }
  }
  if (requested < points.size()) {
    points.resize(requested);
  }
  return points;
}

static ParamPoint proposeBayesianCandidate(const std::vector<CandidateResult> &results,
                                           Range kpRange,
                                           Range kiRange,
                                           const Args &args,
                                           std::mt19937 &rng)
{
  const size_t n = results.size();
  if (n == 0U) {
    return {denormalize(0.5, kpRange), denormalize(0.5, kiRange)};
  }

  std::vector<ParamPoint> x(n);
  std::vector<double> y(n);
  for (size_t i = 0; i < n; ++i) {
    x[i] = {
        normalize(results[i].kp, kpRange),
        normalize(results[i].ki, kiRange)};
    y[i] = results[i].score;
  }

  const double yMean = std::accumulate(y.begin(), y.end(), 0.0) / static_cast<double>(y.size());
  double yStd = standardDeviation(y);
  if (yStd < 1.0e-9) {
    yStd = 1.0;
  }
  std::vector<double> yNorm(n);
  for (size_t i = 0; i < n; ++i) {
    yNorm[i] = (y[i] - yMean) / yStd;
  }

  std::vector<std::vector<double>> kernel(n, std::vector<double>(n, 0.0));
  for (size_t row = 0; row < n; ++row) {
    for (size_t col = 0; col < n; ++col) {
      kernel[row][col] = rbfKernel(x[row], x[col], args.boLengthScale);
    }
    kernel[row][row] += std::max(args.boNoise, 1.0e-12);
  }
  const auto alpha = solveLinearSystem(kernel, yNorm);
  const double bestNorm = (*std::min_element(y.begin(), y.end()) - yMean) / yStd;

  std::uniform_real_distribution<double> unit(0.0, 1.0);
  ParamPoint bestPoint{0.5, 0.5};
  double bestEi = -1.0;
  const size_t candidateCount = std::max<size_t>(args.boCandidates, 16U);

  for (size_t i = 0; i < candidateCount; ++i) {
    ParamPoint candidate{};
    if (i < 4U) {
      candidate = {
          (i & 1U) ? 1.0 : 0.0,
          (i & 2U) ? 1.0 : 0.0};
    } else if (i == 4U) {
      candidate = {0.5, 0.5};
    } else {
      candidate = {unit(rng), unit(rng)};
    }

    const ParamPoint realCandidate = {
        denormalize(candidate.kp, kpRange),
        denormalize(candidate.ki, kiRange)};
    if (wasEvaluated(results, realCandidate)) {
      continue;
    }

    std::vector<double> k(n);
    for (size_t row = 0; row < n; ++row) {
      k[row] = rbfKernel(candidate, x[row], args.boLengthScale);
    }

    double mean = 0.0;
    for (size_t row = 0; row < n; ++row) {
      mean += k[row] * alpha[row];
    }

    const auto v = solveLinearSystem(kernel, k);
    double variance = 1.0;
    for (size_t row = 0; row < n; ++row) {
      variance -= k[row] * v[row];
    }
    variance = std::max(variance, 1.0e-12);
    const double sigma = std::sqrt(variance);

    const double improvement = bestNorm - mean - args.boXi;
    const double z = improvement / sigma;
    const double ei = improvement * normalCdf(z) + sigma * normalPdf(z);
    if (ei > bestEi) {
      bestEi = ei;
      bestPoint = candidate;
    }
  }

  return {denormalize(bestPoint.kp, kpRange), denormalize(bestPoint.ki, kiRange)};
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
                                        double outputLimit,
                                        double closedLoopDuty)
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

  std::vector<size_t> effectiveIndexes;
  effectiveIndexes.reserve(frames.size());
  for (size_t i = 0; i < vq.size(); ++i) {
    if (std::abs(vq[i]) >= closedLoopDuty) {
      effectiveIndexes.push_back(i);
    }
  }
  result.effectiveRatio =
      static_cast<double>(effectiveIndexes.size()) / static_cast<double>(frames.size());
  if (effectiveIndexes.size() < 5U) {
    result.score = 1.0e8 + (1.0 - result.effectiveRatio) * 1.0e6;
    result.riseTime = captureSeconds;
    result.saturation = 0.0;
    return result;
  }

  std::vector<double> effectiveIq;
  std::vector<double> effectiveVq;
  effectiveIq.reserve(effectiveIndexes.size());
  effectiveVq.reserve(effectiveIndexes.size());
  for (size_t index : effectiveIndexes) {
    effectiveIq.push_back(iq[index]);
    effectiveVq.push_back(vq[index]);
  }

  const double absTarget = std::max(std::abs(target), 1.0e-6);
  const size_t finalWindow = std::max<size_t>(5U, effectiveIq.size() / 5U);
  const size_t firstFinal = effectiveIq.size() - finalWindow;
  std::vector<double> finalSamples(effectiveIq.begin() + static_cast<std::ptrdiff_t>(firstFinal),
                                   effectiveIq.end());
  double finalMean = 0.0;
  for (double value : finalSamples) {
    finalMean += value;
  }
  finalMean /= static_cast<double>(finalSamples.size());
  double finalMae = 0.0;
  for (double value : finalSamples) {
    finalMae += std::abs(value - target);
  }
  finalMae /= static_cast<double>(finalSamples.size());

  const auto peakIt = (target >= 0.0) ? std::max_element(effectiveIq.begin(), effectiveIq.end())
                                      : std::min_element(effectiveIq.begin(), effectiveIq.end());
  const double peak = *peakIt;
  if (target >= 0.0) {
    result.overshoot = std::max(0.0, (peak - target) / absTarget);
  } else {
    result.overshoot = std::max(0.0, (target - peak) / absTarget);
  }
  const auto [finalMinIt, finalMaxIt] =
      std::minmax_element(finalSamples.begin(), finalSamples.end());
  result.settleError = finalMae / absTarget;
  result.ripple = (*finalMaxIt - *finalMinIt) / absTarget;

  const double threshold = target * 0.9;
  result.riseTime = captureSeconds;
  for (size_t i = 0; i < effectiveIq.size(); ++i) {
    if ((target >= 0.0 && effectiveIq[i] >= threshold) ||
        (target < 0.0 && effectiveIq[i] <= threshold)) {
      result.riseTime = static_cast<double>(effectiveIndexes[i]) * samplePeriod;
      break;
    }
  }

  size_t saturationCount = 0U;
  for (double value : effectiveVq) {
    if (std::abs(value) >= outputLimit * 0.98) {
      saturationCount++;
    }
  }
  result.saturation = static_cast<double>(saturationCount) / static_cast<double>(effectiveVq.size());
  result.score = result.overshoot * 4.0 +
                 result.settleError * 5.0 +
                 result.ripple * 3.0 +
                 result.saturation * 2.0 +
                 (1.0 - result.effectiveRatio) * 4.0 +
                 std::min(result.riseTime / std::max(captureSeconds, samplePeriod), 2.0);
  return result;
}

static ScoreResult scoreVelocityResponse(const std::vector<std::vector<float>> &frames,
                                         double target,
                                         double captureSeconds,
                                         double samplePeriod,
                                         double outputLimit,
                                         double closedLoopDuty)
{
  ScoreResult result;
  if (frames.size() < 10U) {
    result.riseTime = captureSeconds;
    return result;
  }

  std::vector<double> velocity;
  std::vector<double> targetIq;
  std::vector<double> phaseDuty;
  velocity.reserve(frames.size());
  targetIq.reserve(frames.size());
  phaseDuty.reserve(frames.size());
  for (const auto &frame : frames) {
    velocity.push_back(frame[1]);
    targetIq.push_back(frame[3]);
    phaseDuty.push_back(std::max(std::abs(frame[14]), std::abs(frame[15])));
  }

  std::vector<size_t> effectiveIndexes;
  effectiveIndexes.reserve(frames.size());
  for (size_t i = 0; i < phaseDuty.size(); ++i) {
    if (phaseDuty[i] >= closedLoopDuty) {
      effectiveIndexes.push_back(i);
    }
  }
  result.effectiveRatio =
      static_cast<double>(effectiveIndexes.size()) / static_cast<double>(frames.size());
  if (effectiveIndexes.size() < 5U) {
    result.score = 1.0e8 + (1.0 - result.effectiveRatio) * 1.0e6;
    result.riseTime = captureSeconds;
    result.saturation = 0.0;
    return result;
  }

  std::vector<double> effectiveVelocity;
  std::vector<double> effectiveTargetIq;
  effectiveVelocity.reserve(effectiveIndexes.size());
  effectiveTargetIq.reserve(effectiveIndexes.size());
  for (size_t index : effectiveIndexes) {
    effectiveVelocity.push_back(velocity[index]);
    effectiveTargetIq.push_back(targetIq[index]);
  }

  const double absTarget = std::max(std::abs(target), 1.0e-6);
  const size_t finalWindow = std::max<size_t>(5U, effectiveVelocity.size() / 5U);
  const size_t firstFinal = effectiveVelocity.size() - finalWindow;
  std::vector<double> finalSamples(
      effectiveVelocity.begin() + static_cast<std::ptrdiff_t>(firstFinal),
      effectiveVelocity.end());
  double finalMean = 0.0;
  for (double value : finalSamples) {
    finalMean += value;
  }
  finalMean /= static_cast<double>(finalSamples.size());

  const auto peakIt = (target >= 0.0) ?
                      std::max_element(effectiveVelocity.begin(), effectiveVelocity.end()) :
                      std::min_element(effectiveVelocity.begin(), effectiveVelocity.end());
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
  for (size_t i = 0; i < effectiveVelocity.size(); ++i) {
    if ((target >= 0.0 && effectiveVelocity[i] >= threshold) ||
        (target < 0.0 && effectiveVelocity[i] <= threshold)) {
      result.riseTime = static_cast<double>(effectiveIndexes[i]) * samplePeriod;
      break;
    }
  }

  size_t saturationCount = 0U;
  for (double value : effectiveTargetIq) {
    if (std::abs(value) >= outputLimit * 0.98) {
      saturationCount++;
    }
  }
  result.saturation =
      static_cast<double>(saturationCount) / static_cast<double>(effectiveTargetIq.size());
  result.score = result.overshoot * 0.5 +
                 result.settleError * 80.0 +
                 result.ripple * 8.0 +
                 result.saturation * 2.0 +
                 (1.0 - result.effectiveRatio) * 4.0 +
                 std::min(result.riseTime / std::max(captureSeconds, samplePeriod), 2.0) * 0.25;
  return result;
}

static std::string candidateName(const std::string &loop, double kp, double ki)
{
  std::ostringstream oss;
  oss << std::setprecision(6) << loop << "_kp_" << kp << "_ki_" << ki;
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

static void writeVelocityFramesCsv(const fs::path &path,
                                   const std::vector<std::vector<float>> &frames,
                                   double samplePeriod)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "t_s,target_speed,measured_speed,raw_speed,target_iq,iq,vq,velocity_i,vkp,vki,vmax,mech_pos,mech_angle,electrical_angle,vd,va,vb\n";
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
  out << "rank,kp,ki,score,overshoot,settle_error,ripple,rise_time_s,saturation,effective_ratio,sample_count,csv_path\n";
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
        << result.effectiveRatio << ','
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
  serial.writeLine("foc mode current");
  serial.drainText(std::chrono::milliseconds(50));
  serial.writeLine("foc limit " + std::to_string(args.limit));
  serial.writeLine("foc ckp " + std::to_string(kp));
  serial.writeLine("foc cki " + std::to_string(ki));
  serial.writeLine("foc id 0");
  serial.writeLine("foc iq 0");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("foc on");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("vofa current");
  serial.drainText(std::chrono::milliseconds(50));
  serial.writeLine("vofa on");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::this_thread::sleep_for(std::chrono::duration<double>(args.settle));
  (void)collectFrames(serial, parser, 0.2);

  serial.writeLine("foc iq " + std::to_string(args.targetIq), true);
  auto frames = collectFrames(serial, parser, args.capture);
  serial.writeLine("foc iq 0", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  serial.writeLine("vofa off", true);
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("foc off", true);
  serial.drainText(std::chrono::milliseconds(100));

  const fs::path csvPath = outputDir / (candidateName("current", kp, ki) + ".csv");
  writeFramesCsv(csvPath, frames, args.samplePeriod);

  const auto score = scoreCurrentResponse(frames,
                                          args.targetIq,
                                          args.capture,
                                          args.samplePeriod,
                                          args.limit,
                                          args.closedLoopDuty);
  CandidateResult result;
  result.kp = kp;
  result.ki = ki;
  result.score = score.score;
  result.overshoot = score.overshoot;
  result.settleError = score.settleError;
  result.ripple = score.ripple;
  result.riseTime = score.riseTime;
  result.saturation = score.saturation;
  result.effectiveRatio = score.effectiveRatio;
  result.sampleCount = frames.size();
  result.csvPath = csvPath;
  return result;
}

static CandidateResult runVelocityCandidate(SerialPort &serial,
                                            double kp,
                                            double ki,
                                            const Args &args,
                                            const fs::path &outputDir)
{
  JustFloatParser parser(16U);

  serial.writeLine("vofa off");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("foc off");
  serial.drainText(std::chrono::milliseconds(100));
  serial.writeLine("foc prep velocity");
  serial.drainText(std::chrono::milliseconds(200));
  serial.writeLine("foc mode current");
  serial.drainText(std::chrono::milliseconds(50));
  serial.writeLine("foc limit " + std::to_string(args.limit));
  serial.writeLine("foc ckp " + std::to_string(args.currentKp));
  serial.writeLine("foc cki " + std::to_string(args.currentKi));
  serial.writeLine("foc vmax " + std::to_string(args.limit));
  serial.writeLine("foc vkp " + std::to_string(kp));
  serial.writeLine("foc vki " + std::to_string(ki));
  serial.writeLine("foc vel 0");
  serial.writeLine("foc vel on");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("foc on");
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("vofa velocity");
  serial.drainText(std::chrono::milliseconds(50));
  serial.writeLine("vofa on");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::this_thread::sleep_for(std::chrono::duration<double>(args.settle));
  (void)collectFrames(serial, parser, 0.2);

  serial.writeLine("foc vel " + std::to_string(args.targetVelocity), true);
  auto frames = collectFrames(serial, parser, args.capture);
  serial.writeLine("foc vel 0", true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  serial.writeLine("vofa off", true);
  serial.drainText(std::chrono::milliseconds(150));
  serial.writeLine("foc off", true);
  serial.drainText(std::chrono::milliseconds(100));

  const fs::path csvPath = outputDir / (candidateName("velocity", kp, ki) + ".csv");
  writeVelocityFramesCsv(csvPath, frames, args.samplePeriod);

  const auto score = scoreVelocityResponse(frames,
                                           args.targetVelocity,
                                           args.capture,
                                           args.samplePeriod,
                                           args.limit,
                                           args.closedLoopDuty);
  CandidateResult result;
  result.kp = kp;
  result.ki = ki;
  result.score = score.score;
  result.overshoot = score.overshoot;
  result.settleError = score.settleError;
  result.ripple = score.ripple;
  result.riseTime = score.riseTime;
  result.saturation = score.saturation;
  result.effectiveRatio = score.effectiveRatio;
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
  const Range kpRange = parseRange(args.kp);
  const Range kiRange = parseRange(args.ki);
  const fs::path outputDir = args.output / ("current_" + timestamp());
  fs::create_directories(outputDir);

  std::cout << "Opening " << args.port << " at " << args.baud << " baud\n";
  SerialPort serial(args.port, args.baud);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  (void)serial.drainText(std::chrono::milliseconds(200));

  std::vector<CandidateResult> results;
  size_t index = 0U;

  try {
    if (args.optimizer == "grid") {
      const size_t total = kpValues.size() * kiValues.size();
      for (double kp : kpValues) {
        for (double ki : kiValues) {
          index++;
          std::cout << "\n[" << index << "/" << total
                    << "] grid current loop kp=" << kp
                    << ", ki=" << ki << "\n";
          auto result = runCurrentCandidate(serial, kp, ki, args, outputDir);
          results.push_back(result);
          std::cout << "score=" << result.score
                    << " overshoot=" << result.overshoot
                    << " settle=" << result.settleError
                    << " ripple=" << result.ripple
                    << " rise=" << result.riseTime << "s"
                    << " sat=" << result.saturation
                    << " eff=" << result.effectiveRatio
                    << " samples=" << result.sampleCount << "\n";
        }
      }
    } else if (args.optimizer == "bo") {
      std::mt19937 rng(args.seed);
      const auto init = initialDesign(kpRange,
                                      kiRange,
                                      args.boInitial,
                                      {args.initialKp, args.initialKi});
      const size_t total = init.size() + args.boIterations;

      for (const auto &point : init) {
        index++;
        std::cout << "\n[" << index << "/" << total
                  << "] bo-init current loop kp=" << point.kp
                  << ", ki=" << point.ki << "\n";
        auto result = runCurrentCandidate(serial, point.kp, point.ki, args, outputDir);
        results.push_back(result);
        std::cout << "score=" << result.score
                  << " overshoot=" << result.overshoot
                  << " settle=" << result.settleError
                  << " ripple=" << result.ripple
                  << " rise=" << result.riseTime << "s"
                  << " sat=" << result.saturation
                  << " eff=" << result.effectiveRatio
                  << " samples=" << result.sampleCount << "\n";
      }

      for (size_t iter = 0; iter < args.boIterations; ++iter) {
        const ParamPoint point = proposeBayesianCandidate(results, kpRange, kiRange, args, rng);
        index++;
        std::cout << "\n[" << index << "/" << total
                  << "] bo-ei current loop kp=" << point.kp
                  << ", ki=" << point.ki << "\n";
        auto result = runCurrentCandidate(serial, point.kp, point.ki, args, outputDir);
        results.push_back(result);
        std::cout << "score=" << result.score
                  << " overshoot=" << result.overshoot
                  << " settle=" << result.settleError
                  << " ripple=" << result.ripple
                  << " rise=" << result.riseTime << "s"
                  << " sat=" << result.saturation
                  << " eff=" << result.effectiveRatio
                  << " samples=" << result.sampleCount << "\n";
      }
    }
  } catch (...) {
    try {
      serial.writeLine("foc iq 0", true);
      serial.writeLine("vofa off", true);
      serial.writeLine("foc off", true);
    } catch (...) {
    }
    throw;
  }

  serial.writeLine("foc iq 0", true);
  serial.writeLine("vofa off", true);
  serial.writeLine("foc off", true);

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

static int runVelocitySearch(const Args &args)
{
  const auto kpValues = parseFloatList(args.kp);
  const auto kiValues = parseFloatList(args.ki);
  const Range kpRange = parseRange(args.kp);
  const Range kiRange = parseRange(args.ki);
  const fs::path outputDir = args.output / ("velocity_" + timestamp());
  fs::create_directories(outputDir);

  std::cout << "Opening " << args.port << " at " << args.baud << " baud\n";
  SerialPort serial(args.port, args.baud);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  (void)serial.drainText(std::chrono::milliseconds(200));

  std::vector<CandidateResult> results;
  size_t index = 0U;

  try {
    if (args.optimizer == "grid") {
      const size_t total = kpValues.size() * kiValues.size();
      for (double kp : kpValues) {
        for (double ki : kiValues) {
          index++;
          std::cout << "\n[" << index << "/" << total
                    << "] grid velocity loop vkp=" << kp
                    << ", vki=" << ki << "\n";
          auto result = runVelocityCandidate(serial, kp, ki, args, outputDir);
          results.push_back(result);
          std::cout << "score=" << result.score
                    << " overshoot=" << result.overshoot
                    << " settle=" << result.settleError
                    << " ripple=" << result.ripple
                    << " rise=" << result.riseTime << "s"
                    << " sat=" << result.saturation
                    << " eff=" << result.effectiveRatio
                    << " samples=" << result.sampleCount << "\n";
        }
      }
    } else if (args.optimizer == "bo") {
      std::mt19937 rng(args.seed);
      const auto init = initialDesign(kpRange,
                                      kiRange,
                                      args.boInitial,
                                      {args.initialKp, args.initialKi});
      const size_t total = init.size() + args.boIterations;

      for (const auto &point : init) {
        index++;
        std::cout << "\n[" << index << "/" << total
                  << "] bo-init velocity loop vkp=" << point.kp
                  << ", vki=" << point.ki << "\n";
        auto result = runVelocityCandidate(serial, point.kp, point.ki, args, outputDir);
        results.push_back(result);
        std::cout << "score=" << result.score
                  << " overshoot=" << result.overshoot
                  << " settle=" << result.settleError
                  << " ripple=" << result.ripple
                  << " rise=" << result.riseTime << "s"
                  << " sat=" << result.saturation
                  << " eff=" << result.effectiveRatio
                  << " samples=" << result.sampleCount << "\n";
      }

      for (size_t iter = 0; iter < args.boIterations; ++iter) {
        const ParamPoint point = proposeBayesianCandidate(results, kpRange, kiRange, args, rng);
        index++;
        std::cout << "\n[" << index << "/" << total
                  << "] bo-ei velocity loop vkp=" << point.kp
                  << ", vki=" << point.ki << "\n";
        auto result = runVelocityCandidate(serial, point.kp, point.ki, args, outputDir);
        results.push_back(result);
        std::cout << "score=" << result.score
                  << " overshoot=" << result.overshoot
                  << " settle=" << result.settleError
                  << " ripple=" << result.ripple
                  << " rise=" << result.riseTime << "s"
                  << " sat=" << result.saturation
                  << " eff=" << result.effectiveRatio
                  << " samples=" << result.sampleCount << "\n";
      }
    }
  } catch (...) {
    try {
      serial.writeLine("foc vel 0", true);
      serial.writeLine("vofa off", true);
      serial.writeLine("foc off", true);
    } catch (...) {
    }
    throw;
  }

  serial.writeLine("foc vel 0", true);
  serial.writeLine("vofa off", true);
  serial.writeLine("foc off", true);

  if (results.empty()) {
    throw std::runtime_error("no candidate result");
  }

  const fs::path summaryPath = outputDir / "summary.csv";
  writeSummaryCsv(summaryPath, results);
  const auto best = std::min_element(results.begin(), results.end(), [](const auto &a, const auto &b) {
    return a.score < b.score;
  });

  std::cout << "\nBest velocity PI:\n"
            << "  vkp=" << best->kp << "\n"
            << "  vki=" << best->ki << "\n"
            << "  score=" << best->score << "\n"
            << "  summary=" << summaryPath.string() << "\n";
  return 0;
}

int main(int argc, char **argv)
{
  try {
    const auto args = parseArgs(argc, argv);
    if (args.loop == "velocity") {
      return runVelocitySearch(args);
    }
    return runCurrentSearch(args);
  } catch (const std::exception &ex) {
    std::cerr << "error: " << ex.what() << "\n";
    return 1;
  }
}
