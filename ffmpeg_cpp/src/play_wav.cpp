#include <chrono>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

using namespace std::chrono_literals;
namespace av = ffmpeg_cpp;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input.wav>" << std::endl;
    return 1;
  }
  const std::string input_filename = argv[1];

  try {
    // Open the input file
    av::Input input = av::Input::create(argv[1], "", av::Dictionary::null(), "audio");
    std::cout << "Input device " << input_filename
              << " opened (codec: " << input.codec_parameters().codec_name() << ")" << std::endl;

    // Open the output device for the input codec parameters
    av::Output output("pulse", "default", input.codec_parameters(), {});
    std::cout << "Output device opened" << std::endl;

    // Read packets from the input file and play them on the output device
    while (true) {
      av::Packet packet = av::Packet::null();
      while (true) {
        packet = input.read_frame();
        if (!packet.empty()) {
          break;
        }
        std::cerr << "Failed to read frame for temporary reason. Retrying..." << std::endl;
        std::this_thread::sleep_for(10ms);
      }

      while (!output.write_frame(packet)) {
        std::cerr << "Failed to write frame for temporary reason. Retrying..." << std::endl;
        std::this_thread::sleep_for(10ms);
      }
    }
  } catch (const std::runtime_error &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }

  return 0;
}