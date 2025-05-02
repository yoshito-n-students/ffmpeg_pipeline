#include <iostream>
#include <sstream>
#include <stdexcept>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

Dictionary::Dictionary() : std::unique_ptr<AVDictionary, Deleter<AVDictionary>>(nullptr) {};

Dictionary::Dictionary(const std::string &yaml) : Dictionary() {
  if (!YAML::convert<Dictionary>::decode(YAML::Load(yaml), *this)) {
    throw Error("Dictionary::Dictionary(): Failed to parse dictionary from yaml");
  }
}

Dictionary::Dictionary(const Dictionary &other) : Dictionary() {
  AVDictionary *dict = nullptr;
  const int ret = av_dict_copy(&dict, other.get(), 0);
  if (ret < 0) {
    av_dict_free(&dict);
    throw Error("Dictionary::Dictionary(): Failed to copy dictionary", ret);
  }
  reset(dict);
}

std::string Dictionary::to_yaml() const {
  YAML::Emitter emitter;
  emitter << YAML::convert<Dictionary>::encode(*this);
  return emitter.c_str();
}

std::string Dictionary::to_flow_style_yaml() const {
  YAML::Emitter emitter;
  emitter << YAML::Flow << YAML::convert<Dictionary>::encode(*this);
  return emitter.c_str();
}

} // namespace ffmpeg_cpp

namespace YAML {

bool convert<ffmpeg_cpp::Dictionary>::decode(const Node &yaml, ffmpeg_cpp::Dictionary &dict) {
  try {
    AVDictionary *dict_ptr = nullptr;
    for (const auto &entry : yaml) {
      const std::string key = entry.first.as<std::string>(), value = entry.second.as<std::string>();
      if (const int ret = av_dict_set(&dict_ptr, key.c_str(), value.c_str(), 0); ret < 0) {
        av_dict_free(&dict_ptr);
        throw std::runtime_error("Failed to set dictionary entry [" + key + ", " + value + "]");
      }
    }
    dict.reset(dict_ptr);
    return true;
  } catch (const std::runtime_error &error) {
    std::cerr << error.what() << std::endl;
    return false;
  }
}

Node convert<ffmpeg_cpp::Dictionary>::encode(const ffmpeg_cpp::Dictionary &dict) {
  const AVDictionaryEntry *entry = nullptr;
  Node yaml;
  while (true) {
    entry = av_dict_iterate(dict.get(), entry);
    if (!entry) {
      break;
    }
    // yaml-cpp does not accept char * as key, so we need to cast it to const char *
    yaml[static_cast<const char *>(entry->key)] = static_cast<const char *>(entry->value);
  }
  return yaml;
}

} // namespace YAML