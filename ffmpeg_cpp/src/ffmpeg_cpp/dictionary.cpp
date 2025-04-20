#include <iostream>
#include <stdexcept>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

Dictionary::Dictionary(const std::string &yaml) : dict_(nullptr, &free_dict) {
  if (!YAML::convert<Dictionary>::decode(YAML::Load(yaml), *this)) {
    throw Error("Dictionary::Dictionary(): Failed to parse dictionary from yaml");
  }
}

Dictionary::Dictionary(const std::map<std::string, std::string> &map) : dict_(nullptr, &free_dict) {
  AVDictionary *dict = nullptr;
  for (const auto &[key, value] : map) {
    if (const int ret = av_dict_set(&dict, key.c_str(), value.c_str(), 0); ret < 0) {
      av_dict_free(&dict);
      throw Error("Dictionary::Dictionary(): Failed to set dictionary entry [" + key + ", " +
                      value + "]",
                  ret);
    }
  }
  dict_.reset(dict);
}

Dictionary::Dictionary(const Dictionary &other) : dict_(nullptr, &free_dict) {
  AVDictionary *dict = nullptr;
  const int ret = av_dict_copy(&dict, other.get(), 0);
  if (ret < 0) {
    av_dict_free(&dict);
    throw Error("Dictionary::Dictionary(): Failed to copy dictionary", ret);
  }
  dict_.reset(dict);
}

std::map<std::string, std::string> Dictionary::to_map() const {
  const AVDictionaryEntry *entry = nullptr;
  std::map<std::string, std::string> map;
  while(true){
    entry = av_dict_iterate(get(), entry);
    if (!entry) {
      break;
    }
    map.insert({entry->key, entry->value});
  }
  return map;
}

void Dictionary::free_dict(AVDictionary *dict) { av_dict_free(&dict); }

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
    dict = ffmpeg_cpp::Dictionary(dict_ptr);
    return true;
  } catch (const std::runtime_error &error) {
    std::cerr << error.what() << std::endl;
    return false;
  }
}

} // namespace YAML