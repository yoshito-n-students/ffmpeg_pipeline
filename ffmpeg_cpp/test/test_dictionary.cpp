#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace {

std::string normalize_yaml(const std::string &yaml) {
  const YAML::Node node = YAML::Load(yaml);
  YAML::Emitter emitter;
  emitter << node;
  return emitter.c_str();
}

} // namespace

// Ensures null() produces empty YAML representations.
TEST(DictionaryTest, NullDictionaryProducesEmptyYaml) {
  const auto dict = ffmpeg_cpp::Dictionary::null();
  EXPECT_FALSE(dict);

  const YAML::Node node = YAML::Load(dict.to_yaml());
  EXPECT_TRUE(node.IsNull());

  const YAML::Node flow_node = YAML::Load(dict.to_flow_style_yaml());
  EXPECT_TRUE(flow_node.IsNull());
}

// Ensures a dictionary created from YAML round-trips back to the same data.
TEST(DictionaryTest, CreateFromYamlRoundTrip) {
  const std::string yaml = R"(foo: bar
answer: "42"
)";

  const auto dict = ffmpeg_cpp::Dictionary::create(yaml);
  ASSERT_TRUE(dict);

  const YAML::Node node = YAML::Load(dict.to_yaml());
  ASSERT_TRUE(node.IsMap());
  EXPECT_EQ("bar", node["foo"].as<std::string>());
  EXPECT_EQ("42", node["answer"].as<std::string>());

  const std::string flow_yaml = dict.to_flow_style_yaml();
  EXPECT_FALSE(flow_yaml.empty());
  EXPECT_EQ('{', flow_yaml.front());

  const YAML::Node flow_node = YAML::Load(flow_yaml);
  EXPECT_EQ("bar", flow_node["foo"].as<std::string>());
  EXPECT_EQ("42", flow_node["answer"].as<std::string>());
}

// Ensures the copy constructor clones data independently from the source.
TEST(DictionaryTest, CopyConstructorProducesIndependentClone) {
  auto original = ffmpeg_cpp::Dictionary::create(R"(foo: bar
)");
  ASSERT_TRUE(original);

  const ffmpeg_cpp::Dictionary copy(original);
  ASSERT_TRUE(copy);
  EXPECT_EQ(normalize_yaml(original.to_yaml()), normalize_yaml(copy.to_yaml()));

  original = ffmpeg_cpp::Dictionary::null();
  EXPECT_FALSE(original);
  EXPECT_EQ("bar", YAML::Load(copy.to_yaml())["foo"].as<std::string>());
}

// Ensures the move constructor transfers ownership and clears the source wrapper.
TEST(DictionaryTest, MoveConstructorTransfersOwnership) {
  auto original = ffmpeg_cpp::Dictionary::create(R"(foo: bar
)");
  ASSERT_TRUE(original);

  const auto moved = ffmpeg_cpp::Dictionary(std::move(original));
  EXPECT_TRUE(moved);
  EXPECT_FALSE(original);
  EXPECT_EQ("bar", YAML::Load(moved.to_yaml())["foo"].as<std::string>());

  const YAML::Node moved_from_yaml = YAML::Load(original.to_yaml());
  EXPECT_EQ(0u, moved_from_yaml.size());
}

// Ensures move assignment transfers content, clears the source, and survives self-move.
TEST(DictionaryTest, MoveAssignmentTransfersAndClearsSource) {
  auto source = ffmpeg_cpp::Dictionary::create(R"(foo: bar
)");
  auto destination = ffmpeg_cpp::Dictionary::create(R"(baz: qux
)");

  destination = std::move(source);
  EXPECT_TRUE(destination);
  EXPECT_FALSE(source);
  EXPECT_EQ("bar", YAML::Load(destination.to_yaml())["foo"].as<std::string>());
  EXPECT_EQ(0u, YAML::Load(source.to_yaml()).size());

  auto self_move = ffmpeg_cpp::Dictionary::create(R"(alpha: beta
)");
  const std::string expected = normalize_yaml(self_move.to_yaml());
  self_move = std::move(self_move);
  EXPECT_TRUE(self_move);
  EXPECT_EQ(expected, normalize_yaml(self_move.to_yaml()));
}

// Ensures invalid YAML input triggers an ffmpeg_cpp::Error.
TEST(DictionaryTest, CreateFromInvalidYamlThrowsError) {
  const std::string yaml = R"(foo:
  - bar
)";
  EXPECT_THROW(ffmpeg_cpp::Dictionary::create(yaml), ffmpeg_cpp::Error);
}
