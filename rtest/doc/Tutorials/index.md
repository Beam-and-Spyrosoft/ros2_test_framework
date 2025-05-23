# Tutorials

These step-by-step tutorials will help you get started with RTEST. They cover various testing scenarios and are designed to be followed in sequence.

## Tutorial Sequence

```{toctree}
---
maxdepth: 2
titlesonly: true
---
Subscriber-Test
Publisher-Test
Timer-Test
Clock-Test
Service-Test
```

## Quick Start Example

Here's a simple example of testing a ROS 2 publisher node:

```cpp
#include "my_package/publisher_node.hpp"

TEST(PublisherNodeTest, PublishesExpectedMessage)
{
  auto node = std::make_shared<PublisherNode>();
  
  /// Retrieve the publisher created by the Node
  auto publisher = rtest::findPublisher<std_msgs::msg::String>(node, "/test_topic");
  
  // Check that the Node actually created the Publisher with topic: "/test_topic"
  ASSERT_TRUE(publisher);
  
  /// Set up expectation that the Node will publish a message when triggered
  auto expectedMsg = std_msgs::msg::String();
  expectedMsg.set__data("test_msg");
  EXPECT_CALL(*publisher, publish(expectedMsg)).Times(1);
  
  // Trigger the node
  node->publishMsg();
}
```

## Examples Repository

All example code can be found in the [C++ minimal examples](https://github.com/Beam-and-Spyrosoft/rtest/tree/main/examples/test) directory.

For a more comprehensive example, see the [publisher/subscriber tests](https://github.com/Beam-and-Spyrosoft/rtest/blob/main/examples/test/pub_sub_tests.cpp).
