# RTEST Documentation

```{toctree}
---
maxdepth: 2
titlesonly: true
hidden: true
---
Concepts/index
Installation/index
Tutorials/index
Contributing
cpp_api_docs
```

Welcome to the documentation for RTEST - a ROS 2 Unit Testing Framework!

**RTEST** is a framework for writing and running tests in ROS 2. It provides a set of tools and libraries to help developers create, manage, and execute tests for their ROS 2 packages.

## Key Benefits

- **Reliable Testing**: Eliminates "flaky tests" by providing deterministic test execution
- **Single-Threaded Control**: Precise control over test execution flow
- **Simulated Time**: Direct timer control for time-dependent implementation testing
- **Mock Entities**: Verify publishers, subscribers, timers, services, and clients

## Getting started

[Installation](Installation/index.md)
  - Integrating RTEST with your ROS 2 package tests

[Tutorials](Tutorials/index.md)
  - The best place to start for new users!
  - Hands-on sample unit tests that help you write your own tests

[Concepts](Concepts/index.md)
  - Detailed explanation of underlying concepts and mechanisms

[C++ API](cpp_api_docs.md)
  - Detailed API documentation
