#pragma once

namespace uORBTest {

class UnitTest {
public:
  static UnitTest &instance();

  int test();

  UnitTest(const UnitTest &) = delete;

private:
  UnitTest() = default;
};

} // namespace uORBTest
