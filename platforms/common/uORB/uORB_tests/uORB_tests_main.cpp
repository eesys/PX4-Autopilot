#include "uORBTest_UnitTest.hpp"

#include <cstdio>
#include <cstring>

static void usage() { std::puts("Usage: uorb_tests [latency_test]"); }

int uorb_tests_main(int argc, char *argv[]) {
  if (argc == 1) {
    uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
    const int rc = t.test();

    if (rc == 0) {
      std::puts("PASS");
      return 0;
    }

    std::puts("FAIL");
    return -1;
  }

  if ((argc > 1) && (std::strcmp(argv[1], "latency_test") == 0)) {
    usage();
    return -1;
  }

  usage();
  return -1;
}

// Host entry until this file is built as a PX4 module. main calls
// uorb_tests_main.
int main(int argc, char *argv[]) { return uorb_tests_main(argc, argv); }
