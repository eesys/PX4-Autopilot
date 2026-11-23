#include "wqueue_test.h"

#include <cstdio>

int main(int argc, char *argv[]) {
  const int rc = wqueue_test_main(argc, argv);

  if (rc == 0) {
    std::puts("PASS");
    return 0;
  }

  std::puts("FAIL");
  return -1;
}
