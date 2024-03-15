#include <cstdio>
#include "convert.cpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world my_pcl_pkg package\n");
  cloud_callback(nullptr);
  return 0;
}
