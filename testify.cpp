#include <iostream>

#include <chdl/chdl.h>
#include <cstdlib>
#include <ctime>

#include <dlfcn.h>
#include <unistd.h>

#include "stopwatch.h"

using namespace std;
using namespace chdl;

string build(const char* file) {
  void *handle = dlopen(file, RTLD_NOW);
  if (!handle) {
    cerr << "Error: Could not dlopen() \"" << file << '\"' << endl;
    abort();
  }
  const char *version(*(const char**)dlsym(handle, "TESTIFY_MODULE_VERSION"));
  void (*build)() = (void (*)())dlsym(handle, "build");

  if (!version) {
    version = "[unspecified]";
    cerr << "Warning: No version number in \"" << file << '\"' << endl;
  }

  if (!build) {
    cerr << "Error: No build() function in \"" << file << '\"' << endl;
    abort();
  }

  build();

  return string(version);
}

int main(int argc, char **argv) {
  char hostname[80];
  gethostname(hostname, 80);

  time_t t(time(NULL));
  string version, filename, date(ctime(&t));

  date.pop_back();

  if (argc != 2) {
    cerr << "Usage:" << endl << "  " << argv[0] << " <design library>" << endl;
    return 1;
  }

  filename = argv[1];

  stopwatch_start();

  version = build(filename.c_str());

  double ms_build(stopwatch_stop());
  stopwatch_start();

  optimize();

  double ms_opt(stopwatch_stop());

  unsigned cp(critpath());
  nodeid_t n(nodes.size());

  stopwatch_start();

  ofstream vcd("dump.vcd");
  run(vcd, 1000);  

  double ms_run(stopwatch_stop());

  cout << date << ", " << t << ", " << hostname << ", "
       << filename << ", " << version << ", " << n << ", " 
       << cp << ", " << ms_build << ", " << ms_opt << ", " << ms_run << endl;

  return 0;
}
