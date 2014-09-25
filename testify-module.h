extern "C" {
  void build();
};

#define TESTIFY_VERSION(x) extern "C" { const char *TESTIFY_MODULE_VERSION = x; }
