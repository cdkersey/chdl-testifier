LDLIBS ?= -lchdl -ldl
CXXFLAGS ?= -std=c++11

TESTLIBS = sample.so harmonica2.so score.so life.so lavabug.so lfsr.so

all: testify $(TESTLIBS)

testify: testify.cpp stopwatch.cpp stopwatch.h

run: testify $(TESTLIBS)
	./run_tests.sh >> RESULT_DB

clean:
	rm -f $(TESTLIBS) testify dump.vcd

%.so : %.cpp testify-module.h
	$(CXX) -o $@ -shared -fPIC $(CXXFLAGS) $(LDFLAGS) $< $(LDLIBS)
