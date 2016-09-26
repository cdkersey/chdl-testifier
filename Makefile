LDLIBS ?= -lchdl -ldl
CXXFLAGS ?= -std=c++11 -O3

TESTLIBS = sample.so harmonica2.so score.so life.so lavabug.so lfsr.so \
           lavabug-fp.so fft.so

all: testify $(TESTLIBS)

testify: testify.cpp stopwatch.cpp stopwatch.h
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) testify.cpp stopwatch.cpp $(LDLIBS)

run: testify $(TESTLIBS)
	./run_tests.sh >> RESULT_DB

clean:
	rm -f $(TESTLIBS) testify dump.vcd

%.so : %.cpp testify-module.h
	$(CXX) -o $@ -shared -fPIC $(CXXFLAGS) $(LDFLAGS) $< $(LDLIBS)
