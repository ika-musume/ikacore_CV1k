CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

all: blitgold

blitgold: gold_main.cpp golden.h vram.h png.h ../blitstudy/blit_trace.h
	$(CXX) $(CXXFLAGS) -o $@ gold_main.cpp

test: blitgold
	./blitgold --selftest

clean:
	rm -f blitgold

.PHONY: all test clean
