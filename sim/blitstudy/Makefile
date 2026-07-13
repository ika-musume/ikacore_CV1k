CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra

all: trace_dump blit_study fifo_study

trace_dump: trace_dump.cpp blit_trace.h
	$(CXX) $(CXXFLAGS) -o $@ trace_dump.cpp

blit_study: study_main.cpp blit_trace.h cost_model.h ddr3_stat.h engine.h workload.h
	$(CXX) $(CXXFLAGS) -o $@ study_main.cpp

fifo_study: fifo_study.cpp blit_trace.h cost_model.h ddr3_stat.h engine.h workload.h
	$(CXX) $(CXXFLAGS) -o $@ fifo_study.cpp

clean:
	rm -f trace_dump blit_study fifo_study

.PHONY: all clean
