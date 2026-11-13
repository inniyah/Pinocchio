#!/usr/bin/make -f
# Regression tests. Linked statically so they run without LD_LIBRARY_PATH.

CXX= g++
CFLAGS= -O2 -g -Wall -std=c++17
LIBS= ../Pinocchio/libpinocchio.a -lm

all: test_fixes

test_fixes: test_fixes.cpp ../Pinocchio/libpinocchio.a
	$(CXX) $(CFLAGS) -o $@ test_fixes.cpp $(LIBS)

check: test_fixes
	./test_fixes

clean distclean cleanall:
	rm -f test_fixes *.o *~

.PHONY: all check clean distclean cleanall
