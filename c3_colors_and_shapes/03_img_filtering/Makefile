CXX=g++
CXXFLAGS=`pkg-config --cflags opencv`
LDLIBS=`pkg-config --libs opencv`

SRCS:=Morphology_1.cpp Morphology_2.cpp Smoothing.cpp
TARGETS:=$(patsubst %.cpp,%,$(SRCS))

all: $(TARGETS)

%: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

clean:
	rm -rf $(TARGETS)
	
.PHONY: all clean
