CXX=g++
CXXFLAGS=`pkg-config --cflags opencv`
LDLIBS=`pkg-config --libs opencv`

SRCS:=color_space_hsv.cpp color_space_rgb.cpp
TARGETS:=$(patsubst %.cpp,%,$(SRCS))

all: $(TARGETS)

%: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS)

clean:
	rm -rf $(TARGETS)
	
.PHONY: all clean
