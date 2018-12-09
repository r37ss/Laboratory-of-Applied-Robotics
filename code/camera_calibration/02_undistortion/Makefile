TARGET=undistort
CXX=g++
CXXFLAGS=`pkg-config --cflags opencv`
LDLIBS=`pkg-config --libs opencv`

SRCS:=$(wildcard *.cpp)
OBJS:=$(patsubst %.cpp,%.o,$(SRCS))

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -rf $(TARGET) *.o
	
.PHONY: all clean
