# Makefile for Basler pylon sample program
.PHONY: all clean

# The program to build
NAME       := Grab_UsingActionCommand

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5

# Build tools and flags
LD         := $(CXX)
CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) -DUSE_GIGE 
CXXFLAGS   := #e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath) -std=c++11 -pthread
LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs)
LDOPENCV   := $(shell pkg-config --libs opencv)

# Rules for building
all: $(NAME)

$(NAME): $(NAME).o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS) $(LDOPENCV)

$(NAME).o: $(NAME).cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(NAME).o $(NAME)

	
	