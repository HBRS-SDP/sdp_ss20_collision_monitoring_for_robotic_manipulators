# Makefile for the SDP Assignment

# -----------------------------------------------------
# Variables to control Makefile operation
# EXEC = The final executable name
# OBJ = The object file names seperated by spaces, i.e. filename.o

EXEC = sdpAssignment.exe
OBJ = main.o model_interpreter.o
CXX = g++
CXXFLAGS = -Wall -g

# -----------------------------------------------------
# OS detection for multi-platform clean

ifeq ($(OS),Windows_NT)
		RM = del -f
else
		RM = rm -rf
endif

# -----------------------------------------------------
# Target files

# main build command
main: $(OBJ)
	$(CXX) $(OBJ) -o $(EXEC)

# Unit test files
test: $(OBJ) test.o
	$(CXX) $(OBJ) test.o -o unit_test

# Individual object files
main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp

test.o: test.cpp model_interpreter.cpp model_interpreter.h
	$(CXX) $(CXXFLAGS) -c test.cpp

model_interpreter.o: model_interpreter.cpp model_interpreter.h
	$(CXX) $(CXXFLAGS) -c model_interpreter.cpp

clean:
	$(RM) *.o
	$(RM) *.exe
