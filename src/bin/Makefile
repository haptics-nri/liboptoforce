SOURCES=$(wildcard *.cpp)
EXES=$(SOURCES:.cpp=)
all: $(EXES)

%: %.cpp
	g++ -o $@ $^ -std=c++11 -lboost_program_options -lboost_system -lboost_chrono -loptoforce
