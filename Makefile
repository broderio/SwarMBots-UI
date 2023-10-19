CC=g++ -g -Wall -std=c++17
CXX         = g++

debug: CXXFLAGS += -g3 -DDEBUG
debug:
	$(CXX) $(CXXFLAGS) drive.c -o drive_debug


clean:
	rm -f drive
drive: 
	${CC} $@.c -o $@ $^
 