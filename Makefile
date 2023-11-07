CC=g++ -g -Wall -std=c++17
CXX         = g++

SOURCES= mbot.cpp main.cpp
OBJS=${SOURCES:.cpp=.o}

debug: CXXFLAGS += -g3 -DDEBUG
debug:
	$(CXX) $(CXXFLAGS) drive.c -o drive_debug

main: ${OBJS}
	${CC} -o $@.cpp $^ -ldl -pthread

%.o: %.cpp
	${CC} -c $<
%.o: %.cc
	${CC} -c $<

clean:
	rm -f drive *.o
drive: 
	${CC} $@.c -o $@ $^
 