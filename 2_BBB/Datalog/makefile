COMP= g++
FFLAGS= -Wall
SRC= datalog.cpp
OBJ=${SRC:.cpp=.o}

%.o : %.cpp ${DEPS}
	${COMP} ${FFLAGS} -o $@ -c $<

datalog: ${OBJ}
	${COMP} ${FFLAGS} -o $@ ${OBJ}

clean:
	rm *.o
