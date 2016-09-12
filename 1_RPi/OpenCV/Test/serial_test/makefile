COMP= g++
FFLAGS= -Wall
SRC= serial.cpp
OBJ=${SRC:.cpp=.o}
#DEPS= i2c_function_decs.h

%.o : %.cpp ${DEPS}
	${COMP} ${FFLAGS} -o $@ -c $<

prog: ${OBJ}
	${COMP} ${FFLAGS} -o $@ ${OBJ}

clean:
	rm *.o
