COMP= g++
FFLAGS= -Wall
SRC= serial.cpp
SRC2= read_serial.cpp
OBJ=${SRC:.cpp=.o}
OBJ2=${SRC2:.cpp=.o}
#DEPS= i2c_function_decs.h

%.o : %.cpp ${DEPS}
	${COMP} ${FFLAGS} -o $@ -c $<

prog: ${OBJ} gorp
	${COMP} ${FFLAGS} -o $@ ${OBJ}

gorp: ${OBJ2}
	${COMP} ${FFLAGS} -o $@ ${OBJ2}

clean:
	rm *.o
