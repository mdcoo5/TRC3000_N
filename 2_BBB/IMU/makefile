COMP= g++
FFLAGS= -Wall
SRC= imu_i2c.cpp
OBJ=${SRC:.cpp=.o}
DEPS= i2c_function_decs.h

%.o : %.cpp ${DEPS}
	${COMP} ${FFLAGS} -o $@ -c $<

prog: ${OBJ}
	${COMP} ${FFLAGS} -o $@ ${OBJ}

clean:
	rm *.o
