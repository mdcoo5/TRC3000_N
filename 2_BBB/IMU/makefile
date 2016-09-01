COMP= g++
FFLAGS= -Wall
SRC= imu_i2c.cpp
OBJ=${SRC:.cpp=.o}

%.o : %.cpp
	${COMP} ${FFLAGS} -o $@ -c $<

prog: ${OBJ}
	${COMP} ${FFLAGS} -o $@ ${OBJ}

clean:
	rm *.o
