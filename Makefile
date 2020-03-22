# Makefile for robot manipulator
 
all: bin/run bin/test

bin/test: build/test.o build/manipulator.o
	g++ build/test.o build/manipulator.o -o bin/test

 bin/run: build/main.o build/manipulator.o
	g++ build/main.o build/manipulator.o -o bin/run

build/test.o: test/tests.cpp
	g++ -g -c test/tests.cpp -o build/test.o

build/main.o: src/main.cpp src/robot_configuration.h
	g++ -g -c src/main.cpp -o build/main.o

build/manipulator.o: src/manipulator.cpp src/manipulator.h src/robot_configuration.h
	g++ -g -c src/manipulator.cpp -o build/manipulator.o

clean:
	rm  -f build/*.o		
