flags = -O3
cc = g++

main: RRT_Star_Dist.o main.o 
	$(cc) $(flags) -o main.out main.o RRT_Star_Dist.o

RRT_Star_Dist.o:RRT_Star_Dist.cpp
	$(cc) -c RRT_Star_Dist.cpp $(flags) 

main.o: main.cpp
	$(cc) $(flags) -c main.cpp

run: main
	./main.out

clean:
	rm *.o
	rm main.out
clean_save:
	rm Save/*.txt
