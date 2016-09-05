cc = clang++
includes = -I /usr/local/include
copts = -Wall -std=c++11 -O2 -g
links = -framework OpenGL -L /usr/local/lib -lglfw3 -lepoxy
srcs = window.cpp main.cpp
target = trajedit

all:
	$(cc) $(includes) $(copts) $(links) -o $(target) $(srcs)
