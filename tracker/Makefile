SHELL=/bin/sh
INCLUDES = -I ../../../TUIO_CPP/TUIO -I ../../../TUIO_CPP/oscpack
CPPFLAGS=-g -O2 -Wall -Wextra `pkg-config --cflags opencv` $(INCLUDES) 
CPPLINKERFLAGS=-g -O2 -Wall -Wextra `pkg-config --cflags --libs opencv` /usr/local/lib/libTUIO.so 
#CPPFLAGS=-O2 -pg -Wall -Wextra `pkg-config --cflags opencv` $(INCLUDES) 
#CPPLINKERFLAGS=-O2 -pg -Wall -Wextra `pkg-config --cflags --libs opencv` /usr/local/lib/libTUIO.so 

objects_keypoint=KeypointTracker.o MovementFilteredTracker.o TuioSender.o starter_video.o WarpPts.o #Warper.o
objects=$(objects_keypoint)
all : starter_video

#bgfg_segm:$(objects_bgfg)
#	g++ $(objects_bgfg) $(CPPLINKERFLAGS)  -o bgfg_segm

starter_video:$(objects_keypoint)
	g++ $(objects_keypoint) $(CPPLINKERFLAGS)  -o starter_video


.PHONY : clean

clean :
	rm starter_video $(objects) 
                  