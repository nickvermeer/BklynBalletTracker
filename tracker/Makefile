SHELL=/bin/sh
INCLUDES = -I ../../../TUIO_CPP/TUIO -I ../../../TUIO_CPP/oscpack
CPPFLAGS=-O2 -g -Wall -Wextra -pthread `pkg-config --cflags opencv` $(INCLUDES) 
CPPLINKERFLAGS=-O2 -g -Wall -Wextra -pthread `pkg-config --cflags --libs opencv` /usr/local/lib/libTUIO.so 
#CPPFLAGS=-O2 -pg -Wall -Wextra `pkg-config --cflags opencv` $(INCLUDES) 
#CPPLINKERFLAGS=-O2 -pg -Wall -Wextra `pkg-config --cflags --libs opencv` /usr/local/lib/libTUIO.so 

objects_keypoint=KeypointTracker.o MovementFilteredTracker.o TuioSender.o starter_video.o WarpPts.o #Warper.o
objects_threaded=KeypointTracker.o MovementFilteredTracker.o TuioSender.o threaded_video.o WarpPts.o
objects_threaded_single=KeypointTracker.o MovementFilteredTracker.o TuioSender.o threaded_video_single.o WarpPts.o

objects=$(objects_keypoint)
all : starter_video threaded_video threaded_video_single

#bgfg_segm:$(objects_bgfg)
#	g++ $(objects_bgfg) $(CPPLINKERFLAGS)  -o bgfg_segm

starter_video:$(objects_keypoint)
	g++ $(objects_keypoint) $(CPPLINKERFLAGS)  -o starter_video

threaded_video:$(objects_threaded)
	g++ $(objects_threaded) $(CPPLINKERFLAGS)  -o threaded_video

threaded_video_single:$(objects_threaded_single)
	g++ $(objects_threaded_single) $(CPPLINKERFLAGS)  -o threaded_video_single


.PHONY : clean

clean :
	rm starter_video threaded_video threaded_video.o threaded_video_single.o threaded_video_single $(objects) 
                  