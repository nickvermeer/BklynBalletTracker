SHELL=/bin/sh
CPPFLAGS=`pkg-config --cflags opencv`
CPPLINKERFLAGS=`pkg-config --cflags --libs opencv`
#objects_bgfg=bgfg_segm.o 
objects_dancer=dancer_segm.o 
objects_keypoint=Keypoints.o Keypoint_Tracker.o
all : dancer_segm Keypoint_Tracker 

#bgfg_segm:$(objects_bgfg)
#	g++ $(objects_bgfg) $(CPPLINKERFLAGS)  -o bgfg_segm

dancer_segm:$(objects_dancer)
	g++ $(objects_dancer) $(CPPLINKERFLAGS)  -o dancer_segm

Keypoint_Tracker:$(objects_keypoint)
	g++ $(objects_keypoint) $(CPPLINKERFLAGS)  -o Keypoint_Tracker



.PHONY : clean

clean :
	rm dancer_segm $(objects)
                  