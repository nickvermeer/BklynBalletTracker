SHELL=/bin/sh
CPPFLAGS=`pkg-config --cflags opencv`
CPPLINKERFLAGS=`pkg-config --cflags --libs opencv`
#objects_bgfg=bgfg_segm.o 
objects_dancer=dancer_segm.o 
objects_dualvideo=dualvideo.o 
objects_keypoint=Keypoints.o Keypoint_Tracker.o
objects_dualstitch=Keypoints.o dualstitch.o
objects=$(objects_dancer) $(objects_keypoint) $(objects_dualvideo) dualstitch.o
all : dancer_segm Keypoint_Tracker dualvideo dualstitch

#bgfg_segm:$(objects_bgfg)
#	g++ $(objects_bgfg) $(CPPLINKERFLAGS)  -o bgfg_segm

dancer_segm:$(objects_dancer)
	g++ $(objects_dancer) $(CPPLINKERFLAGS)  -o dancer_segm

dualvideo:$(objects_dualvideo)
	g++ $(objects_dualvideo) $(CPPLINKERFLAGS)  -o dualvideo

Keypoint_Tracker:$(objects_keypoint)
	g++ $(objects_keypoint) $(CPPLINKERFLAGS)  -o Keypoint_Tracker

dualstitch:$(objects_dualstitch)
	g++ $(objects_dualstitch) $(CPPLINKERFLAGS)  -o dualstitch



.PHONY : clean

clean :
	rm dancer_segm Keypoint_Tracker dualvideo dualstitch $(objects) 
                  