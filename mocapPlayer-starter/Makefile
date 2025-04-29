# Motion capture viewer Makefile 
# Jernej Barbic, Yili Zhao, USC

include Makefile.FLTK

FLTK_PATH=../fltk-1.3.11
PLAYER_OBJECT_FILES = displaySkeleton.o interface.o motion.o posture.o skeleton.o transform.o vector.o mocapPlayer.o ppm.o pic.o performanceCounter.o 
INTERPOLATE_OBJECT_FILES = motion.o posture.o skeleton.o transform.o vector.o interpolator.o quaternion.o interpolate.o
GRAPH_OBJECT_FILES = graph.o
GRAPH_FRONTEND_FILES = graph-frontend.o
COMPILER = g++ -std=c++17
COMPILEMODE= -O2
COMPILERFLAGS = $(COMPILEMODE) -I$(FLTK_PATH) $(CXXFLAGS)
LINKERFLAGS = $(COMPILEMODE) $(LINKFLTK_ALL)

all: mocapPlayer interpolate graph graph-frontend

mocapPlayer: $(PLAYER_OBJECT_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -o $@

graph: $(GRAPH_OBJECT_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -o $@

graph-frontend: $(GRAPH_FRONTEND_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -o $@

interpolate: $(INTERPOLATE_OBJECT_FILES)
	$(COMPILER) $^ $(LINKERFLAGS) -o $@

%.o: %.cpp 
	$(COMPILER) -c $(COMPILERFLAGS) $^

clean:
	-rm -rf core *.o *~ "#"*"#" test

