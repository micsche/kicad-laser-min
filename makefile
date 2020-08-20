.PHONY: all generate

CC = g++

# Comment next line to turn off optimization
# g++ -g kicadpcb2contour.cpp -o kicadcontour `pkg-config --cflags  opencv4 --libs opencv4`

CFLAGS = `pkg-config --cflags  opencv4`
LDFLAGS = `pkg-config --libs opencv4`


all: generate

generate:
	@echo "Creating KICAD contour "
	${CC} -g kicadpcb2contour.cpp -o kicadcontour ${CFLAGS} ${LDFLAGS}
	@echo "Done "
kicadcontour :
