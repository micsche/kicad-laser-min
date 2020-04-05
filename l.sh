if g++ -o cvimage cvimage.cpp `pkg-config opencv4 --cflags --libs `; then
	./cvimage
else 
echo "Failure"; 
fi
