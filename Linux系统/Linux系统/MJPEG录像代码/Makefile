main: main.o avilib.o v4l2uvc.o  
	gcc -o main main.o avilib.o v4l2uvc.o  
main.o: main.c avilib.h v4l2uvc.h  
	gcc -c main.c  
avilib.o: avilib.c avilib.h  
	gcc -c avilib.c  
v4l2uvc.o: v4l2uvc.c v4l2uvc.h  
	gcc -c v4l2uvc.c 
clean :
	rm main main.o avilib.o v4l2uvc.o  *.avi
	
