sudo apt install gpac
sudo apt install ffmpeg
gcc -g -O0 encode_video.c -lavformat -lavutil -lavdevice  -lavcodec -o encode_video.exe
./encode_video.exe或者(python photo.py)

