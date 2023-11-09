#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include "avilib.h"

#define NB_BUFFER 4
#define DHT_SIZE 432

struct vdIn {
    int fd;
    char *videodevice;
    char *status;
    char *pictName;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers rb;
    void *mem[NB_BUFFER];
    unsigned char *tmpbuffer;
    unsigned char *framebuffer;
    int isstreaming;
    int grabmethod;
    int width;
    int height;
    int fps;
    int formatIn;
    int formatOut;
    int framesizeIn;
    int signalquit;
    int toggleAvi;
    int getPict;
    int rawFrameCapture;
    /* raw frame capture */
    unsigned int fileCounter;
    /* raw frame stream capture */
    unsigned int rfsFramesWritten;
    unsigned int rfsBytesWritten;
    /* raw stream capture */
    FILE *captureFile;
    unsigned int framesWritten;
    unsigned int bytesWritten;
    avi_t *avifile;
    char *avifilename;
    int framecount;
    int recordstart;
    int recordtime;
};
int
init_videoIn(struct vdIn *vd, char *device, int width, int height, int fps,
	     int format, int grabmethod, char *avifilename);

int close_v4l2(struct vdIn *vd);

int enum_frame_intervals(int dev, __u32 pixfmt, __u32 width, __u32 height);
int enum_frame_sizes(int dev, __u32 pixfmt);
int enum_frame_formats(int dev, unsigned int *supported_formats, unsigned int max_formats);

int video_enable(struct vdIn *vd);
int video_disable(struct vdIn *vd);
