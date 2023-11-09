#include <string.h>
#include <time.h>
#include <unistd.h>

#include "libavdevice/avdevice.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"

#define V_WIDTH 1280
#define V_HEIGHT 720
#define FRAMES 90

void create_time_command(char command[]){
    time_t tt = time(0);
    char s[32];
    strftime(s, sizeof(s), "%02m%02d_%02H%02M%02S", localtime(&tt));
    sprintf(command,"MP4Box -add video.h264 -fps 30 ./result/out%s.mp4",s);
}

//@brief
// return
static AVFormatContext *open_dev() {
    int ret = 0;
    char errors[1024] = {
        0,
    };

    // ctx
    AVFormatContext *fmt_ctx = NULL;
    AVDictionary *options = NULL;

    //摄像头的设备文件
    char *devicename = "/dev/video6";

    // register video device 
    avdevice_register_all();

    // get format
    AVInputFormat *iformat = av_find_input_format("video4linux2");

    av_dict_set(&options, "video_size", "1280x720", 0);
    av_dict_set(&options, "framerate", "30", 0);
    av_dict_set(&options, "pixel_format", "yuyv422", 0);

    // open device
    ret = avformat_open_input(&fmt_ctx, devicename, iformat, &options);
    if (ret < 0) {
        av_strerror(ret, errors, 1024);
        fprintf(stderr, "Failed to open video device, [%d]%s\n", ret, errors);
        return NULL;
    }

    return fmt_ctx;
}

static void open_encoder(int width, int height, AVCodecContext **enc_ctx) {
    int ret = 0;
    AVCodec *codec = NULL;
    //新版本无需调用
    //avcodec_register_all();

    codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) {
        printf("Codec libx264 not found\n");
        exit(1);
    }

    *enc_ctx = avcodec_alloc_context3(codec);
    if (!enc_ctx) {
        printf("Could not allocate video codec context!\n");
        exit(1);
    }

    // SPS/PPS
    (*enc_ctx)->profile = FF_PROFILE_H264_HIGH_444;
    (*enc_ctx)->level = 50; //表示LEVEL是5.0

    //设置分辫率
    (*enc_ctx)->width = width;   // 640
    (*enc_ctx)->height = height; // 480

    // GOP
    (*enc_ctx)->gop_size = 250;
    (*enc_ctx)->keyint_min = 25; // option

    //设置B帧数据
    (*enc_ctx)->max_b_frames = 3; // option
    (*enc_ctx)->has_b_frames = 1; // option

    //参考帧的数量
    (*enc_ctx)->refs = 3; // option

    //设置输入YUV格式
    (*enc_ctx)->pix_fmt = AV_PIX_FMT_YUV420P;

    //设置码率
    (*enc_ctx)->bit_rate = 600000; // 600kbps

    //设置帧率
    (*enc_ctx)->time_base = (AVRational){1, 30}; //帧与帧之间的间隔是time_base
    (*enc_ctx)->framerate = (AVRational){30, 1}; //帧率，每秒 30帧

    ret = avcodec_open2((*enc_ctx), codec, NULL);
    if (ret < 0) {
        printf("Could not open codec: %s!\n", av_err2str(ret));
        exit(1);
    }
}

static AVFrame *create_frame(int width, int height) {
    int ret = 0;
    AVFrame *frame = NULL;

    frame = av_frame_alloc();
    if (!frame) {
        printf("Error, No Memory!\n");
        goto __ERROR;
    }

    //设置参数
    frame->width = width;
    frame->height = height;
    frame->format = AV_PIX_FMT_YUV420P;

    // alloc inner memory
    ret = av_frame_get_buffer(frame, 32); //按 32 位对齐
    if (ret < 0) {
        printf("Error, Failed to alloc buffer for frame!\n");
        goto __ERROR;
    }

    return frame;

__ERROR:
    if (frame) {
        av_frame_free(&frame);
    }

    return NULL;
}

static void encode(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *newpkt,
                   FILE *outfile) {
    int ret = 0;
    if (frame) {
        //printf("send frame to encoder, pts=%ld", frame->pts);
    }
    //送原始数据给编码器进行编码
    ret = avcodec_send_frame(enc_ctx, frame);
    if (ret < 0) {
        printf("Error, Failed to send a frame for enconding!\n");
        exit(1);
    }

    //从编码器获取编码好的数据
    while (ret >= 0) {
        ret = avcodec_receive_packet(enc_ctx, newpkt);

        //如果编码器数据不足时会返回  EAGAIN,或者到数据尾时会返回 AVERROR_EOF
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            return;
        } else if (ret < 0) {
            printf("Error, Failed to encode!\n");
            exit(1);
        }

        fwrite(newpkt->data, 1, newpkt->size, outfile);
        fflush(outfile);
        av_packet_unref(newpkt);
    }
}

void yuyv422ToYuv420p(AVFrame *frame, AVPacket *pkt) {
    int i = 0;
    int yuv422_length = V_WIDTH * V_HEIGHT * 2;
    int y_index = 0;
    // copy all y
    for (i = 0; i < yuv422_length; i += 2) {
        frame->data[0][y_index] = pkt->data[i];
        y_index++;
    }

    // copy u and v
    int line_start = 0;
    int is_u = 1;
    int u_index = 0;
    int v_index = 0;
    // copy u, v per line. skip a line once
    for (i = 0; i < V_HEIGHT; i += 2) {
        // line i offset
        line_start = i * V_WIDTH * 2;
        for (int j = line_start + 1; j < line_start + V_WIDTH * 2; j += 4) {
            frame->data[1][u_index] = pkt->data[j];
            u_index++;
            frame->data[2][v_index] = pkt->data[j + 2];
            v_index++;
        }
    }
}

void rec_video() {
    int ret = 0;
    int base = 0;
    int count = 0;
    int frame_index = 0;
    int64_t calc_duration = 0;
    // pakcet
    AVPacket pkt;
    AVFormatContext *fmt_ctx = NULL;
    AVCodecContext *enc_ctx = NULL;

    // set log level
    av_log_set_level(AV_LOG_DEBUG);

    // create file
    char *yuvout = "./video.yuv";
    char *out = "./video.h264";

    FILE *yuvoutfile = fopen(yuvout, "wb+");
    FILE *outfile = fopen(out, "wb+");

    //打开设备
    fmt_ctx = open_dev();

    //打开编码器
    open_encoder(V_WIDTH, V_HEIGHT, &enc_ctx);
    
    calc_duration = (double)AV_TIME_BASE / av_q2d((*enc_ctx).framerate);

    //创建 AVFrame
    AVFrame *frame = create_frame(V_WIDTH, V_HEIGHT);

    //创建编码后输出的Packet
    AVPacket *newpkt = av_packet_alloc();
    if (!newpkt) {
        printf("Error, Failed to alloc avpacket!\n");
        goto __ERROR;
    }

    // read data from device
    while (((ret = av_read_frame(fmt_ctx, &pkt)) == 0) && (count++ < FRAMES)) { 
        int i = 0;
        //av_log(NULL, AV_LOG_INFO, "packet size is %d(%p)\n", pkt.size,
        //       pkt.data);

        // YUYVYUYVYUYVYUYV   YUYV422
        // YYYYYYYYUUVV YUV420
        yuyv422ToYuv420p(frame, &pkt);

        fwrite(frame->data[0], 1, 307200, yuvoutfile);
        fwrite(frame->data[1], 1, 307200 / 4, yuvoutfile);
        fwrite(frame->data[2], 1, 307200 / 4, yuvoutfile);

        frame->pts = base++;
        
        
        if(newpkt->pts == AV_NOPTS_VALUE) {
          //Write PTS
          AVRational time_base1 =  (*enc_ctx).time_base;
          //Duration between 2 frames (us)
          //Parameters
          newpkt->pts = (double)(frame_index*calc_duration) / (double)(av_q2d(time_base1)*AV_TIME_BASE);
          newpkt->dts = newpkt->pts;
          newpkt->duration = (double)calc_duration / (double)(av_q2d(time_base1)*AV_TIME_BASE);
          frame_index++;
        }
        encode(enc_ctx, frame, newpkt, outfile);
        //
        av_packet_unref(&pkt); // release pkt
    }

    encode(enc_ctx, NULL, newpkt, outfile);

__ERROR:
    if (yuvoutfile) {
        // close file
        fclose(yuvoutfile);
    }

    // close device and release ctx
    if (fmt_ctx) {
        avformat_close_input(&fmt_ctx);
    }

    av_log(NULL, AV_LOG_DEBUG, "finish!\n");
    return;
}


int main(int argc, char *argv[]) {
    rec_video();
    char command[128];
    create_time_command(command);
    system(command);
    return 0;
}

