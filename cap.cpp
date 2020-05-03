#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEV_STR "/dev/video0"
#define QLEN    1
#define LOOP_CNT 10
//#define INTERACTIVE
#define ON_DEMAND

struct buffer {
    void *ptr;
    size_t length;
};

static void udelay(int u)
{
    clock_t goal = u + clock();
    while (goal > clock());
}

static void timediff(struct timeval *start, struct timeval *end, long *secs, long *usecs)
{
    *secs = (end->tv_sec - start->tv_sec); //avoid overflow by subtracting first
    *usecs = (end->tv_usec - start->tv_usec);
    if (*usecs < 0) {
        *secs -= 1;
        *usecs = 1000000 + *usecs;
    }
    return; 
}

int main(int argc, char *argv[])
{
    int fd, i, ret;
    long delay, sum;
    char *cmd = argv[0];
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    struct buffer *buffers;
    struct v4l2_buffer buf;
    void *ptr;
    enum v4l2_buf_type type;

    /* open video device */
    fd = open(DEV_STR, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        perror(cmd);
        return EXIT_FAILURE;
    }

    /* query and validate device capabiliities */
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        perror(cmd);
        return EXIT_FAILURE;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s: device (%s) does not have the capture capability\n", cmd, DEV_STR);
        return EXIT_FAILURE;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s: device (%s) does not have the capture capability\n", cmd, DEV_STR);
        return EXIT_FAILURE;
    }

    /* initialize v4l2 buffers */
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_REQBUFS, &req);
    if (ret < 0) {
        perror(cmd);
        return EXIT_FAILURE;
    }

    /* allocate user space buffers */
    buffers = (struct buffer *) malloc(req.count * sizeof(struct buffer));
    if (buffers == NULL) {
        perror(cmd);
        return EXIT_FAILURE;
    }

    /* map v4l2 buffers to user space */
    for (i = 0; i < req.count; i++) {
        memset(&buf, 0x00, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
        if (ret < 0) {
            perror(cmd);
            return EXIT_FAILURE;

        }
        ptr = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (ptr == MAP_FAILED) {
            perror(cmd);
            return EXIT_FAILURE;
        }
        buffers[i].ptr = ptr; 
        buffers[i].length = buf.length;
    } 

    /* queue buffers */
#ifndef ON_DEMAND
    for (i = 0; i < req.count; i++) {
        memset(&buf, 0x00, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        if (ret < 0) {
            perror(cmd);
            return EXIT_FAILURE;
        }
    }
#endif

    /* turn on stream */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    if (ret < 0) {
        perror(cmd);
        return EXIT_FAILURE;
    }

	long timelog[LOOP_CNT];
    /* main loop */
    for (i = 0; i < LOOP_CNT; i++) {
    	struct timeval start, end;
    	long secs, usecs;
    	fd_set fds;
    	struct timeval tv;

        /* prepare for v4l2 buffer */
        memset(&buf, 0x00, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
#ifdef ON_DEMAND
        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        if (ret < 0) {
            perror(cmd);
            return EXIT_FAILURE;
        }
#endif
        /* wait for camera capture */
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        gettimeofday(&start, NULL);
        ret = select(fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            perror(cmd);
            return EXIT_FAILURE;
        }
        if (ret == 0) {
            perror(cmd);
            return EXIT_FAILURE;
        }
        gettimeofday(&end, NULL);
        timediff(&start, &end, &secs, &usecs);
		timelog[i] = usecs;

        ret = ioctl(fd, VIDIOC_DQBUF, &buf);
        if (ret < 0) {
            perror(cmd);
            return EXIT_FAILURE;
        }
#ifndef ON_DEMAND
        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        if (ret < 0) {
            perror(cmd);
            return EXIT_FAILURE;
        }
#endif
        /* artifical delay */
        if (argv[1]) {
            delay = atol(argv[1]);
        }
        else {
            delay = 0;
        }
        udelay(delay);
#ifdef INTERACTIVE
    	IplImage* frame;
    	CvMat cvmat = cvMat(480, 640, CV_8UC3, buffers[buf.index].ptr);
    	frame = cvDecodeImage(&cvmat, 1);
    	cvNamedWindow("window",CV_WINDOW_AUTOSIZE);
    	cvShowImage("window", frame);
    	cvWaitKey(0);
    	cvSaveImage("image.jpg", frame, 0);
#endif
    }
    /* print each blocking time and average for select*/
    sum = 0;
	for (i = 3; i < LOOP_CNT; i++) {    /* skip first three */
		sum += timelog[i];
		printf("%ld;", timelog[i]);
	}
	printf("%ld\n", sum / (LOOP_CNT - 3));

    /* unmap and free user space buffers */
    for (i = 0; i < req.count; i++) {
        munmap(buffers[i].ptr, buffers[i].length);
    }
    free(buffers);
    
    /* close video device */
    close(fd);

    return EXIT_SUCCESS;
}
