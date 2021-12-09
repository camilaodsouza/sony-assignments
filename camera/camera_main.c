/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <sdk/config.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <nuttx/video/video.h>

#include "camera_fileutil.h"

#include <fcntl.h>
#include <sys/boardctl.h>

#include <arch/board/cxd56_imageproc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define IMAGE_RGB_SIZE     (320*240*2) /* QVGA RGB565 */
#define IMAGE_UYVY_SIZE     (320*240*2)

#define IMAGE_WIDTH       (320)
#define IMAGE_HEIGHT      (240)
#define IMAGE_DATASIZE    (IMAGE_WIDTH*IMAGE_HEIGHT*2) /* QVGA YUV422 */

#define IMG_WIDTH_SAMPLE_REQUIRED     (120)
#define IMG_HEIGHT_SAMPLE_REQUIRED    (120)
#define IMG_DATASIZE_SAMPLE_REQUIRED  \
  (IMG_WIDTH_SAMPLE_REQUIRED*IMG_HEIGHT_SAMPLE_REQUIRED*2)

#define IMG_WIDTH_RECT_CLIP      (240)
#define IMG_HEIGHT_RECT_CLIP     (240)

#define DEFAULT_CAPTURE_NUM (10)
#define VIDEO_BUFNUM       (3)

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct v_buffer
{
  uint32_t *start; // Can I just turn this into a uint8_t? I used to be uint32
  uint32_t length;
};

typedef struct v_buffer v_buffer_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int camera_prepare(int fd, enum v4l2_buf_type type,
                          uint32_t buf_mode, uint32_t pixformat,
                          uint16_t hsize, uint16_t vsize,
                          struct v_buffer **vbuf,
                          uint8_t buffernum, int buffersize);
static void free_buffer(struct v_buffer  *buffers, uint8_t bufnum);
static unsigned char *get_camimage(int fd, struct v4l2_buffer *v4l2_buf,
    enum v4l2_buf_type buf_type);
static int release_camimage(int fd, struct v4l2_buffer *v4l2_buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static unsigned char *resized_mem;
static unsigned char *gray_mem;
const imageproc_rect_t clip_rect = {
  .x1 = (IMAGE_WIDTH - IMG_WIDTH_RECT_CLIP)/2,
  .y1 = (IMAGE_HEIGHT - IMG_HEIGHT_RECT_CLIP)/2,
  .x2 = (IMAGE_WIDTH - IMG_WIDTH_RECT_CLIP)/2 + IMG_WIDTH_RECT_CLIP -1,
  .y2 = (IMAGE_HEIGHT - IMG_HEIGHT_RECT_CLIP)/2 + IMG_HEIGHT_RECT_CLIP -1
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: camera_prepare()
 *
 * Description:
 *   Allocate frame buffer for camera and Queue the allocated buffer
 *   into video driver.
 ****************************************************************************/

static int camera_prepare(int fd, enum v4l2_buf_type type,
                          uint32_t buf_mode, uint32_t pixformat,
                          uint16_t hsize, uint16_t vsize,
                          struct v_buffer **vbuf,
                          uint8_t buffernum, int buffersize)
{
  int ret;
  int cnt;
  struct v4l2_format fmt =
  {
    0
  };

  struct v4l2_requestbuffers req =
  {
    0
  };

  struct v4l2_buffer buf =
  {
    0
  };

    /* VIDIOC_REQBUFS initiate user pointer I/O */

  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = buffernum;
  req.mode   = buf_mode;

  ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
      return ret;
    }
  
  /* VIDIOC_S_FMT set format */

  fmt.type                = type;
  fmt.fmt.pix.width       = hsize;
  fmt.fmt.pix.height      = vsize;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = pixformat;

  ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
      return ret;
    }

  /* Prepare video memory to store images */

  resized_mem = (unsigned char *)memalign(32, IMG_DATASIZE_SAMPLE_REQUIRED);
  if (!resized_mem)
    {
      return -1;
    }

  gray_mem = (unsigned char *)memalign(32, IMG_WIDTH_SAMPLE_REQUIRED*IMG_HEIGHT_SAMPLE_REQUIRED);
  if (!gray_mem)
    {
      return -1;
    }

  *vbuf = malloc(sizeof(v_buffer_t) * buffernum);

  if (!(*vbuf))
    {
      printf("Out of memory for array of v_buffer_t[%d]\n", buffernum);
      return ERROR;
    }

  for (cnt = 0; cnt < buffernum; cnt++)
    {
      (*vbuf)[cnt].length = buffersize;

      /* Note:
       * VIDIOC_QBUF set buffer pointer.
       * Buffer pointer must be 32bytes aligned.
       */

      (*vbuf)[cnt].start  = memalign(32, buffersize);
      if (!(*vbuf)[cnt].start)
        {
          printf("Out of memory for image buffer of %d/%d\n",
                 cnt, buffernum);
      
          /* Release allocated memory. */

          while (cnt)
            {
              cnt--;
              free((*vbuf)[cnt].start);
            }

          free(*vbuf);
          *vbuf = NULL;
          return ERROR;
        }
    }
  /* VIDIOC_QBUF enqueue buffer */

  for (cnt = 0; cnt < buffernum; cnt++)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = type;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = cnt;
      buf.m.userptr = (unsigned long)(*vbuf)[cnt].start;
      buf.length = (*vbuf)[cnt].length;

      ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          free_buffer(*vbuf, buffernum);
          *vbuf = NULL;
          return ERROR;
        }
    }

  /* VIDIOC_STREAMON start stream */

  ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
      free_buffer(*vbuf, buffernum);
      *vbuf = NULL;
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: free_buffer()
 *
 * Description:
 *   All free allocated memory of v_buffer.
 ****************************************************************************/

static void free_buffer(struct v_buffer  *buffers, uint8_t bufnum)
{
  uint8_t cnt;
  if (buffers)
    {
      for (cnt = 0; cnt < bufnum; cnt++)
        {
          if (buffers[cnt].start)
            {
              free(buffers[cnt].start);
            }
        }

      free(buffers);
    }
}

/****************************************************************************
 * Name: get_camimage()
 *
 * Description:
 *   DQBUF camera frame buffer from video driver with taken picture data.
 ****************************************************************************/
static unsigned char *get_camimage(int fd, struct v4l2_buffer *v4l2_buf,
    enum v4l2_buf_type buf_type)
{
  int ret;

  /* VIDIOC_DQBUF acquires captured data. */

  memset(v4l2_buf, 0, sizeof(v4l2_buffer_t));
  v4l2_buf->type = buf_type;
  v4l2_buf->memory = V4L2_MEMORY_USERPTR;

  ret = ioctl(fd, VIDIOC_DQBUF, (unsigned long)v4l2_buf);
  if (ret)
    {
      printf("Fail DQBUF %d\n", errno);
      return NULL;
    }

  return (unsigned char *)v4l2_buf->m.userptr;//OK;
}


/****************************************************************************
 * Name: release_camimage()
 *
 * Description:
 *   Re-QBUF to set used frame buffer into video driver.
 ****************************************************************************/

static int release_camimage(int fd, struct v4l2_buffer *v4l2_buf)
{
  int ret;

  /* VIDIOC_QBUF sets buffer pointer into video driver again. */

  ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)v4l2_buf);
  if (ret)
    {
      printf("Fail QBUF %d\n", errno);
      return ERROR;
    }

  return OK;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main()
 *
 * Description:
 *   main routine of this example.
 ****************************************************************************/

int camera_main(int argc, char *argv[])
{
  int ret;
  int v_fd;
  unsigned char *mem;
  enum v4l2_buf_type capture_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  struct v4l2_buffer v4l2_buf;
  const char *save_dir;
  
  struct v_buffer *buffers_video = NULL;

  // Select storage to save image files
  save_dir = futil_initialize();

  // Initialize video driver to create a device file
  ret = video_initialize("/dev/video");
  if (ret != 0){
    printf("ERROR: Failed to initialize video: errno = %d\n", errno);
    goto exit_without_cleaning_videodriver;
  }

  // Open the device file
  v_fd = open("/dev/video", 0);
  if (v_fd < 0)
    {
      printf("ERROR: Failed to open video.errno = %d\n", errno);
      ret = ERROR; 
      goto exit_without_cleaning_buffer;
    }

  // Prepare for VIDEO_CAPTURE stream
  ret = camera_prepare(v_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                       V4L2_BUF_MODE_RING, V4L2_PIX_FMT_UYVY,
                       VIDEO_HSIZE_QVGA, VIDEO_VSIZE_QVGA,
                       &buffers_video, VIDEO_BUFNUM, IMAGE_UYVY_SIZE);
  if (ret != OK)
    {
      goto exit_this_app;
    }

  imageproc_initialize();

  // Get image from camera
  mem = get_camimage(v_fd, &v4l2_buf, V4L2_BUF_TYPE_VIDEO_CAPTURE);
  if (mem != NULL)
    {
      if (imageproc_clip_and_resize(mem, IMAGE_WIDTH, IMAGE_HEIGHT, resized_mem,
                                     IMG_WIDTH_SAMPLE_REQUIRED, IMG_HEIGHT_SAMPLE_REQUIRED,
                                     16, (imageproc_rect_t *)&clip_rect) == 0)
        {
          imageproc_convert_yuv2gray(resized_mem, gray_mem,
              IMG_WIDTH_SAMPLE_REQUIRED, IMG_HEIGHT_SAMPLE_REQUIRED);
        }
      else
        {
          printf("imageproc_clic_and_resize failed.\n");
        }
    }
  else
    {
      printf("get_camimage failed.\n");
      goto exit_this_app;
    }

  // Save image to chosen storage
  imageproc_convert_yuv2rgb((uint8_t *)v4l2_buf.m.userptr, IMAGE_WIDTH, IMAGE_HEIGHT); //remove origRGB_mem
  futil_writeimage(
                  (uint8_t *)v4l2_buf.m.userptr,
                  (size_t)v4l2_buf.bytesused,
                  (capture_type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
                  "RGB" : "JPG");

  futil_writeimage(
                  gray_mem,
                  IMG_WIDTH_SAMPLE_REQUIRED*IMG_HEIGHT_SAMPLE_REQUIRED,
                  (capture_type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
                  "RGB" : "JPG");

  // Release image to return to buffer
  ret = release_camimage(v_fd, &v4l2_buf);
  if (ret != OK)
    {
      goto exit_this_app;
    }  


exit_this_app:

  /* Close video device file makes dequeue all buffers */

  close(v_fd);
  free_buffer(buffers_video, VIDEO_BUFNUM);

exit_without_cleaning_buffer:

  video_uninitialize();

exit_without_cleaning_videodriver:
  
  return ret;
}
