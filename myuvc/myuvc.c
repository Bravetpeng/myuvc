
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/version.h>
#include <asm/unaligned.h>
#include <linux/usb/input.h>
#include <linux/mm.h>
#include <linux/hid.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-core.h>

#include "uvcvideo.h"

#define  UVC_URBS  5

/* 2.4.3.3. Payload Header Information */
#define UVC_STREAM_EOH					(1 << 7)
#define UVC_STREAM_ERR					(1 << 6)
#define UVC_STREAM_STI					(1 << 5)
#define UVC_STREAM_RES					(1 << 4)
#define UVC_STREAM_SCR					(1 << 3)
#define UVC_STREAM_PTS					(1 << 2)
#define UVC_STREAM_EOF					(1 << 1)
#define UVC_STREAM_FID					(1 << 0)


struct frame_desc {
    int width;
    int height;
};

struct myuvc_streaming_control {
	__u16 bmHint;
	__u8  bFormatIndex;
	__u8  bFrameIndex;
	__u32 dwFrameInterval;
	__u16 wKeyFrameRate;
	__u16 wPFrameRate;
	__u16 wCompQuality;
	__u16 wCompWindowSize;
	__u16 wDelay;
	__u32 dwMaxVideoFrameSize;
	__u32 dwMaxPayloadTransferSize;
	__u32 dwClockFrequency;
	__u8  bmFramingInfo;
	__u8  bPreferedVersion;
	__u8  bMinVersion;
	__u8  bMaxVersion;
};


/* 参考uvc_video_queue定义一些结构体 */
struct myuvc_buffer {
    struct v4l2_buffer buf;
    int state;
    int vma_use_count;       /* 表示是否已经被mmap */
    wait_queue_head_t wait;  /* APP要读某个缓冲区,如果无数据,在此休眠 */
	struct list_head stream;
	struct list_head irq; 
};

struct myuvc_queue {
    void *mem;
    int count;
    int buf_size;    
    struct myuvc_buffer buffer[32];
	struct list_head mainqueue;   /* 供APP消费用 */
	struct list_head irqqueue;    /* 供底层驱动生产用 */

	struct urb *urb[32];
	char *urb_buffer[32];
	dma_addr_t urb_dma[32];
	unsigned int urb_size;
};

static struct myuvc_queue myuvc_queue;

static struct video_device *myuvc_vdev;

static struct v4l2_format myuvc_format;
static struct frame_desc frames[] = {{640, 480}, {320, 240}, {160, 120}};
static int frame_idx = 1;
static int bBitsPerPixel = 0; /* 压缩的MJPEG数据, 不需要这个位 */
static int myuvc_version = 0x0100;
static struct myuvc_streaming_control  myuvc_streaming_ctl; 
static struct usb_device  *mydev;
static int myuvc_streaming_intf;
static int myuvc_control_intf;
static int myuvc_streaming_bAlternateSetting = 5;
static int myuvc_bEndpointAddress = 0x82;    /* 流接口 端点地址 */
static int myuvc_bInterval  = 1;
static int ProcessingUnitID = 3;

static int wMaxPacketSize      = 800;
static int dwMaxVideoFrameSize = 77312;

static int last_fid = -1;

static const char *get_guid(const unsigned char *buf)
{
	static char guid[39];

	/* NOTE:  see RFC 4122 for more information about GUID/UUID
	 * structure.  The first fields fields are historically big
	 * endian numbers, dating from Apollo mc68000 workstations.
	 */
	sprintf(guid, "{%02x%02x%02x%02x"
			"-%02x%02x"
			"-%02x%02x"
			"-%02x%02x"
			"-%02x%02x%02x%02x%02x%02x}",
	       buf[0], buf[1], buf[2], buf[3],
	       buf[4], buf[5],
	       buf[6], buf[7],
	       buf[8], buf[9],
	       buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	return guid;
}

static void parse_videocontrol_interface(struct usb_interface *intf, unsigned char *buf, int buflen)
{
    static const char * const ctrlnames[] = {
        "Brightness", "Contrast", "Hue", "Saturation", "Sharpness", "Gamma",
        "White Balance Temperature", "White Balance Component", "Backlight Compensation",
        "Gain", "Power Line Frequency", "Hue, Auto", "White Balance Temperature, Auto",
        "White Balance Component, Auto", "Digital Multiplier", "Digital Multiplier Limit",
        "Analog Video Standard", "Analog Video Lock Status"
    };
    static const char * const camctrlnames[] = {
        "Scanning Mode", "Auto-Exposure Mode", "Auto-Exposure Priority",
        "Exposure Time (Absolute)", "Exposure Time (Relative)", "Focus (Absolute)",
        "Focus (Relative)", "Iris (Absolute)", "Iris (Relative)", "Zoom (Absolute)",
        "Zoom (Relative)", "PanTilt (Absolute)", "PanTilt (Relative)",
        "Roll (Absolute)", "Roll (Relative)", "Reserved", "Reserved", "Focus, Auto",
        "Privacy"
    };
    static const char * const stdnames[] = {
        "None", "NTSC - 525/60", "PAL - 625/50", "SECAM - 625/50",
        "NTSC - 625/50", "PAL - 525/60" };
    unsigned int i, ctrls, stds, n, p, termt, freq;

    while (buflen > 0)
    {

        if (buf[1] != USB_DT_CS_INTERFACE)
            printk("      Warning: Invalid descriptor\n");
        else if (buf[0] < 3)
            printk("      Warning: Descriptor too short\n");
        printk("      VideoControl Interface Descriptor:\n"
               "        bLength             %5u\n"
               "        bDescriptorType     %5u\n"
               "        bDescriptorSubtype  %5u ",
               buf[0], buf[1], buf[2]);
        switch (buf[2]) {
        case 0x01:  /* HEADER */
            printk("(HEADER)\n");
            n = buf[11];
            if (buf[0] < 12+n)
                printk("      Warning: Descriptor too short\n");
            freq = buf[7] | (buf[8] << 8) | (buf[9] << 16) | (buf[10] << 24);
            printk("        bcdUVC              %2x.%02x\n"
                   "        wTotalLength        %5u\n"
                   "        dwClockFrequency    %5u.%06uMHz\n"
                   "        bInCollection       %5u\n",
                   buf[4], buf[3], buf[5] | (buf[6] << 8), freq / 1000000,
                   freq % 1000000, n);
            for (i = 0; i < n; i++)
                printk("        baInterfaceNr(%2u)   %5u\n", i, buf[12+i]);
            break;

        case 0x02:  /* INPUT_TERMINAL */
            printk("(INPUT_TERMINAL)\n");
            termt = buf[4] | (buf[5] << 8);
            n = termt == 0x0201 ? 7 : 0;
            if (buf[0] < 8 + n)
                printk("      Warning: Descriptor too short\n");
            printk("        bTerminalID         %5u\n"
                   "        wTerminalType      0x%04x\n"
                   "        bAssocTerminal      %5u\n",
                   buf[3], termt, buf[6]);
            printk("        iTerminal           %5u\n",
                   buf[7]);
            if (termt == 0x0201) {
                n += buf[14];
                printk("        wObjectiveFocalLengthMin  %5u\n"
                       "        wObjectiveFocalLengthMax  %5u\n"
                       "        wOcularFocalLength        %5u\n"
                       "        bControlSize              %5u\n",
                       buf[8] | (buf[9] << 8), buf[10] | (buf[11] << 8),
                       buf[12] | (buf[13] << 8), buf[14]);
                ctrls = 0;
                for (i = 0; i < 3 && i < buf[14]; i++)
                    ctrls = (ctrls << 8) | buf[8+n-i-1];
                printk("        bmControls           0x%08x\n", ctrls);
                for (i = 0; i < 19; i++)
                    if ((ctrls >> i) & 1)
                        printk("          %s\n", camctrlnames[i]);
            }
            break;

        case 0x03:  /* OUTPUT_TERMINAL */
            printk("(OUTPUT_TERMINAL)\n");
            termt = buf[4] | (buf[5] << 8);
            if (buf[0] < 9)
                printk("      Warning: Descriptor too short\n");
            printk("        bTerminalID         %5u\n"
                   "        wTerminalType      0x%04x\n"
                   "        bAssocTerminal      %5u\n"
                   "        bSourceID           %5u\n"
                   "        iTerminal           %5u\n",
                   buf[3], termt, buf[6], buf[7], buf[8]);
            break;

        case 0x04:  /* SELECTOR_UNIT */
            printk("(SELECTOR_UNIT)\n");
            p = buf[4];
            if (buf[0] < 6+p)
                printk("      Warning: Descriptor too short\n");

            printk("        bUnitID             %5u\n"
                   "        bNrInPins           %5u\n",
                   buf[3], p);
            for (i = 0; i < p; i++)
                printk("        baSource(%2u)        %5u\n", i, buf[5+i]);
            printk("        iSelector           %5u\n",
                   buf[5+p]);
            break;

        case 0x05:  /* PROCESSING_UNIT */
            printk("(PROCESSING_UNIT)\n");
            n = buf[7];
            if (buf[0] < 10+n)
                printk("      Warning: Descriptor too short\n");
            printk("        bUnitID             %5u\n"
                   "        bSourceID           %5u\n"
                   "        wMaxMultiplier      %5u\n"
                   "        bControlSize        %5u\n",
                   buf[3], buf[4], buf[5] | (buf[6] << 8), n);
            ctrls = 0;
            for (i = 0; i < 3 && i < n; i++)
                ctrls = (ctrls << 8) | buf[8+n-i-1];
            printk("        bmControls     0x%08x\n", ctrls);
            for (i = 0; i < 18; i++)
                if ((ctrls >> i) & 1)
                    printk("          %s\n", ctrlnames[i]);
            stds = buf[9+n];
            printk("        iProcessing         %5u\n"
                   "        bmVideoStandards     0x%2x\n", buf[8+n], stds);
            for (i = 0; i < 6; i++)
                if ((stds >> i) & 1)
                    printk("          %s\n", stdnames[i]);
            break;

        case 0x06:  /* EXTENSION_UNIT */
            printk("(EXTENSION_UNIT)\n");
            p = buf[21];
            n = buf[22+p];
            if (buf[0] < 24+p+n)
                printk("      Warning: Descriptor too short\n");
            printk("        bUnitID             %5u\n"
                   "        guidExtensionCode         %s\n"
                   "        bNumControl         %5u\n"
                   "        bNrPins             %5u\n",
                   buf[3], get_guid(&buf[4]), buf[20], buf[21]);
            for (i = 0; i < p; i++)
                printk("        baSourceID(%2u)      %5u\n", i, buf[22+i]);
            printk("        bControlSize        %5u\n", buf[22+p]);
            for (i = 0; i < n; i++)
                printk("        bmControls(%2u)       0x%02x\n", i, buf[23+p+i]);
            printk("        iExtension          %5u\n",
                   buf[23+p+n]);
            break;

        default:
            printk("(unknown)\n"
                   "        Invalid desc subtype:");
            break;
        }

        buflen -= buf[0];
        buf    += buf[0];
    }
}


static void parse_videostreaming_interface(struct usb_interface *intf, unsigned char *buf, int buflen)
{
    static const char * const colorPrims[] = { "Unspecified", "BT.709,sRGB",
        "BT.470-2 (M)", "BT.470-2 (B,G)", "SMPTE 170M", "SMPTE 240M" };
    static const char * const transferChars[] = { "Unspecified", "BT.709",
        "BT.470-2 (M)", "BT.470-2 (B,G)", "SMPTE 170M", "SMPTE 240M",
        "Linear", "sRGB"};
    static const char * const matrixCoeffs[] = { "Unspecified", "BT.709",
        "FCC", "BT.470-2 (B,G)", "SMPTE 170M (BT.601)", "SMPTE 240M" };
    unsigned int i, m, n, p, flags, len;

    while (buflen > 0)
    {

        if (buf[1] != USB_DT_CS_INTERFACE)
            printk("      Warning: Invalid descriptor\n");
        else if (buf[0] < 3)
            printk("      Warning: Descriptor too short\n");
        printk("      VideoStreaming Interface Descriptor:\n"
               "        bLength                         %5u\n"
               "        bDescriptorType                 %5u\n"
               "        bDescriptorSubtype              %5u ",
               buf[0], buf[1], buf[2]);
        switch (buf[2]) {
        case 0x01: /* INPUT_HEADER */
            printk("(INPUT_HEADER)\n");
            p = buf[3];
            n = buf[12];
            if (buf[0] < 13+p*n)
                printk("      Warning: Descriptor too short\n");
            printk("        bNumFormats                     %5u\n"
                   "        wTotalLength                    %5u\n"
                   "        bEndPointAddress                %5u\n"
                   "        bmInfo                          %5u\n"
                   "        bTerminalLink                   %5u\n"
                   "        bStillCaptureMethod             %5u\n"
                   "        bTriggerSupport                 %5u\n"
                   "        bTriggerUsage                   %5u\n"
                   "        bControlSize                    %5u\n",
                   p, buf[4] | (buf[5] << 8), buf[6], buf[7], buf[8],
                   buf[9], buf[10], buf[11], n);
            for (i = 0; i < p; i++)
                printk(
                "        bmaControls(%2u)                 %5u\n",
                    i, buf[13+p*n]);
            break;

        case 0x02: /* OUTPUT_HEADER */
            printk("(OUTPUT_HEADER)\n");
            p = buf[3];
            n = buf[8];
            if (buf[0] < 9+p*n)
                printk("      Warning: Descriptor too short\n");
            printk("        bNumFormats                 %5u\n"
                   "        wTotalLength                %5u\n"
                   "        bEndpointAddress            %5u\n"
                   "        bTerminalLink               %5u\n"
                   "        bControlSize                %5u\n",
                   p, buf[4] | (buf[5] << 8), buf[6], buf[7], n);
            for (i = 0; i < p; i++)
                printk(
                "        bmaControls(%2u)             %5u\n",
                    i, buf[9+p*n]);
            break;

        case 0x03: /* STILL_IMAGE_FRAME */
            printk("(STILL_IMAGE_FRAME)\n");
            n = buf[4];
            m = buf[5+4*n];
            if (buf[0] < 6+4*n+m)
                printk("      Warning: Descriptor too short\n");
            printk("        bEndpointAddress                %5u\n"
                   "        bNumImageSizePatterns             %3u\n",
                   buf[3], n);
            for (i = 0; i < n; i++)
                printk("        wWidth(%2u)                      %5u\n"
                       "        wHeight(%2u)                     %5u\n",
                       i, buf[5+4*i] | (buf[6+4*i] << 8),
                       i, buf[7+4*i] | (buf[8+4*i] << 8));
            printk("        bNumCompressionPatterns           %3u\n", n);
            for (i = 0; i < m; i++)
                printk("        bCompression(%2u)                %5u\n",
                       i, buf[6+4*n+i]);
            break;

        case 0x04: /* FORMAT_UNCOMPRESSED */
        case 0x10: /* FORMAT_FRAME_BASED */
            if (buf[2] == 0x04) {
                printk("(FORMAT_UNCOMPRESSED)\n");
                len = 27;
            } else {
                printk("(FORMAT_FRAME_BASED)\n");
                len = 28;
            }
            if (buf[0] < len)
                printk("      Warning: Descriptor too short\n");
            flags = buf[25];
            printk("        bFormatIndex                    %5u\n"
                   "        bNumFrameDescriptors            %5u\n"
                   "        guidFormat                            %s\n"
                   "        bBitsPerPixel                   %5u\n"
                   "        bDefaultFrameIndex              %5u\n"
                   "        bAspectRatioX                   %5u\n"
                   "        bAspectRatioY                   %5u\n"
                   "        bmInterlaceFlags                 0x%02x\n",
                   buf[3], buf[4], get_guid(&buf[5]), buf[21], buf[22],
                   buf[23], buf[24], flags);
            printk("          Interlaced stream or variable: %s\n",
                   (flags & (1 << 0)) ? "Yes" : "No");
            printk("          Fields per frame: %u fields\n",
                   (flags & (1 << 1)) ? 1 : 2);
            printk("          Field 1 first: %s\n",
                   (flags & (1 << 2)) ? "Yes" : "No");
            printk("          Field pattern: ");
            switch ((flags >> 4) & 0x03) {
            case 0:
                printk("Field 1 only\n");
                break;
            case 1:
                printk("Field 2 only\n");
                break;
            case 2:
                printk("Regular pattern of fields 1 and 2\n");
                break;
            case 3:
                printk("Random pattern of fields 1 and 2\n");
                break;
            }
            printk("          bCopyProtect                  %5u\n", buf[26]);
            if (buf[2] == 0x10)
                printk("          bVariableSize                 %5u\n", buf[27]);
            break;

        case 0x05: /* FRAME UNCOMPRESSED */
        case 0x07: /* FRAME_MJPEG */
        case 0x11: /* FRAME_FRAME_BASED */
            if (buf[2] == 0x05) {
                printk("(FRAME_UNCOMPRESSED)\n");
                n = 25;
            } else if (buf[2] == 0x07) {
                printk("(FRAME_MJPEG)\n");
                n = 25;
            } else {
                printk("(FRAME_FRAME_BASED)\n");
                n = 21;
            }
            len = (buf[n] != 0) ? (26+buf[n]*4) : 38;
            if (buf[0] < len)
                printk("      Warning: Descriptor too short\n");
            flags = buf[4];
            printk("        bFrameIndex                     %5u\n"
                   "        bmCapabilities                   0x%02x\n",
                   buf[3], flags);
            printk("          Still image %ssupported\n",
                   (flags & (1 << 0)) ? "" : "un");
            if (flags & (1 << 1))
                printk("          Fixed frame-rate\n");
            printk("        wWidth                          %5u\n"
                   "        wHeight                         %5u\n"
                   "        dwMinBitRate                %9u\n"
                   "        dwMaxBitRate                %9u\n",
                   buf[5] | (buf[6] <<  8), buf[7] | (buf[8] << 8),
                   buf[9] | (buf[10] << 8) | (buf[11] << 16) | (buf[12] << 24),
                   buf[13] | (buf[14] << 8) | (buf[15] << 16) | (buf[16] << 24));
            if (buf[2] == 0x11)
                printk("        dwDefaultFrameInterval      %9u\n"
                       "        bFrameIntervalType              %5u\n"
                       "        dwBytesPerLine              %9u\n",
                       buf[17] | (buf[18] << 8) | (buf[19] << 16) | (buf[20] << 24),
                       buf[21],
                       buf[22] | (buf[23] << 8) | (buf[24] << 16) | (buf[25] << 24));
            else
                printk("        dwMaxVideoFrameBufferSize   %9u\n"
                       "        dwDefaultFrameInterval      %9u\n"
                       "        bFrameIntervalType              %5u\n",
                       buf[17] | (buf[18] << 8) | (buf[19] << 16) | (buf[20] << 24),
                       buf[21] | (buf[22] << 8) | (buf[23] << 16) | (buf[24] << 24),
                       buf[25]);
            if (buf[n] == 0)
                printk("        dwMinFrameInterval          %9u\n"
                       "        dwMaxFrameInterval          %9u\n"
                       "        dwFrameIntervalStep         %9u\n",
                       buf[26] | (buf[27] << 8) | (buf[28] << 16) | (buf[29] << 24),
                       buf[30] | (buf[31] << 8) | (buf[32] << 16) | (buf[33] << 24),
                       buf[34] | (buf[35] << 8) | (buf[36] << 16) | (buf[37] << 24));
            else
                for (i = 0; i < buf[n]; i++)
                    printk("        dwFrameInterval(%2u)         %9u\n",
                           i, buf[26+4*i] | (buf[27+4*i] << 8) |
                           (buf[28+4*i] << 16) | (buf[29+4*i] << 24));
            break;

        case 0x06: /* FORMAT_MJPEG */
            printk("(FORMAT_MJPEG)\n");
            if (buf[0] < 11)
                printk("      Warning: Descriptor too short\n");
            flags = buf[5];
            printk("        bFormatIndex                    %5u\n"
                   "        bNumFrameDescriptors            %5u\n"
                   "        bFlags                          %5u\n",
                   buf[3], buf[4], flags);
            printk("          Fixed-size samples: %s\n",
                   (flags & (1 << 0)) ? "Yes" : "No");
            flags = buf[9];
            printk("        bDefaultFrameIndex              %5u\n"
                   "        bAspectRatioX                   %5u\n"
                   "        bAspectRatioY                   %5u\n"
                   "        bmInterlaceFlags                 0x%02x\n",
                   buf[6], buf[7], buf[8], flags);
            printk("          Interlaced stream or variable: %s\n",
                   (flags & (1 << 0)) ? "Yes" : "No");
            printk("          Fields per frame: %u fields\n",
                   (flags & (1 << 1)) ? 2 : 1);
            printk("          Field 1 first: %s\n",
                   (flags & (1 << 2)) ? "Yes" : "No");
            printk("          Field pattern: ");
            switch ((flags >> 4) & 0x03) {
            case 0:
                printk("Field 1 only\n");
                break;
            case 1:
                printk("Field 2 only\n");
                break;
            case 2:
                printk("Regular pattern of fields 1 and 2\n");
                break;
            case 3:
                printk("Random pattern of fields 1 and 2\n");
                break;
            }
            printk("          bCopyProtect                  %5u\n", buf[10]);
            break;

        case 0x0a: /* FORMAT_MPEG2TS */
            printk("(FORMAT_MPEG2TS)\n");
            len = buf[0] < 23 ? 7 : 23;
            if (buf[0] < len)
                printk("      Warning: Descriptor too short\n");
            printk("        bFormatIndex                    %5u\n"
                   "        bDataOffset                     %5u\n"
                   "        bPacketLength                   %5u\n"
                   "        bStrideLength                   %5u\n",
                   buf[3], buf[4], buf[5], buf[6]);
            if (len > 7)
                printk("        guidStrideFormat                      %s\n",
                       get_guid(&buf[7]));
            break;

        case 0x0d: /* COLORFORMAT */
            printk("(COLORFORMAT)\n");
            if (buf[0] < 6)
                printk("      Warning: Descriptor too short\n");
            printk("        bColorPrimaries                 %5u (%s)\n",
                   buf[3], (buf[3] <= 5) ? colorPrims[buf[3]] : "Unknown");
            printk("        bTransferCharacteristics        %5u (%s)\n",
                   buf[4], (buf[4] <= 7) ? transferChars[buf[4]] : "Unknown");
            printk("        bMatrixCoefficients             %5u (%s)\n",
                   buf[5], (buf[5] <= 5) ? matrixCoeffs[buf[5]] : "Unknown");
            break;

        default:
            printk("        Invalid desc subtype:");
            break;
        }
        buflen -= buf[0];
        buf    += buf[0];
    }
}

static void myuvc_release(struct video_device *vdev)
{
}

/* S1 打开 */
static int myuvc_open(struct file *file)
{
	return 0;
}

/* S2 查询设备支持的能力 */
static int myuvc_vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	memset(cap, 0, sizeof *cap);
	strcpy(cap->driver, "myuvcvideo");
	strcpy(cap->card, "myuvcvideo");
	cap->version = 1;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

/* S3 列举支持哪种格式 */
static int myuvc_vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	/* 人工查看描述符可知我们用的摄像头只支持1种格式 */
	if (f->index >= 1)
		return -EINVAL;

	strcpy(f->description, "MJPEG");
	f->pixelformat = V4L2_PIX_FMT_MJPEG;   
	//f->flags        = ;
	f->type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
	return 0;
}

/* S4 返回当前所使用的格式 */
static int myuvc_vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	memcpy(f, &myuvc_format, sizeof(myuvc_format));
	return (0);

}

/* S5 测试驱动程序是否支持某种格式,并设置为该格式的分辨率 */
static int myuvc_vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        return -EINVAL;
    }

    if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
        return -EINVAL;
    
    /* 调整format的width, height, 
     * 计算bytesperline, sizeimage
     */

    /* 人工查看描述符, 确定支持哪几种分辨率 */
    f->fmt.pix.width  = frames[frame_idx].width;
    f->fmt.pix.height = frames[frame_idx].height;
    
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * bBitsPerPixel) >> 3;
        f->fmt.pix.sizeimage = dwMaxVideoFrameSize;

	f->fmt.pix.field      = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	f->fmt.pix.priv       = 0;		/* private data, depends on pixelformat */
	
    return 0;
}

/* S6 设置format */
static int myuvc_vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int  ret = myuvc_vidioc_try_fmt_vid_cap(file, NULL, f);
	if(ret < 0)
		return ret;

	memcpy(&myuvc_format, f, sizeof myuvc_format);
	
	return 0;
}

static int myuvc_free_buffers(void)
{
	if (myuvc_queue.mem)
	{
	    vfree(myuvc_queue.mem);
	    memset(&myuvc_queue, 0, sizeof(myuvc_queue));
	    myuvc_queue.mem = NULL;
	}
	return 0;
}


/* S7 申请缓冲区 参考：uvc_alloc_buffers
 * APP调用该ioctl让驱动程序分配若干个缓存, APP将从这些缓存中读到视频数据 
 */
static int myuvc_vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
    int nbuffers = p->count;
    int bufsize  = PAGE_ALIGN(myuvc_format.fmt.pix.sizeimage);
    unsigned int i;
    void *mem = NULL;
    int ret;

    if ((ret = myuvc_free_buffers()) < 0)
        goto done;

    /* Bail out if no buffers should be allocated. */
    if (nbuffers == 0)
        goto done;

    /* Decrement the number of buffers until allocation succeeds. */
    for (; nbuffers > 0; --nbuffers) {
        mem = vmalloc_32(nbuffers * bufsize);  /* 分配内存的总大小 */
        if (mem != NULL)
            break;
    }

    if (mem == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    /* 这些缓存是一次性作为一个整体来分配的 */
    memset(&myuvc_queue, 0, sizeof(myuvc_queue));

	/* 初始化两个队列 */
	INIT_LIST_HEAD(&myuvc_queue.mainqueue);
	INIT_LIST_HEAD(&myuvc_queue.irqqueue);

    for (i = 0; i < nbuffers; ++i) {
        myuvc_queue.buffer[i].buf.index = i;
        myuvc_queue.buffer[i].buf.m.offset = i * bufsize;
        myuvc_queue.buffer[i].buf.length = myuvc_format.fmt.pix.sizeimage;
        myuvc_queue.buffer[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        myuvc_queue.buffer[i].buf.sequence = 0;
        myuvc_queue.buffer[i].buf.field = V4L2_FIELD_NONE;
        myuvc_queue.buffer[i].buf.memory = V4L2_MEMORY_MMAP;
        myuvc_queue.buffer[i].buf.flags = 0;
        myuvc_queue.buffer[i].state     = VIDEOBUF_IDLE;
        init_waitqueue_head(&myuvc_queue.buffer[i].wait);
    }

    myuvc_queue.mem = mem;
    myuvc_queue.count = nbuffers;
    myuvc_queue.buf_size = bufsize;
    ret = nbuffers;

done:
    return ret;

}

static void myuvc_vm_open(struct vm_area_struct *vma)
{
  struct myuvc_buffer *buffer = vma->vm_private_data;
  buffer->vma_use_count++;
}

static void myuvc_vm_close(struct vm_area_struct *vma)
{
  struct myuvc_buffer *buffer = vma->vm_private_data;
  buffer->vma_use_count--;
}

static struct vm_operations_struct myuvc_vm_ops = {
  .open 	  = myuvc_vm_open,
  .close	  = myuvc_vm_close,
};


/* S8 映射内存mmap */
static int myuvc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct myuvc_buffer *buffer;
    struct page *page;
    unsigned long addr, start, size;
    unsigned int i;
    int ret = 0;

    start = vma->vm_start;
    size = vma->vm_end - vma->vm_start;

    /* 应用程序调用mmap函数时, 会传入offset参数
     * 根据这个offset找出指定的缓冲区
     */
    for (i = 0; i < myuvc_queue.count; ++i) {
        buffer = &myuvc_queue.buffer[i];
        if ((buffer->buf.m.offset >> PAGE_SHIFT) == vma->vm_pgoff)
            break;
    }

    if (i == myuvc_queue.count || size != myuvc_queue.buf_size) {
        ret = -EINVAL;
        goto done;
    }

    /*
     * VM_IO marks the area as being an mmaped region for I/O to a
     * device. It also prevents the region from being core dumped.
     */
    vma->vm_flags |= VM_IO;

    /* 根据虚拟地址找到缓冲区对应的page构体 */
    addr = (unsigned long)myuvc_queue.mem + buffer->buf.m.offset;
    while (size > 0) {
        page = vmalloc_to_page((void *)addr);

        /* 把page和APP传入的虚拟地址挂构 */
        if ((ret = vm_insert_page(vma, start, page)) < 0)
            goto done;

        start += PAGE_SIZE;
        addr += PAGE_SIZE;
        size -= PAGE_SIZE;
    }

    vma->vm_ops = &myuvc_vm_ops;
    vma->vm_private_data = buffer;
    myuvc_vm_open(vma);

done:
    return ret;
}


/* S9 查询缓冲区 */
static int myuvc_vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *v4l2_buf)
{
	int ret = 0;
    
	if (v4l2_buf->index >= myuvc_queue.count) {
		ret = -EINVAL;
		goto done;
	}

    memcpy(v4l2_buf, &myuvc_queue.buffer[v4l2_buf->index].buf, sizeof(*v4l2_buf));

    /* 更新flags */
	if (myuvc_queue.buffer[v4l2_buf->index].vma_use_count)
		v4l2_buf->flags |= V4L2_BUF_FLAG_MAPPED;


	switch (myuvc_queue.buffer[v4l2_buf->index].state) {
    	case VIDEOBUF_ERROR:
    	case VIDEOBUF_DONE:
    		v4l2_buf->flags |= V4L2_BUF_FLAG_DONE;
    		break;
    	case VIDEOBUF_QUEUED:
    	case VIDEOBUF_ACTIVE:
    		v4l2_buf->flags |= V4L2_BUF_FLAG_QUEUED;
    		break;
    	case VIDEOBUF_IDLE:
    	default:
    		break;
	}

done:    
	return ret;

}

/* S10 把缓冲区放入队列, 并打开IO */
static int myuvc_vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *v4l2_buf)
{
	 struct myuvc_buffer *buf;

    /* 0. APP传入的v4l2_buf可能有问题, 要做判断 */

	if (v4l2_buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    v4l2_buf->memory != V4L2_MEMORY_MMAP) {
		return -EINVAL;
	}

	if (v4l2_buf->index >= myuvc_queue.count) {
		return -EINVAL;
	}

    buf = &myuvc_queue.buffer[v4l2_buf->index];

	if (buf->state != VIDEOBUF_IDLE) {
		return -EINVAL;
	}

    /* 1. 修改状态 */
	buf->state = VIDEOBUF_QUEUED;
	buf->buf.bytesused = 0;

    /* 2. 放入2个队列 */
    /* 队列1: 供APP使用 
     * 当缓冲区没有数据时,放入mainqueue队列
     * 当缓冲区有数据时, APP从mainqueue队列中取出
     */
	list_add_tail(&buf->stream, &myuvc_queue.mainqueue);

    /* 队列2: 供产生数据的函数使用
     * 当采集到数据时,从irqqueue队列中取出第1个缓冲区,存入数据
     */
	list_add_tail(&buf->irq, &myuvc_queue.irqqueue);
    
	return 0;
}

static int myuvc_try_streaming_params(struct myuvc_streaming_control *ctrl)
{
	__u8 *data;
    __u16 size;
    int ret;
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
    
	memset(ctrl, 0, sizeof *ctrl);
    
	ctrl->bmHint = 1;	/* dwFrameInterval */
	ctrl->bFormatIndex = 1;
	ctrl->bFrameIndex  = frame_idx + 1;
	ctrl->dwFrameInterval = 333333;

    size = myuvc_version >= 0x0110 ? 34 : 26;
    data = kzalloc(size, GFP_KERNEL);
    if (data == NULL)
        return -ENOMEM;

    *(__le16 *)&data[0] = cpu_to_le16(ctrl->bmHint);
    data[2] = ctrl->bFormatIndex;
    data[3] = ctrl->bFrameIndex;
    *(__le32 *)&data[4] = cpu_to_le32(ctrl->dwFrameInterval);
    *(__le16 *)&data[8] = cpu_to_le16(ctrl->wKeyFrameRate);
    *(__le16 *)&data[10] = cpu_to_le16(ctrl->wPFrameRate);
    *(__le16 *)&data[12] = cpu_to_le16(ctrl->wCompQuality);
    *(__le16 *)&data[14] = cpu_to_le16(ctrl->wCompWindowSize);
    *(__le16 *)&data[16] = cpu_to_le16(ctrl->wDelay);
    put_unaligned_le32(ctrl->dwMaxVideoFrameSize, &data[18]);
    put_unaligned_le32(ctrl->dwMaxPayloadTransferSize, &data[22]);

    if (size == 34) {
        put_unaligned_le32(ctrl->dwClockFrequency, &data[26]);
        data[30] = ctrl->bmFramingInfo;
        data[31] = ctrl->bPreferedVersion;
        data[32] = ctrl->bMinVersion;
        data[33] = ctrl->bMaxVersion;
    }

    pipe = (GET_CUR & 0x80) ? usb_rcvctrlpipe(mydev, 0)
                  : usb_sndctrlpipe(mydev, 0);
    type |= (GET_CUR & 0x80) ? USB_DIR_IN : USB_DIR_OUT;

    ret = usb_control_msg(mydev, pipe, GET_CUR, type, 0x01 << 8,
            0 << 8 | myuvc_streaming_intf, data, size, 5000);

    kfree(data);
    
    return (ret < 0) ? ret : 0;
    
}

static int myuvc_get_streaming_params(struct myuvc_streaming_control *ctrl)
{
	__u8 *data;
	__u16 size;
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
	int ret;

	size = myuvc_version >= 0x0110 ? 34 : 26;
	data = kmalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	pipe  = (GET_CUR & 0x80) ? usb_rcvctrlpipe(mydev, 0)
			      : usb_sndctrlpipe(mydev, 0);
	type |= (GET_CUR & 0x80) ? USB_DIR_IN : USB_DIR_OUT;

	ret = usb_control_msg(mydev, pipe, GET_CUR, type, 0x01 << 8,
			0 << 8 | myuvc_streaming_intf, data, size, 5000);
	if(ret < 0)
		goto done;

	ctrl->bmHint = le16_to_cpup((__le16 *)&data[0]);
	ctrl->bFormatIndex = data[2];
	ctrl->bFrameIndex = data[3];
	ctrl->dwFrameInterval = le32_to_cpup((__le32 *)&data[4]);
	ctrl->wKeyFrameRate = le16_to_cpup((__le16 *)&data[8]);
	ctrl->wPFrameRate = le16_to_cpup((__le16 *)&data[10]);
	ctrl->wCompQuality = le16_to_cpup((__le16 *)&data[12]);
	ctrl->wCompWindowSize = le16_to_cpup((__le16 *)&data[14]);
	ctrl->wDelay = le16_to_cpup((__le16 *)&data[16]);
	ctrl->dwMaxVideoFrameSize = get_unaligned_le32(&data[18]);
	ctrl->dwMaxPayloadTransferSize = get_unaligned_le32(&data[22]);

	if (size == 34) {
		ctrl->dwClockFrequency = get_unaligned_le32(&data[26]);
		ctrl->bmFramingInfo = data[30];
		ctrl->bPreferedVersion = data[31];
		ctrl->bMinVersion = data[32];
		ctrl->bMaxVersion = data[33];
	} else {
		//ctrl->dwClockFrequency = stream->dev->clock_frequency;
		ctrl->bmFramingInfo = 0;
		ctrl->bPreferedVersion = 0;
		ctrl->bMinVersion = 0;
		ctrl->bMaxVersion = 0;
	}

done:
	kfree(data);

	return (ret < 0) ? ret : 0;
}

static int myuvc_set_streaming_params(struct myuvc_streaming_control *ctrl)
{
	__u8 *data;
	__u16 size;
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
	int ret;

	size = myuvc_version >= 0x0110 ? 34 : 26;
	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	*(__le16 *)&data[0] = cpu_to_le16(ctrl->bmHint);
	data[2] = ctrl->bFormatIndex;
	data[3] = ctrl->bFrameIndex;
	*(__le32 *)&data[4] = cpu_to_le32(ctrl->dwFrameInterval);
	*(__le16 *)&data[8] = cpu_to_le16(ctrl->wKeyFrameRate);
	*(__le16 *)&data[10] = cpu_to_le16(ctrl->wPFrameRate);
	*(__le16 *)&data[12] = cpu_to_le16(ctrl->wCompQuality);
	*(__le16 *)&data[14] = cpu_to_le16(ctrl->wCompWindowSize);
	*(__le16 *)&data[16] = cpu_to_le16(ctrl->wDelay);
	put_unaligned_le32(ctrl->dwMaxVideoFrameSize, &data[18]);
	put_unaligned_le32(ctrl->dwMaxPayloadTransferSize, &data[22]);

	if (size == 34) {
		put_unaligned_le32(ctrl->dwClockFrequency, &data[26]);
		data[30] = ctrl->bmFramingInfo;
		data[31] = ctrl->bPreferedVersion;
		data[32] = ctrl->bMinVersion;
		data[33] = ctrl->bMaxVersion;
	}

	pipe  = (GET_CUR & 0x80) ? usb_rcvctrlpipe(mydev, 0)
			      : usb_sndctrlpipe(mydev, 0);
	type |= (GET_CUR & 0x80) ? USB_DIR_IN : USB_DIR_OUT;

	ret = usb_control_msg(mydev, pipe, GET_CUR, type, 0x01 << 8,
			0 << 8 | myuvc_streaming_intf, data, size, 5000);

	kfree(data);

	return (ret < 0) ? ret : 0;
}

static void myuvc_print_streaming_params(struct myuvc_streaming_control *ctrl)
{
    printk("video params:\n");
    printk("bmHint                   = %d\n", ctrl->bmHint);
    printk("bFormatIndex             = %d\n", ctrl->bFormatIndex);
    printk("bFrameIndex              = %d\n", ctrl->bFrameIndex);
    printk("dwFrameInterval          = %d\n", ctrl->dwFrameInterval);
    printk("wKeyFrameRate            = %d\n", ctrl->wKeyFrameRate);
    printk("wPFrameRate              = %d\n", ctrl->wPFrameRate);
    printk("wCompQuality             = %d\n", ctrl->wCompQuality);
    printk("wCompWindowSize          = %d\n", ctrl->wCompWindowSize);
    printk("wDelay                   = %d\n", ctrl->wDelay);
    printk("dwMaxVideoFrameSize      = %d\n", ctrl->dwMaxVideoFrameSize);
    printk("dwMaxPayloadTransferSize = %d\n", ctrl->dwMaxPayloadTransferSize);
    printk("dwClockFrequency         = %d\n", ctrl->dwClockFrequency);
    printk("bmFramingInfo            = %d\n", ctrl->bmFramingInfo);
    printk("bPreferedVersion         = %d\n", ctrl->bPreferedVersion);
    printk("bMinVersion              = %d\n", ctrl->bMinVersion);
    printk("bMinVersion              = %d\n", ctrl->bMinVersion);
}


static int myuvc_uninit_video(void)
{
	int i;
	for(i = 0; i < UVC_URBS; ++i)
	{
		usb_buffer_free(mydev, myuvc_queue.urb_size,
				myuvc_queue.urb_buffer[i], myuvc_queue.urb_dma[i]);
		myuvc_queue.urb_buffer[i] = NULL;

		usb_free_urb(myuvc_queue.urb[i]);
		myuvc_queue.urb[i] = NULL;
	}

	return 0;
}


static void myuvc_video_complete(struct urb *urb)
{
	u8 *src;
    u8 *dest;
	int ret, i;
    int len;
    int maxlen;
    int nbytes;
    struct myuvc_buffer *buf;
	static int fid;

    // 要修改影像資料，必須先宣告一個特別型態的指標變數，才能正確存取記憶體中的資料
    unsigned char *point_mem;
    static unsigned char *mem_temp = NULL;

    // 初始化暫存用的記憶體位置
    static unsigned int nArrayTemp_Size = 1000;

	u8* mem;
	int data_len;
    
	switch (urb->status) {
	case 0:
		break;

	default:
		printk("Non-zero status (%d) in video "
			"completion handler.\n", urb->status);
		return;
	}

    /* 从irqqueue队列中取出第1个缓冲区 */
	if (!list_empty(&myuvc_queue.irqqueue))
	{
		buf = list_first_entry(&myuvc_queue.irqqueue, struct myuvc_buffer, irq);
	}
	else
	{
		buf = NULL;
	}
    

	for (i = 0; i < urb->number_of_packets; ++i) {
		if (urb->iso_frame_desc[i].status < 0) {
			//printk("USB isochronous frame "
			//	"lost (%d).\n", urb->iso_frame_desc[i].status);
			continue;
		}

        src  = urb->transfer_buffer + urb->iso_frame_desc[i].offset;


        len = urb->iso_frame_desc[i].actual_length;
        /* 判断数据是否有效 */
        /* URB数据含义:
         * data[0] : 头部长度
         * data[1] : 错误状态
         */
        if (len < 2 || src[0] < 2 || src[0] > len)
            continue;
        
        /* Skip payloads marked with the error bit ("error frames"). */
        if (src[1] & UVC_STREAM_ERR) {
            //printk("Dropping payload (error bit set).\n");
            continue;
        }

	    /* ip2970/ip2977 */
	    if (mydev->descriptor.idVendor == 0x1B3B)
	    {
	        if ( len >= 16 ) // have data in buffer
	        {
	            // 資料必須從data[12]開始判斷，是因為前面的資料是封包專用
	            if ( (src[12]==0xFF && src[13]==0xD8 && src[14]==0xFF) ||
	                (src[12]==0xD8 && src[13]==0xFF && src[14]==0xC4)) 
	            {
	                if(last_fid)
	                    fid &= ~UVC_STREAM_FID;
	                else
	                    fid |= UVC_STREAM_FID;
	            }
	        }
	    }
	    else
	    {
	    	fid = src[1] & UVC_STREAM_FID;
	    }

		/* Store the payload FID bit and return immediately when the buffer is
		 * NULL.
		 */
		if (buf == NULL) {
			last_fid = fid;
			continue;
		}

		/* 根据FID判断当前帧的数据是否结束 */
		if (buf->state != VIDEOBUF_ACTIVE) {   /* != VIDEOBUF_ACTIVE, 表示"之前还未接收数据" */			
			if (fid == last_fid) {
				/* 既然你刚开始接收数据, 那么FID应该是一个新的值,不应该等于原来的last_fid */
				continue;
			}

			/* 表示开始接收第1个数据 */
			buf->state = VIDEOBUF_ACTIVE;
		}

		/* fid != last_fid 表示开始新一帧了 */
		if (fid != last_fid && buf->buf.bytesused != 0) {
			buf->state = VIDEOBUF_DONE;

			/* 从队列中删除, 唤醒进程 */
	        list_del(&buf->irq);
	        wake_up(&buf->wait);

			mem = myuvc_queue.mem + buf->buf.m.offset;
			data_len = buf->buf.bytesused;
			//printk("wake_up %d : %s %d, start data: %02x %02x, end data: %02x %02x", wake_cnt++, __FUNCTION__, __LINE__, mem[0], mem[1], mem[data_len - 2], mem[data_len - 1]);

			/* 取出下一个buf */
			if (!list_empty(&myuvc_queue.irqqueue))
			{
				buf = list_first_entry(&myuvc_queue.irqqueue, struct myuvc_buffer, irq);
			}
			else
			{
				buf = NULL;
			}
			continue;
		}
		last_fid = fid;

        dest = myuvc_queue.mem + buf->buf.m.offset + buf->buf.bytesused;

        /* 除去头部后的数据长度 */
        len -= src[0];

        /* 缓冲区最多还能存多少数据 */
        maxlen = buf->buf.length - buf->buf.bytesused;
        nbytes = min(len, maxlen);

        /* 复制数据 */
        memcpy(dest, src + src[0], nbytes);
        buf->buf.bytesused += nbytes;

	    /* ip2970/ip2977 */
	    if (mydev->descriptor.idVendor == 0x1B3B)
	    {
	        if(mem_temp == NULL) {
	            mem_temp = kmalloc(nArrayTemp_Size, GFP_KERNEL);
	        }
	        else if(nArrayTemp_Size <= nbytes){ // 當收到的資料長度大於上一次的資料長度，則重新分配所需的空間+
	            kfree(mem_temp);
	            nArrayTemp_Size += 500;
	            mem_temp = kmalloc(nArrayTemp_Size, GFP_KERNEL);
	        }
	        memset(mem_temp, 0x00, nArrayTemp_Size);
	        
	        // 指向資料儲存的記憶體位置
	        point_mem = (unsigned char *)dest;
	        if( *(point_mem) == 0xD8 && *(point_mem + 1) == 0xFF && *(point_mem + 2) == 0xC4){
	            memcpy( mem_temp + 1, point_mem, nbytes);
	            mem_temp[0] = 0xFF;
	            memcpy(point_mem, mem_temp, nbytes + 1);
	        }
	    }



        /* 判断一帧数据是否已经全部接收到 */
        if (len > maxlen) {
            buf->state = VIDEOBUF_DONE;
        }
        
        /* Mark the buffer as done if the EOF marker is set. */
        if (src[1] & UVC_STREAM_EOF && buf->buf.bytesused != 0) {
           // printk("Frame complete (EOF found).\n");
            //if (len == 0)
           //     printk("EOF in empty payload.\n");
            buf->state = VIDEOBUF_DONE;
        }

	    /* 当接收完一帧数据, 
	     * 从irqqueue中删除这个缓冲区
	     * 唤醒等待数据的进程 
	     */
	    if (buf->state == VIDEOBUF_DONE ||
	        buf->state == VIDEOBUF_ERROR)
	    {
	        list_del(&buf->irq);
	        wake_up(&buf->wait);

			mem = myuvc_queue.mem + buf->buf.m.offset;
			data_len = buf->buf.bytesused;
			//printk("wake_up %d : %s %d, start data: %02x %02x, end data: %02x %02x", wake_cnt++, __FUNCTION__, __LINE__, mem[0], mem[1], mem[data_len - 2], mem[data_len - 1]);
			
			/* 取出下一个buf */
			if (!list_empty(&myuvc_queue.irqqueue))
			{
				buf = list_first_entry(&myuvc_queue.irqqueue, struct myuvc_buffer, irq);
			}
			else
			{
				buf = NULL;
			}
	    }

	}


    /* 再次提交URB */
	if ((ret = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		printk("Failed to resubmit video URB (%d).\n", ret);
	}
}


static int myuvc_init_urb(void)
{
	struct urb *urb;
	unsigned int npackets, i, j;
	u16 psize;
	u32 size;

	psize    = wMaxPacketSize;
	size     = myuvc_streaming_ctl.dwMaxVideoFrameSize;
	npackets = DIV_ROUND_UP(size, psize);
	if(npackets >= 32)
		npackets = 32;

	myuvc_queue.urb_size = psize * npackets;

	for(i = 0; i < UVC_URBS; ++i)
	{
		/* 分配urb_buffer */
		myuvc_queue.urb_buffer[i] = usb_buffer_alloc(
				mydev, myuvc_queue.urb_size,
				GFP_KERNEL | __GFP_NOWARN, &myuvc_queue.urb_dma[i]);;

		/* 分配urb */
		myuvc_queue.urb[i] = usb_alloc_urb(npackets, GFP_KERNEL);
		if (!myuvc_queue.urb[i] && !myuvc_queue.urb_buffer[i]) {
			myuvc_uninit_video();
			return -ENOMEM;
		}

		/* 设置urb */
		urb = myuvc_queue.urb[i];
		
		urb->dev     = mydev;
		urb->context = NULL;
		urb->pipe    = usb_rcvisocpipe(mydev, myuvc_bEndpointAddress);
		urb->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma   = myuvc_queue.urb_dma[i];
		urb->interval = myuvc_bInterval;
		urb->transfer_buffer = myuvc_queue.urb_buffer[i];
		urb->complete = myuvc_video_complete;
		urb->number_of_packets = npackets;
		urb->transfer_buffer_length = size;
				
		for (j = 0; j < npackets; ++j) {
			urb->iso_frame_desc[j].offset = j * psize;
			urb->iso_frame_desc[j].length = psize;
		}
	}
	
	return 0;
}

/* 启动传输 
 * 参考 uvc_init_video
 */
static int myuvc_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	int ret;
	/* 1. 向USB摄像头设置参数 比如使用哪个format, 使用这个format下的哪个frame(分辨率)*/
	/* 参考：uvc_set_video_ctrl	
     * 1.1 取出参数
     * 1.2 设置参数
	 */
	ret = myuvc_try_streaming_params(&myuvc_streaming_ctl);
    printk("myuvc_try_streaming_params ret = %d\n", ret);
	
	ret = myuvc_get_streaming_params(&myuvc_streaming_ctl);
    printk("myuvc_get_streaming_params ret = %d\n", ret);

    ret = myuvc_set_streaming_params(&myuvc_streaming_ctl);
    printk("myuvc_set_streaming_params ret = %d\n", ret);

	myuvc_print_streaming_params(&myuvc_streaming_ctl);

	/* d. 设置VideoStreaming Interface所使用的setting
     * d.1 从myuvc_params确定带宽
     * d.2 根据setting的endpoint能传输的wMaxPacketSize
     *     找到能满足该带宽的setting
     */
    /* 手工确定:
     * bandwidth = myuvc_params.dwMaxPayloadTransferSize = 800
     * 观察lsusb -v -d 0x1e4e:的结果:
     *                wMaxPacketSize     0x0320  1x 800 bytes
     * bAlternateSetting       5
     */
    usb_set_interface(mydev, myuvc_streaming_intf, myuvc_streaming_bAlternateSetting);
	
    /* 2. 分配设置URB */
	if ((ret = myuvc_init_urb()) < 0)
		return ret;
	
	
    /* 3. 提交URB以接收数据 */
	for (i = 0; i < UVC_URBS; ++i) {
		if ((ret = usb_submit_urb(myuvc_queue.urb[i], GFP_KERNEL)) < 0) {
			printk("Failed to submit URB %u (%d).\n", i, ret);
			myuvc_uninit_video();
			return ret;
		}
	}

	return 0;
}

/* S11 调用poll监听io */
static unsigned int myuvc_poll(struct file *file, struct poll_table_struct *wait)
{
	struct myuvc_buffer *buf;
	unsigned int mask = 0;
    
    /* 从mainqueuq中取出第1个缓冲区 */

    /*判断它的状态, 如果未就绪, 休眠 */

    if (list_empty(&myuvc_queue.mainqueue)) {
        mask |= POLLERR;
        goto done;
    }
    
    buf = list_first_entry(&myuvc_queue.mainqueue, struct myuvc_buffer, stream);

    poll_wait(file, &buf->wait, wait);
    if (buf->state == VIDEOBUF_DONE ||
        buf->state == VIDEOBUF_ERROR)
        mask |= POLLIN | POLLRDNORM;
    
done:
    return mask;

}

/* S12 如果有数据, 从队列中取出数据 
 *     把缓冲区放入队列, 调用poll...
 */
static int myuvc_vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *v4l2_buf)
{
	/* APP发现数据就绪后, 从mainqueue里取出这个buffer */
    struct myuvc_buffer *buf;
    int ret = 0;

	if (list_empty(&myuvc_queue.mainqueue)) {
		ret = -EINVAL;
		goto done;
	}
    
	buf = list_first_entry(&myuvc_queue.mainqueue, struct myuvc_buffer, stream);

	switch (buf->state) {
	case VIDEOBUF_ERROR:
		ret = -EIO;
	case VIDEOBUF_DONE:
		buf->state = VIDEOBUF_IDLE;
		break;

	case VIDEOBUF_IDLE:
	case VIDEOBUF_QUEUED:
	case VIDEOBUF_ACTIVE:
	default:
		ret = -EINVAL;
		goto done;
	}

	list_del(&buf->stream);
	memcpy(v4l2_buf, &buf->buf, sizeof *v4l2_buf);

done:
	return ret;

}

/* S13 关闭io, 关闭文件    */
static int myuvc_vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type p)
{
	struct urb *urb;
	unsigned int i;

    /* 1. kill URB */
	for (i = 0; i < UVC_URBS; ++i) {
		if ((urb = myuvc_queue.urb[i]) == NULL)
			continue;
		usb_kill_urb(urb);
	}

    /* 2. free URB */
    myuvc_uninit_video();

    /* 3. 设置VideoStreaming Interface为setting 0 */
    usb_set_interface(mydev, myuvc_streaming_intf, 0);
    
    return 0;
}

static int myuvc_close(struct file *file)
{
	//myuvc_vidioc_streamoff(NULL, NULL, 0);
	
	return 0;
}


/* Extract the bit string specified by mapping->offset and mapping->size
 * from the little-endian data stored at 'data' and return the result as
 * a signed 32bit integer. Sign extension will be performed if the mapping
 * references a signed data type.
 */
static __s32 myuvc_get_le_value(const __u8 *data)
{
    int bits = 16;
    int offset = 0;
    __s32 value = 0;
    __u8 mask;

    data += offset / 8;
    offset &= 7;
    mask = ((1LL << bits) - 1) << offset;

    for (; bits > 0; data++) {
        __u8 byte = *data & mask;
        value |= offset > 0 ? (byte >> offset) : (byte << (-offset));
        bits -= 8 - (offset > 0 ? offset : 0);
        offset -= 8;
        mask = (1 << bits) - 1;
    }

    /* Sign-extend the value if needed. */
    value |= -(value & (1 << (16 - 1)));

    return value;
}

/* Set the bit string specified by mapping->offset and mapping->size
 * in the little-endian data stored at 'data' to the value 'value'.
 */
static void myuvc_set_le_value(__s32 value, __u8 *data)
{
	int bits = 16;
	int offset = 0;
	__u8 mask;

	data += offset / 8;
	offset &= 7;

	for (; bits > 0; data++) {
		mask = ((1LL << bits) - 1) << offset;
		*data = (*data & ~mask) | ((value << offset) & mask);
		value >>= offset ? offset : 8;
		bits -= 8 - offset;
		offset = 0;
	}
}
    


static int myuvc_query_v4l2_ctrl (struct file *file, void *fh,
                struct v4l2_queryctrl *ctrl)
{
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
    int ret;
    u8 data[2];

    if (ctrl->id != V4L2_CID_BRIGHTNESS)
        return -EINVAL;
    
	memset(ctrl, 0, sizeof *ctrl);
	ctrl->id   = V4L2_CID_BRIGHTNESS;
	ctrl->type = V4L2_CTRL_TYPE_INTEGER;
	strcpy(ctrl->name, "MyUVC_BRIGHTNESS");
	ctrl->flags = 0;

	//printk("%s %d \n", __FUNCTION__, __LINE__);
	pipe = usb_rcvctrlpipe(mydev, 0);
	type |= USB_DIR_IN;

	//printk("%s %d \n", __FUNCTION__, __LINE__);
    /* 发起USB传输确定这些值 */
	ret = usb_control_msg(mydev, pipe, GET_MIN, type, PU_BRIGHTNESS_CONTROL << 8,
			ProcessingUnitID << 8 | myuvc_control_intf, data, 2, 5000);
    if (ret != 2)
        return -EIO;
	ctrl->minimum = myuvc_get_le_value(data);	/* Note signedness */
	//printk("%s %d \n", __FUNCTION__, __LINE__);

	ret = usb_control_msg(mydev, pipe, GET_MAX, type,  PU_BRIGHTNESS_CONTROL << 8,
			ProcessingUnitID << 8 | myuvc_control_intf, data, 2, 5000);
    if (ret != 2)
        return -EIO;
	ctrl->maximum = myuvc_get_le_value(data);	/* Note signedness */
	//printk("%s %d \n", __FUNCTION__, __LINE__);

	ret = usb_control_msg(mydev, pipe, GET_RES, type, PU_BRIGHTNESS_CONTROL << 8,
			 ProcessingUnitID << 8 | myuvc_control_intf, data, 2, 5000);
    if (ret != 2)
        return -EIO;
	ctrl->step = myuvc_get_le_value(data);	/* Note signedness */

	ret = usb_control_msg(mydev, pipe, GET_DEF, type, PU_BRIGHTNESS_CONTROL << 8,
			ProcessingUnitID << 8 | myuvc_control_intf, data, 2, 5000);
    if (ret != 2)
        return -EIO;
	ctrl->default_value = myuvc_get_le_value(data);	/* Note signedness */
//	printk("%s %d \n", __FUNCTION__, __LINE__);

    printk("Brightness: min =%d, max = %d, step = %d, default = %d\n", ctrl->minimum, ctrl->maximum, ctrl->step, ctrl->default_value);
    
    return 0;
}


static 	int myuvc_ctrl_set (struct file *file, void *fh,
                struct v4l2_control *ctrl)
{
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
    unsigned int pipe;
    int ret;
    u8 data[2];
    
    if (ctrl->id != V4L2_CID_BRIGHTNESS)
        return -EINVAL;

    myuvc_set_le_value(ctrl->value, data);

    pipe = usb_sndctrlpipe(mydev, 0);
    type |= USB_DIR_OUT;

    ret = usb_control_msg(mydev, pipe, SET_CUR, type, PU_BRIGHTNESS_CONTROL << 8,
            ProcessingUnitID  << 8 | myuvc_control_intf, data, 2, 5000);
    if (ret != 2)
        return -EIO;
	return 0;
}

static  int myuvc_ctrl_get (struct file *file, void *fh,
                struct v4l2_control *ctrl)
{
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
    int ret;
    u8 data[2];
    
    if (ctrl->id != V4L2_CID_BRIGHTNESS)
        return -EINVAL;

	pipe = usb_rcvctrlpipe(mydev, 0);
	type |= USB_DIR_IN;

	ret = usb_control_msg(mydev, pipe, GET_CUR, type, PU_BRIGHTNESS_CONTROL << 8,
			ProcessingUnitID << 8 | myuvc_control_intf, data, 2, 5000);
    if (ret != 2)
        return -EIO;
	ctrl->value = myuvc_get_le_value(data);	/* Note signedness */
    
    return 0;

}


static const struct v4l2_ioctl_ops myuvc_ioctl_ops = {
        // 表示它是一个摄像头设备
        .vidioc_querycap      = myuvc_vidioc_querycap,

        /* 用于列举、获得、测试、设置摄像头的数据的格式 */
        .vidioc_enum_fmt_vid_cap  = myuvc_vidioc_enum_fmt_vid_cap,
        .vidioc_g_fmt_vid_cap     = myuvc_vidioc_g_fmt_vid_cap,
        .vidioc_try_fmt_vid_cap   = myuvc_vidioc_try_fmt_vid_cap,
        .vidioc_s_fmt_vid_cap     = myuvc_vidioc_s_fmt_vid_cap,
        
        /* 缓冲区操作: 申请/查询/放入队列/取出队列 */
        .vidioc_reqbufs       = myuvc_vidioc_reqbufs,
        .vidioc_querybuf      = myuvc_vidioc_querybuf,
        .vidioc_qbuf          = myuvc_vidioc_qbuf,
        .vidioc_dqbuf         = myuvc_vidioc_dqbuf,

        /* 属性设置 */
		.vidioc_queryctrl	  = myuvc_query_v4l2_ctrl,
		.vidioc_s_ctrl		  = myuvc_ctrl_set,
		.vidioc_g_ctrl		  = myuvc_ctrl_get,

        // 启动/停止
        .vidioc_streamon      = myuvc_vidioc_streamon,
        .vidioc_streamoff     = myuvc_vidioc_streamoff,   
};


static const struct v4l2_file_operations myuvc_fops = {
	.owner		= THIS_MODULE,
    .open       = myuvc_open,
    .release    = myuvc_close,
    .mmap       = myuvc_mmap,
    .ioctl      = video_ioctl2, /* V4L2 ioctl handler */
    .poll       = myuvc_poll,
};


static struct video_device *myuvc_device;
static int myuvc_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_interface_descriptor	*interface;
	
	static int cnt = 0;
	
	mydev = udev;
	printk("myuvc_probe : cnt = %d \n", cnt++);

#if 0
	/* print device descriptor */
	printk("Device Descriptor:\n"
	       "  bLength             %5u\n"
	       "  bDescriptorType     %5u\n"
	       "  bcdUSB              %2x.%02x\n"
	       "  bDeviceClass        %5u\n"
	       "  bDeviceSubClass     %5u\n"
	       "  bDeviceProtocol     %5u\n"
	       "  bMaxPacketSize0     %5u\n"
	       "  idVendor           0x%04x\n"
	       "  idProduct          0x%04x\n"
	       "  bcdDevice           %2x.%02x\n"
	       "  iManufacturer       %5u\n"
	       "  iProduct            %5u\n"
	       "  iSerial             %5u\n"
	       "  bNumConfigurations  %5u\n",
	       udev->descriptor.bLength, udev->descriptor.bDescriptorType,
	       udev->descriptor.bcdUSB >> 8, udev->descriptor.bcdUSB & 0xff,
	       udev->descriptor.bDeviceClass,
	       udev->descriptor.bDeviceSubClass,
	       udev->descriptor.bDeviceProtocol,
	       udev->descriptor.bMaxPacketSize0,
	       udev->descriptor.idVendor, udev->descriptor.idProduct,
	       udev->descriptor.bcdDevice >> 8, udev->descriptor.bcdDevice & 0xff,
	       udev->descriptor.iManufacturer,
	       udev->descriptor.iProduct,
	       udev->descriptor.iSerialNumber,
	       udev->descriptor.bNumConfigurations);


	/* print configuration descriptor */
	printk("  Configuration Descriptor:\n"
	       "    bLength             %5u\n"
	       "    bDescriptorType     %5u\n"
	       "    wTotalLength        %5u\n"
	       "    bNumInterfaces      %5u\n"
	       "    bConfigurationValue %5u\n"
	       "    iConfiguration      %5u\n"
	       "    bmAttributes         0x%02x\n",
	       udev->config->desc.bLength, udev->config->desc.bDescriptorType,
	       le16_to_cpu(udev->config->desc.wTotalLength),
	       udev->config->desc.bNumInterfaces, udev->config->desc.bConfigurationValue,
	       udev->config->desc.iConfiguration,
	       udev->config->desc.bmAttributes);

	/* print Interface Association descriptor */
	printk("    Interface Association:\n"
	       "      bLength             %5u\n"
	       "      bDescriptorType     %5u\n"
	       "      bFirstInterface     %5u\n"
	       "      bInterfaceCount     %5u\n"
	       "      bFunctionClass      %5u\n"
	       "      bFunctionSubClass   %5u\n"
	       "      bFunctionProtocol   %5u\n"
	       "      iFunction           %5u\n",
			udev->config->intf_assoc[0]->bLength,
			udev->config->intf_assoc[0]->bDescriptorType,
			udev->config->intf_assoc[0]->bFirstInterface,
			udev->config->intf_assoc[0]->bInterfaceCount,
			udev->config->intf_assoc[0]->bFunctionClass,
			udev->config->intf_assoc[0]->bFunctionSubClass,
			udev->config->intf_assoc[0]->bFunctionProtocol,
			udev->config->intf_assoc[0]->iFunction);

	/* print Interface descriptor */
	 for (j = 0; j < intf->num_altsetting; j++)
    {
        interface = &intf->altsetting[j].desc;
        printk("    Interface Descriptor altsetting %d:\n"
               "      bLength             %5u\n"
               "      bDescriptorType     %5u\n"
               "      bInterfaceNumber    %5u\n"
               "      bAlternateSetting   %5u\n"
               "      bNumEndpoints       %5u\n"
               "      bInterfaceClass     %5u\n"
               "      bInterfaceSubClass  %5u\n"
               "      bInterfaceProtocol  %5u\n"
               "      iInterface          %5u\n",
               j, 
               interface->bLength, interface->bDescriptorType, interface->bInterfaceNumber,
               interface->bAlternateSetting, interface->bNumEndpoints, interface->bInterfaceClass,
               interface->bInterfaceSubClass, interface->bInterfaceProtocol,
               interface->iInterface);
    }

	buffer = intf->cur_altsetting->extra;
    buflen = intf->cur_altsetting->extralen;
    printk("extra buffer of interface %d:\n", cnt-1);
    k = 0;
    desc_cnt = 0;
    while (k < buflen)
    {
        desc_len = buffer[k];
        printk("extra desc %d: ", desc_cnt);
        for (l = 0; l < desc_len; l++, k++)
        {
            printk("%02x ", buffer[k]);
        }
        desc_cnt++;
        printk("\n");
    }

    interface = &intf->cur_altsetting->desc;
    if ((buffer[1] == USB_DT_CS_INTERFACE) && (interface->bInterfaceSubClass == 1))
    {
        parse_videocontrol_interface(intf, buffer, buflen);
    }
    if ((buffer[1] == USB_DT_CS_INTERFACE) && (interface->bInterfaceSubClass == 2))
    {
        parse_videostreaming_interface(intf, buffer, buflen);
    }
#endif

	if (cnt == 1)
    {
        myuvc_control_intf = intf->cur_altsetting->desc.bInterfaceNumber;
    }
    else if (cnt == 2)
    {
        myuvc_streaming_intf = intf->cur_altsetting->desc.bInterfaceNumber;
    }

	if(cnt == 2)
	{
		 /* 1. 分配一个video_device结构体 */
		 myuvc_device =  video_device_alloc();
		 if(myuvc_device == NULL)
		 	return -1;

		/* 2. 设置 */
		/* 2.1 */
		myuvc_device->release   = myuvc_release;

		/* 2.2 */
		myuvc_device->fops      = &myuvc_fops;

		/* 2.3 */
		myuvc_device->ioctl_ops = &myuvc_ioctl_ops;

		/* 3. 注册 */
		video_register_device(myuvc_device, VFL_TYPE_GRABBER, -1);
	}

	return 0;
}

static void myuvc_disconnect(struct usb_interface *intf)
{
	static int cnt = 0;

	printk("myuvc_disconnect : %d\n", cnt++);

	if(cnt == 2)
	{
		video_unregister_device(myuvc_device);
        video_device_release(myuvc_device);
	}
}

static struct usb_device_id myuvc_ids[] = {
	/* Generic USB Video Class */
	{ USB_INTERFACE_INFO(USB_CLASS_VIDEO, 1, 0) },  /* VideoControl Interface */
    { USB_INTERFACE_INFO(USB_CLASS_VIDEO, 2, 0) },  /* VideoStreaming Interface */
	{}
};

static struct usb_driver myuvc_driver = {
	.name		= "myuvcvideo",
	.probe		= myuvc_probe,
	.disconnect	= myuvc_disconnect,
	.id_table	= myuvc_ids,
};

static int __init myuvc_init(void)
{
	int ret;

	ret = usb_register(&myuvc_driver);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static void __exit myuvc_exit(void)
{
	usb_deregister(&myuvc_driver);
}

module_init(myuvc_init);
module_exit(myuvc_exit);

MODULE_LICENSE("GPL");


