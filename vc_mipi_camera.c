#include "vc_mipi_core.h"
#include <linux/module.h>
// #include <linux/gpio/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
// #define ENABLE_PM    // support for power management
#ifdef ENABLE_PM
#include <linux/pm_runtime.h>
#endif
#define ENABLE_RK // support for RK ISP
#ifdef ENABLE_RK
#include <linux/rk-preisp.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>
#include "../../platform/rockchip/isp/rkisp_tb_helper.h"

#endif

#define VERSION "0.3.0"

#define V4L2_CID_CSI_LANES      (V4L2_CID_LASTP1 + 0)
#define V4L2_CID_TRIGGER_MODE   (V4L2_CID_LASTP1 + 1)
#define V4L2_CID_IO_MODE        (V4L2_CID_LASTP1 + 2)
#define V4L2_CID_FRAME_RATE     (V4L2_CID_LASTP1 + 3)
#define V4L2_CID_SINGLE_TRIGGER (V4L2_CID_LASTP1 + 4)
#define V4L2_CID_BINNING_MODE   (V4L2_CID_LASTP1 + 5)
#ifdef ENABLE_ADVANCED_CONTROL
#define V4L2_CID_HMAX_OVERWRITE (V4L2_CID_LASTP1 + 6)
#define V4L2_CID_VMAX_OVERWRITE (V4L2_CID_LASTP1 + 7)
#define V4L2_CID_HEIGHT_OFFSET  (V4L2_CID_LASTP1 + 8)
#endif

#define MIPI_FREQ_891M			891000000
#define MIPI_FREQ_446M			446000000
#define MIPI_FREQ_743M			743000000
#define MIPI_FREQ_297M			297000000

#define IMX415_4LANES			4

#define IMX415_MAX_PIXEL_RATE		(MIPI_FREQ_891M / 10 * 2 * IMX415_4LANES)
/* Basic Readout Lines. Number of necessary readout lines in sensor */
#define BRL_ALL				2228u
#define BRL_BINNING			1115u
struct vc_device {
        unsigned int csi_id;
        struct v4l2_subdev sd;
        struct v4l2_ctrl_handler ctrl_handler;
        struct media_pad pad;
        // struct gpio_desc *power_gpio;
        int power_on;
        struct mutex mutex;

        struct vc_cam cam;
};

// static struct __maybe_unused__ rkmodule_csi_dphy_param dcphy_param = {
//         .vendor = PHY_VENDOR_SAMSUNG,
// 	.lp_vol_ref = 6,
// 	.lp_hys_sw = {3, 0, 0, 0},
// 	.lp_escclk_pol_sel = {1, 1, 1, 1},
// 	.skew_data_cal_clk = {0, 3, 3, 3},
// 	.clk_hs_term_sel = 2,
// 	.data_hs_term_sel = {2, 2, 2, 2},
// 	.reserved = {0},
// };

static inline struct vc_device *to_vc_device(struct v4l2_subdev *sd)
{
        return container_of(sd, struct vc_device, sd);
}

static inline struct vc_cam *to_vc_cam(struct v4l2_subdev *sd)
{
        struct vc_device *device = to_vc_device(sd);
        return &device->cam;
}


// --- v4l2_subdev_core_ops ---------------------------------------------------

static void vc_set_power(struct vc_device *device, int on)
{
        struct device *dev = &device->cam.ctrl.client_sen->dev;

        if (device->power_on == on)
                return;

        vc_dbg(dev, "%s(): Set power: %s\n", __func__, on ? "on" : "off");

        // if (device->power_gpio)
        // 	gpiod_set_value_cansleep(device->power_gpio, on);

        // if (on == 1) {
        //         vc_core_wait_until_device_is_ready(&device->cam, 1000);
        // }
        device->power_on = on;
}

static int vc_sd_s_power(struct v4l2_subdev *sd, int on)
{
        struct vc_device *device = to_vc_device(sd);

        mutex_lock(&device->mutex);

        vc_set_power(to_vc_device(sd), on);

        mutex_unlock(&device->mutex);

        return 0;
}

#ifdef ENABLE_PM
static int __maybe_unused vc_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct vc_device *device = to_vc_device(sd);
        struct vc_state *state = &device->cam.state;

        vc_dbg(dev, "%s()\n", __func__);

        mutex_lock(&device->mutex);

        if (state->streaming)
                vc_sen_stop_stream(&device->cam);

        vc_set_power(device, 0);

        mutex_unlock(&device->mutex);

        return 0;
}

static int __maybe_unused vc_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct vc_device *device = to_vc_device(sd);

        vc_dbg(dev, "%s()\n", __func__);

        mutex_lock(&device->mutex);

        vc_set_power(device, 1);

        mutex_unlock(&device->mutex);

        return 0;
}
#endif

static const s64 ctrl_csi_lanes_menu[] = {
	1, 2, 4
};

static int vc_sd_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *control)
{
        struct vc_cam *cam = to_vc_cam(sd);
        struct device *dev = vc_core_get_sen_device(cam);

        switch (control->id) {
        case V4L2_CID_EXPOSURE:
                return vc_sen_set_exposure(cam, control->value);

        case V4L2_CID_GAIN:
                return vc_sen_set_gain(cam, control->value);
        
        case V4L2_CID_CSI_LANES:
                return vc_core_set_num_lanes(cam, ctrl_csi_lanes_menu[control->value]);

        case V4L2_CID_BLACK_LEVEL:
                return vc_sen_set_blacklevel(cam, control->value);

        case V4L2_CID_TRIGGER_MODE:
                return vc_mod_set_trigger_mode(cam, control->value);

        case V4L2_CID_IO_MODE:
                return vc_mod_set_io_mode(cam, control->value);

        case V4L2_CID_FRAME_RATE:
                return vc_core_set_framerate(cam, control->value);

        case V4L2_CID_SINGLE_TRIGGER:
                return vc_mod_set_single_trigger(cam);

        case V4L2_CID_BINNING_MODE:
                return vc_core_set_binning_mode(cam, control->value);

#ifdef ENABLE_ADVANCED_CONTROL
        case V4L2_CID_HMAX_OVERWRITE:
                return vc_core_set_hmax_overwrite(cam, control->value);

        case V4L2_CID_VMAX_OVERWRITE:
                return vc_core_set_vmax_overwrite(cam, control->value);
        
        case V4L2_CID_HEIGHT_OFFSET:
                return vc_core_set_height_offset(cam, control->value);
#endif

        default:
                vc_warn(dev, "%s(): Unknown control 0x%08x\n", __func__, control->id);
                return -EINVAL;
        }

        return 0;
}
static int vc_sd_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
    struct vc_cam *cam = to_vc_cam(sd);
    struct v4l2_fract interval = {1, cam->ctrl.framerate.max};
     

	fi->interval =  interval; 

	return 0;
}
// --- v4l2_subdev_video_ops ---------------------------------------------------

static int vc_sd_s_stream(struct v4l2_subdev *sd, int enable)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct vc_state *state = &cam->state;
        struct device *dev = sd->dev;
        int reset = 0;
        int ret = 0;

        vc_notice(dev, "%s(): Set streaming: %s\n", __func__, enable ? "on" : "off");

        if (state->streaming == enable)
                return 0;

        mutex_lock(&device->mutex);
        if (enable) {
#ifdef ENABLE_PM
                ret = pm_runtime_get_sync(dev);
                if (ret < 0) {
                        pm_runtime_put_noidle(dev);
                        mutex_unlock(&device->mutex);
                        return ret;
                }
#endif

                ret  = vc_mod_set_mode(cam, &reset);
                ret |= vc_sen_set_roi(cam);
                if (!ret && reset) {
                        ret |= vc_sen_set_exposure(cam, cam->state.exposure);
                        ret |= vc_sen_set_gain(cam, cam->state.gain);
                        ret |= vc_sen_set_blacklevel(cam, cam->state.blacklevel);
                }

                ret = vc_sen_start_stream(cam);
                if (ret) {
                        enable = 0;
                        vc_sen_stop_stream(cam);
#ifdef ENABLE_PM
                        pm_runtime_mark_last_busy(dev);
                        pm_runtime_put_autosuspend(dev);
#endif
                }

        } else {
                vc_sen_stop_stream(cam);
#ifdef ENABLE_PM
                pm_runtime_mark_last_busy(dev);
                pm_runtime_put_autosuspend(dev);
#endif
        }

        state->streaming = enable;
        mutex_unlock(&device->mutex);

        return ret;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
// --- v4l2_subdev_pad_ops ---------------------------------------------------
static int vc_sd_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
	        __u32 mbus_code = 0;

        mutex_lock(&device->mutex);

        mbus_code = vc_core_enum_mbus_code(cam, code->index);
        if (mbus_code == -EINVAL) {
                return -EINVAL;
        }
        code->code = mbus_code;

        mutex_unlock(&device->mutex);

        return 0;
}
static int vc_sd_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	 struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct v4l2_mbus_framefmt *mf = &format->format;
        struct vc_frame* frame = NULL;

        mutex_lock(&device->mutex);

        frame = vc_core_get_frame(cam);
        mf->width       = frame->width;
        mf->height      = frame->height;
        mf->code        = vc_core_get_format(cam);
        mf->colorspace  = V4L2_COLORSPACE_SRGB;
        mf->field       = V4L2_FIELD_NONE;

        mutex_unlock(&device->mutex);

        return 0;
}
static int vc_sd_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct vc_frame *frame = vc_core_get_frame(cam);
        struct v4l2_mbus_framefmt *mf = &format->format;

        mutex_lock(&device->mutex);

        if (mf->code != 0) {
                vc_core_set_format(cam, mf->code);
                vc_core_set_frame(cam, frame->left, frame->top, mf->width, mf->height);
        }

        mutex_unlock(&device->mutex);

        return 0;
}
int vc_sd_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_selection *sel)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct vc_frame *frame = vc_core_get_frame(cam);
        struct vc_frame *frame_bounds = &cam->ctrl.frame;

        mutex_lock(&device->mutex);

        switch (sel->target) {
        case V4L2_SEL_TGT_CROP:
                sel->r.left = frame->left;
                sel->r.top = frame->top;
                sel->r.width = frame->width;
                sel->r.height = frame->height;
                break;
        case V4L2_SEL_TGT_CROP_DEFAULT:
        case V4L2_SEL_TGT_CROP_BOUNDS:
                sel->r.left = frame_bounds->left;
                sel->r.top = frame_bounds->top;
                sel->r.width = frame_bounds->width;
                sel->r.height = frame_bounds->height;
                break;
        }

        mutex_unlock(&device->mutex);

        return 0;
}
int vc_sd_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_selection *sel)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);

        mutex_lock(&device->mutex);

        switch (sel->target) {
        case V4L2_SEL_TGT_CROP:
                vc_core_set_frame(cam, sel->r.left, sel->r.top, sel->r.width, sel->r.height);
                break;
        }

        mutex_unlock(&device->mutex);

        return 0;
}
#else
int vc_sd_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_mbus_code_enum *code)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        __u32 mbus_code = 0;

        mutex_lock(&device->mutex);

        mbus_code = vc_core_enum_mbus_code(cam, code->index);
        if (mbus_code == -EINVAL) {
                return -EINVAL;
        }
        code->code = mbus_code;

        mutex_unlock(&device->mutex);

        return 0;
}
static int vc_sd_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct v4l2_mbus_framefmt *mf = &format->format;
        struct vc_frame* frame = NULL;

        mutex_lock(&device->mutex);

        frame = vc_core_get_frame(cam);
        mf->width       = frame->width;
        mf->height      = frame->height;
        mf->code        = vc_core_get_format(cam);
        mf->colorspace  = V4L2_COLORSPACE_SRGB;
        mf->field       = V4L2_FIELD_NONE;

        mutex_unlock(&device->mutex);

     
        return 0;
}
static int vc_sd_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct vc_frame *frame = vc_core_get_frame(cam);
        struct v4l2_mbus_framefmt *mf = &format->format;

        mutex_lock(&device->mutex);

        if (mf->code != 0) {
                vc_core_set_format(cam, mf->code);
                vc_core_set_frame(cam, frame->left, frame->top, mf->width, mf->height);
        }

        mutex_unlock(&device->mutex);

        return 0;
}
int vc_sd_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_selection *sel)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct vc_frame *frame = vc_core_get_frame(cam);
        struct vc_frame *frame_bounds = &cam->ctrl.frame;

        mutex_lock(&device->mutex);

        switch (sel->target) {
        case V4L2_SEL_TGT_CROP:
                sel->r.left = frame->left;
                sel->r.top = frame->top;
                sel->r.width = frame->width;
                sel->r.height = frame->height;
                break;
        case V4L2_SEL_TGT_CROP_DEFAULT:
        case V4L2_SEL_TGT_CROP_BOUNDS:
                sel->r.left = frame_bounds->left;
                sel->r.top = frame_bounds->top;
                sel->r.width = frame_bounds->width;
                sel->r.height = frame_bounds->height;
                break;
        }

        mutex_unlock(&device->mutex);

        return 0;
}

int vc_sd_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_selection *sel)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);

        mutex_lock(&device->mutex);

        switch (sel->target) {
        case V4L2_SEL_TGT_CROP:
                vc_core_set_frame(cam, sel->r.left, sel->r.top, sel->r.width, sel->r.height);
                break;
        }

        mutex_unlock(&device->mutex);

        return 0;
}

#endif








// --- v4l2_ctrl_ops ---------------------------------------------------

int vc_ctrl_s_ctrl(struct v4l2_ctrl *ctrl)
{
        struct vc_device *device = container_of(ctrl->handler, struct vc_device, ctrl_handler);
        struct v4l2_control control;
#ifdef ENABLE_PM
        struct i2c_client *client = device->cam.ctrl.client_sen;

        V4L2 controls values will be applied only when power is already up
        if (!pm_runtime_get_if_in_use(&client->dev))
        	return 0;
#endif

        mutex_lock(&device->mutex);

        control.id = ctrl->id;
        control.value = ctrl->val;
        vc_sd_s_ctrl(&device->sd, &control);

        mutex_unlock(&device->mutex);

        return 0;
}

#ifdef ENABLE_VVCAM
// --- Vivante Sensor IOCTL ---------------------------------------------------

static int vc_vidioc_querycap(struct vc_device *device, void *arg)
{
        struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

        strcpy((char *)pcap->driver, "vc-mipi-vvcam");
        sprintf((char *)pcap->bus_info, "csi%d", device->csi_id);
        pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = (__u8)device->cam.ctrl.client_sen->adapter->nr;

        return 0;
}

// #define DEBUG_MODE_INFO
static void vc_get_mode_info(struct vc_device *device, struct vvcam_mode_info_s *info)
{
        struct vc_cam *cam = &device->cam;
#ifdef DEBUG_MODE_INFO
        struct device *dev = vc_core_get_sen_device(cam);
#endif
        struct vc_frame *frame = vc_core_get_frame(cam);
        struct vc_mode mode = vc_core_get_mode(cam);
        __u32 num_lanes = vc_core_get_num_lanes(cam);
        __u32 code = vc_core_get_format(cam);
        __u32 time_per_line_ns = vc_core_get_time_per_line_ns(cam);
        __u32 framerate = vc_core_get_framerate(cam);

        // Required infos for streaming
        info->index = 0;
        info->hdr_mode = SENSOR_MODE_LINEAR;
        info->data_compress.enable = 0;
        info->mipi_info.mipi_lane = num_lanes;

        info->size.left = 0;
        info->size.top = 0;
        info->size.width = frame->width;
        info->size.height = frame->height;
        info->size.bounds_width = frame->width;
        info->size.bounds_height = frame->height;

        switch (code) {
        case MEDIA_BUS_FMT_Y8_1X8:
        case MEDIA_BUS_FMT_SRGGB8_1X8:
                info->bit_width = 8;
                info->bayer_pattern = BAYER_RGGB;
                break;
        case MEDIA_BUS_FMT_SGBRG8_1X8:
                info->bit_width = 8;
                info->bayer_pattern = BAYER_GBRG;
                break;
        case MEDIA_BUS_FMT_Y10_1X10:
        case MEDIA_BUS_FMT_SRGGB10_1X10:
                info->bit_width = 10;
                info->bayer_pattern = BAYER_RGGB;
                break;
        case MEDIA_BUS_FMT_SGBRG10_1X10:
                info->bit_width = 10;
                info->bayer_pattern = BAYER_GBRG;
                break;
        case MEDIA_BUS_FMT_Y12_1X12:
        case MEDIA_BUS_FMT_SRGGB12_1X12:
                info->bit_width = 12;
                info->bayer_pattern = BAYER_RGGB;
                break;
        case MEDIA_BUS_FMT_SGBRG12_1X12:
                info->bit_width = 12;
                info->bayer_pattern = BAYER_GBRG;
                break;
        case MEDIA_BUS_FMT_Y14_1X14:
        case MEDIA_BUS_FMT_SRGGB14_1X14:
                info->bit_width = 14;
                info->bayer_pattern = BAYER_RGGB;
                break;
        case MEDIA_BUS_FMT_SGBRG14_1X14:
                info->bit_width = 14;
                info->bayer_pattern = BAYER_GBRG;
                break;
        }
        
        // Required infos for auto exposure control
        info->ae_info.one_line_exp_time_ns  = time_per_line_ns;
        info->ae_info.max_integration_line  = (__u64)1000000000000 / framerate / time_per_line_ns;
        info->ae_info.min_integration_line  = mode.vmax.min;
        info->ae_info.def_frm_len_lines     = 0;
        info->ae_info.curr_frm_len_lines    = 0;
        info->ae_info.start_exposure        = 0;

        info->ae_info.max_again             = cam->ctrl.gain.max_mdB;           // mdB
        info->ae_info.min_again             = 1;                                // mdB
        info->ae_info.max_dgain             = 1 * (1 << SENSOR_FIX_FRACBITS);   // 1024 mdB 
        info->ae_info.min_dgain             = 1 * (1 << SENSOR_FIX_FRACBITS);   // 1024 mdB
        
        info->ae_info.cur_fps               = framerate;                        // mHz
        info->ae_info.max_fps               = cam->ctrl.framerate.max;          // mHz
        info->ae_info.min_fps               = 1 * (1 << SENSOR_FIX_FRACBITS);   // 1024 mHz
        info->ae_info.min_afps              = 1 * (1 << SENSOR_FIX_FRACBITS);   // 1024 mHz

        info->ae_info.int_update_delay_frm  = 1;
        info->ae_info.gain_update_delay_frm = 1;

#ifdef DEBUG_MODE_INFO
        vc_info(dev, "%s(): ------------------------------------------\n", __func__);
        vc_info(dev, "%s(): size.left:            %5u px\n", __func__, info->size.left);
        vc_info(dev, "%s(): size.top:             %5u px\n", __func__, info->size.top);
        vc_info(dev, "%s(): size.width:           %5u px\n", __func__, info->size.width);
        vc_info(dev, "%s(): size.height:          %5u px\n", __func__, info->size.height);
        vc_info(dev, "%s(): ------------------------------------------\n", __func__);
        vc_info(dev, "%s(): one_line_exp_time_ns: %5u ns\n", __func__, info->ae_info.one_line_exp_time_ns);
        vc_info(dev, "%s(): max_integration_line: %5u lines\n", __func__, info->ae_info.max_integration_line);
        vc_info(dev, "%s(): min_integration_line: %5u lines\n", __func__, info->ae_info.min_integration_line);
        vc_info(dev, "%s(): def_frm_len_lines:    %5u lines\n", __func__, info->ae_info.def_frm_len_lines);
        vc_info(dev, "%s(): curr_frm_len_lines:   %5u lines\n", __func__, info->ae_info.curr_frm_len_lines);
        vc_info(dev, "%s(): start_exposure:       %5u lines\n", __func__, info->ae_info.start_exposure);
        vc_info(dev, "%s(): ------------------------------------------\n", __func__);
        vc_info(dev, "%s(): max_again:            %5u mdB\n", __func__, info->ae_info.max_again);
        vc_info(dev, "%s(): ------------------------------------------\n", __func__);
        vc_info(dev, "%s(): cur_fps:              %5u mHz\n", __func__, info->ae_info.cur_fps);
        vc_info(dev, "%s(): max_fps:              %5u mHz\n", __func__, info->ae_info.max_fps);
        vc_info(dev, "%s(): min_fps:              %5u mHz\n", __func__, info->ae_info.min_fps);
        vc_info(dev, "%s(): min_afps:             %5u mHz\n", __func__, info->ae_info.min_afps);
        vc_info(dev, "%s(): ------------------------------------------\n", __func__);
#endif
}


static int vc_vvsensorioc_query(struct vc_device *device, struct vvcam_mode_info_array_s *mode_info)
{
        vc_get_mode_info(device, &mode_info->modes[0]);
        mode_info->count = 1;
        return 0;
}

static int vc_vvsensorioc_g_sensor_mode(struct vc_device *device, struct vvcam_mode_info_s *mode)
{
        vc_get_mode_info(device, mode);
        return 0;
}

static long vc_sd_vvsensorioc(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        long ret = 0;

        switch (cmd){
        // Required cases for streaming
        case VIDIOC_QUERYCAP:
                ret = vc_vidioc_querycap(device, arg);
                vc_dbg(sd->dev, "%s(): VIDIOC_QUERYCAP\n", __func__);
                break;
        case VVSENSORIOC_QUERY:
                ret = vc_vvsensorioc_query(device, arg);
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_QUERY\n", __func__);
                break;
        case VVSENSORIOC_G_SENSOR_MODE:
                ret = vc_vvsensorioc_g_sensor_mode(device, arg);
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_G_SENSOR_MODE [index: %u]\n", __func__,
                        ((struct vvcam_mode_info_s *)arg)->index);
                break;
        case VVSENSORIOC_S_STREAM:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_STREAM [%u]\n", __func__, *(u32 *)arg);
                ret = vc_sd_s_stream(sd, *(int *)arg);
                break;

        // Required cases for auto exposure control
        case VVSENSORIOC_S_EXP:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_EXP [%u]\n", __func__, *(u32 *)arg);
                ret = vc_sen_set_exposure(&device->cam, ((*(u32 *)arg) * vc_core_get_time_per_line_ns(cam)) / 1000);
                break;
        case VVSENSORIOC_S_GAIN:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_GAIN [%u]\n", __func__, *(u32 *)arg);
                ret = vc_sen_set_gain(&device->cam, *(u32 *)arg);
                break;
        case VVSENSORIOC_S_FPS:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_FPS [%u]\n", __func__, *(u32 *)arg);
                // NOTE: Diese Funktion wird nicht mehr aufgerufen. 
                // ret = vc_core_set_framerate(&device->cam, *(u32 *)arg);
                break;
        case VVSENSORIOC_G_FPS:
                *(u32 *)arg = vc_core_get_framerate(&device->cam);
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_G_FPS [%u]\n", __func__, *(u32 *)arg);
                break;

        // Not implemented but called cases
        case VVSENSORIOC_RESET:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_RESET [%u] (not implemented)\n", __func__, *(u32 *)arg);
                break;
        case VVSENSORIOC_S_POWER:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_POWER [%u] (not implemented)\n", __func__, *(u32 *)arg);
                break;
        case VVSENSORIOC_S_CLK:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_CLK [%lu] (not implemented)\n", __func__, 
                        ((struct vvcam_clk_s*)arg)->sensor_mclk);
                break;
        case VVSENSORIOC_G_CLK:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_G_CLK [%lu] (not implemented)\n", __func__,
                        ((struct vvcam_clk_s*)arg)->sensor_mclk);
                break;
        case VVSENSORIOC_G_RESERVE_ID:
                *(u32 *)arg = 0;
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_G_RESERVE_ID [0x%04x] (not implemented)\n", __func__, *(u32 *)arg);
                break;
        case VVSENSORIOC_G_CHIP_ID:
                *(u32 *)arg = 0;
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_G_CHIP_ID [0x%04x] (not implemented)\n", __func__, *(u32 *)arg);
                break;
        case VVSENSORIOC_S_SENSOR_MODE:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_SENSOR_MODE [index: %u] (not implemented)\n", __func__,
                        ((struct vvcam_mode_info_s *)arg)->index);
                break;
        case VVSENSORIOC_S_HDR_RADIO:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_HDR_RADIO [%u] (not implemented)\n", __func__, *(u32 *)arg);
                break;
        case VVSENSORIOC_S_TEST_PATTERN:
                vc_dbg(sd->dev, "%s(): VVSENSORIOC_S_TEST_PATTERN [%u] (not implemented)\n", __func__, *(u32 *)arg);
                break;
        default:
                vc_dbg(sd->dev, "%s(): invalid IOCTL 0x%x\n", __func__, cmd);
                break;
        }

        return ret;
}
#endif
#ifdef ENABLE_RK
#define IMX415_NAME			"imx415"

static void imx415_get_module_inf(struct vc_cam *cam,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX415_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, cam->desc.sen_type,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, "", sizeof(inf->base.lens));
}
static int imx415_get_channel_info(struct vc_cam *cam, struct rkmodule_channel_info *ch_info)
{
    struct vc_frame* frame = NULL;
        __u32 code = vc_core_get_format(cam);


    frame = vc_core_get_frame(cam);
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

    //TODO Adjust for 4 lanes
	ch_info->vc = V4L2_MBUS_CSI2_CHANNEL_0;
	ch_info->width = frame->width;
	ch_info->height = frame->height;
	ch_info->bus_fmt = code;
	return 0;
}

static long imx415_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    struct vc_device *device __maybe_unused = to_vc_device(sd);
    struct vc_cam *cam = to_vc_cam(sd);
    // struct vc_mode mode = vc_core_get_mode(cam);
    struct vc_frame* frame = NULL;


    struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	u32 h, w, stream;
	long ret = 0;
	// u64 pixel_rate = 0;
	// struct rkmodule_csi_dphy_param *dphy_param;

    frame = vc_core_get_frame(cam);

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		// if (mode->hdr_mode == HDR_X2)
		// 	ret = imx415_set_hdrae(imx415, arg);
		// else if (mode->hdr_mode == HDR_X3)
		// 	ret = imx415_set_hdrae_3frame(imx415, arg);
		break;
	case RKMODULE_GET_MODULE_INFO:
		imx415_get_module_inf(cam, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = NO_HDR;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = frame->width;
		h = frame->height;
		// for (i = 0; i < device->cfg_num; i++) {
		// 	if (w == supported_modes[i].width &&
		// 	    h == supported_modes[i].height &&
		// 	    supported_modes[i].hdr_mode == hdr->hdr_mode) {
		// 		imx415_change_mode(device, &supported_modes[i]);
		// 		break;
		// 	}
		// }
		// if (i == dev->cfg_num) {
		// 	dev_err(&cam->client->dev,
		// 		"not find hdr mode:%d %dx%d config\n",
		// 		hdr->hdr_mode, w, h);
		// 	ret = -EINVAL;
		// } else {
		// 	mode = dev->cur_mode;
		// 	if (imx415->streaming) {
		// 		ret = imx415_write_reg(dev->client, IMX415_GROUP_HOLD_REG,
		// 			IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_START);

		// 		ret |= imx415_write_array(dev->client, dev->cur_mode->reg_list);

		// 		ret |= imx415_write_reg(dev->client, IMX415_GROUP_HOLD_REG,
		// 			IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_END);
		// 		if (ret)
		// 			return ret;
		// 	}
		// 	w = mode->hts_def - dev->cur_mode->width;
		// 	h = mode->vts_def - mode->height;
		// 	mutex_lock(&dev->mutex);
		// 	__v4l2_ctrl_modify_range(dev->hblank, w, w, 1, w);
		// 	__v4l2_ctrl_modify_range(dev->vblank, h,
		// 		IMX415_VTS_MAX - mode->height,
		// 		1, h);
		// 	__v4l2_ctrl_s_ctrl(dev->link_freq, mode->mipi_freq_idx);
		// 	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * IMX415_4LANES;
		// 	__v4l2_ctrl_s_ctrl_int64(dev->pixel_rate,
		// 				 pixel_rate);
		// 	mutex_unlock(&dev->mutex);
		// }
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		// if (stream)
		// 	ret = imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE,
		// 		IMX415_REG_VALUE_08BIT, IMX415_MODE_STREAMING);
		// else
		// 	ret = imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE,
		// 		IMX415_REG_VALUE_08BIT, IMX415_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_SONY_BRL:
    //TODO Adjust for binning
		*((u32 *)arg) = BRL_ALL;
		
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = imx415_get_channel_info(cam, ch_info);
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		// if (imx415->cur_mode->hdr_mode == HDR_X2) {
		// 	dphy_param = (struct rkmodule_csi_dphy_param *)arg;
		// 	if (dphy_param->vendor == dcphy_param.vendor)
		// 		*dphy_param = dcphy_param;
		// 	dev_info(&imx415->client->dev,
		// 		 "get sensor dphy param\n");
		// } else
			ret = -EINVAL;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}



#endif
// *** Initialisation *********************************************************

// static void vc_setup_power_gpio(struct vc_device *device)
// {
//         struct device *dev = &device->cam.ctrl.client_sen->dev;

//         device->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
//         if (IS_ERR(device->power_gpio)) {
//                 vc_err(dev, "%s(): Failed to setup power-gpio\n", __func__);
//                 device->power_gpio = NULL;
//         }
// }

static int vc_check_hwcfg(struct vc_device *device, struct device *dev)
{
        struct vc_cam *cam = &device->cam;
        struct fwnode_handle *endpoint;
        struct v4l2_fwnode_endpoint ep_cfg = {
                .bus_type = V4L2_MBUS_CSI2_DPHY
        };
        int ret = -EINVAL;

        ret = of_property_read_u32(dev->of_node, "csi_id", &(device->csi_id));
	if (ret) {
		dev_err(dev, "csi id missing or invalid\n");
		return ret;
	}

        endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
        if (!endpoint) {
                dev_err(dev, "Endpoint node not found!\n");
                return -EINVAL;
        }

        if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
                dev_err(dev, "Could not parse endpoint!\n");
                goto error_out;
        }

        /* Set and check the number of MIPI CSI2 data lanes */
        ret = vc_core_set_num_lanes(cam, ep_cfg.bus.mipi_csi2.num_data_lanes);
        ret = vc_core_set_link_frequencies(cam, ep_cfg.link_frequencies, ep_cfg.nr_of_link_frequencies);

error_out:
        v4l2_fwnode_endpoint_free(&ep_cfg);
        fwnode_handle_put(endpoint);

        return ret;
}

static const struct v4l2_subdev_core_ops vc_core_ops = {
        .s_power = vc_sd_s_power,
#ifdef ENABLE_VVCAM
        .ioctl = vc_sd_vvsensorioc,
#endif
#ifdef ENABLE_RK
    .ioctl = imx415_ioctl,
#endif
};

static const struct v4l2_subdev_video_ops vc_video_ops = {
        .s_stream = vc_sd_s_stream,
        .g_frame_interval = vc_sd_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops vc_pad_ops = {
        .enum_mbus_code = vc_sd_enum_mbus_code,
        .get_fmt = vc_sd_get_fmt,
        .set_fmt = vc_sd_set_fmt,
        .get_selection = vc_sd_get_selection,
        .set_selection = vc_sd_set_selection,

        // .enum_frame_size = imx415_enum_frame_sizes,
        // .enum_frame_interval = imx415_enum_frame_interval,
	    // .get_mbus_config = imx415_g_mbus_config,
};

static const struct v4l2_subdev_ops vc_subdev_ops = {
        .core = &vc_core_ops,
        .video = &vc_video_ops,
        .pad = &vc_pad_ops,
};

static const struct v4l2_ctrl_ops vc_ctrl_ops = {
        .s_ctrl = vc_ctrl_s_ctrl,
};

static int vc_ctrl_init_ctrl(struct vc_device *device, struct v4l2_ctrl_handler *hdl, int id, int min, int max, int def)
{
        struct i2c_client *client = device->cam.ctrl.client_sen;
        struct device *dev = &client->dev;
        struct v4l2_ctrl *ctrl;

        ctrl = v4l2_ctrl_new_std(&device->ctrl_handler, &vc_ctrl_ops, id, min, max, 1, def);
        if (ctrl == NULL) {
                vc_err(dev, "%s(): Failed to init 0x%08x ctrl\n", __func__, id);
                return -EIO;
        }

        return 0;
}

static int vc_ctrl_init_custom_ctrl(struct vc_device *device, struct v4l2_ctrl_handler *hdl, const struct v4l2_ctrl_config *config)
{
        struct i2c_client *client = device->cam.ctrl.client_sen;
        struct device *dev = &client->dev;
        struct v4l2_ctrl *ctrl;

        ctrl = v4l2_ctrl_new_custom(&device->ctrl_handler, config, NULL);
        if (ctrl == NULL) {
                vc_err(dev, "%s(): Failed to init 0x%08x ctrl\n", __func__, config->id);
                return -EIO;
        }

        return 0;
}

static const struct v4l2_ctrl_config ctrl_csi_lanes = {
	.ops = &vc_ctrl_ops,
	.id = V4L2_CID_CSI_LANES,
	.name = "CSI Lanes",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	.max = ARRAY_SIZE(ctrl_csi_lanes_menu) - 1,
	.def = 2,
	.qmenu_int = ctrl_csi_lanes_menu,
};

static const struct v4l2_ctrl_config ctrl_black_level = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_BLACK_LEVEL,
        .name = "Black Level",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 100000,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_trigger_mode = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_TRIGGER_MODE,
        .name = "Trigger Mode",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 7,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_io_mode = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_IO_MODE,
        .name = "IO Mode",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 1,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_frame_rate = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_FRAME_RATE,
        .name = "Frame Rate",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 1000000,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_single_trigger = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_SINGLE_TRIGGER,
        .name = "Single Trigger",
        .type = V4L2_CTRL_TYPE_BUTTON,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 1,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_binning_mode = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_BINNING_MODE,
        .name = "Binning Mode",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 4,
        .step = 1,
        .def = 0,
};

#ifdef ENABLE_ADVANCED_CONTROL
static const struct v4l2_ctrl_config ctrl_hmax_overwrite = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_HMAX_OVERWRITE,
        .name = "hmax Overwrite",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = -1,
        .max = 10000,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_vmax_overwrite = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_VMAX_OVERWRITE,
        .name = "vmax Overwrite",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = -1,
        .max = 10000,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_height_offset = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_HEIGHT_OFFSET,
        .name = "Height Offset",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = -10000,
        .max = 10000,
        .step = 1,
        .def = 0,
};
#endif

static int vc_sd_init(struct vc_device *device)
{
        struct i2c_client *client = device->cam.ctrl.client_sen;
        struct device *dev = &client->dev;
        int ret;
	    u64 pixel_rate = 0;

        // Initializes the subdevice
        v4l2_i2c_subdev_init(&device->sd, client, &vc_subdev_ops);

        // Initialize the handler
        ret = v4l2_ctrl_handler_init(&device->ctrl_handler, 3);
        if (ret) {
                vc_err(dev, "%s(): Failed to init control handler\n", __func__);
                return ret;
        }
        // Hook the control handler into the driver
        device->sd.ctrl_handler = &device->ctrl_handler;


		pixel_rate = (u32)device->cam.state.framerate / vc_core_mbus_to_bpp(device->cam.state.format_code) * 2 * device->cam.state.num_lanes;

        // Add controls
        ret |= vc_ctrl_init_ctrl(device, &device->ctrl_handler, V4L2_CID_EXPOSURE, 
                device->cam.ctrl.exposure.min, device->cam.ctrl.exposure.max,
                device->cam.ctrl.exposure.def);
        ret |= vc_ctrl_init_ctrl(device, &device->ctrl_handler, V4L2_CID_GAIN, 
                0, device->cam.ctrl.gain.max_mdB, 0);
        ret |= 	vc_ctrl_init_ctrl(device, &device->ctrl_handler, 	V4L2_CID_PIXEL_RATE, 0, IMX415_MAX_PIXEL_RATE, pixel_rate);
        v4l2_ctrl_new_int_menu(&device->ctrl_handler, &vc_ctrl_ops,          V4L2_CID_LINK_FREQ,  device->cam.state.num_link_frequencies - 1, 0,     device->cam.state.link_frequencies);
   
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_csi_lanes);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_black_level);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_trigger_mode);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_io_mode);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_frame_rate);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_single_trigger);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_binning_mode);
#ifdef ENABLE_ADVANCED_CONTROL
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_hmax_overwrite);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_vmax_overwrite);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_height_offset);
#endif

        return 0;
}

static int vc_link_setup(struct media_entity *entity, const struct media_pad *local, const struct media_pad *remote,
                         __u32 flags)
{
        return 0;
}

static const struct media_entity_operations vc_sd_media_ops = {
        .link_setup = vc_link_setup,
};

static int vc_probe(struct i2c_client *client)
{
        struct device *dev = &client->dev;
        struct vc_device *device;
        struct vc_cam *cam;
        int ret;

        vc_notice(dev, "%s(): Probing UNIVERSAL VC MIPI Driver (v%s)\n", __func__, VERSION);

        device = devm_kzalloc(dev, sizeof(*device), GFP_KERNEL);
        if (!device)
                return -ENOMEM;

        cam = &device->cam;
        cam->ctrl.client_sen = client;

        // vc_setup_power_gpio(device);
        vc_set_power(device, 1);

        ret = vc_core_init(cam, client);
        if (ret)
                goto error_power_off;

        ret = vc_check_hwcfg(device, dev);
        if (ret)
                goto error_power_off;

        mutex_init(&device->mutex);
        ret = vc_sd_init(device);
        if (ret)
                goto error_handler_free;

        device->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
        device->pad.flags = MEDIA_PAD_FL_SOURCE;
        device->sd.entity.ops = &vc_sd_media_ops;
        device->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
        ret = media_entity_pads_init(&device->sd.entity, 1, &device->pad);
        if (ret)
                goto error_handler_free;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)                
        ret = v4l2_async_register_subdev_sensor_common(&device->sd);
#else
        ret = v4l2_async_register_subdev_sensor(&device->sd);
#endif
        if (ret)
                goto error_media_entity;

#ifdef ENABLE_PM
        pm_runtime_get_noresume(dev);
        pm_runtime_set_active(dev);
        pm_runtime_enable(dev);
        pm_runtime_set_autosuspend_delay(dev, 2000);
        pm_runtime_use_autosuspend(dev);
        pm_runtime_mark_last_busy(dev);
        pm_runtime_put_autosuspend(dev);
#endif

        return 0;

error_media_entity:
        media_entity_cleanup(&device->sd.entity);

error_handler_free:
        v4l2_ctrl_handler_free(&device->ctrl_handler);
        mutex_destroy(&device->mutex);

error_power_off:
#ifdef ENABLE_PM
        pm_runtime_disable(dev);
        pm_runtime_set_suspended(dev);
        pm_runtime_put_noidle(dev);
#endif
        vc_set_power(device, 0);
        vc_core_release(&device->cam);
        return ret;
}

static int vc_remove(struct i2c_client *client)
{
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct vc_device *device = to_vc_device(sd);

        v4l2_async_unregister_subdev(&device->sd);
        media_entity_cleanup(&device->sd.entity);
        v4l2_ctrl_handler_free(&device->ctrl_handler);
#ifdef ENABLE_PM
        pm_runtime_disable(&client->dev);
#endif
        mutex_destroy(&device->mutex);

#ifdef ENABLE_PM
        pm_runtime_get_sync(&client->dev);
        pm_runtime_disable(&client->dev);
        pm_runtime_set_suspended(&client->dev);
        pm_runtime_put_noidle(&client->dev);
#endif

        vc_set_power(device, 0);
        vc_core_release(&device->cam);

        return 0;
}

#ifdef ENABLE_PM
static const struct dev_pm_ops vc_pm_ops = {
        SET_SYSTEM_SLEEP_ENABLE_PM_OPS(vc_suspend, vc_resume)
};
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id vc_acpi_ids[] = {
        {"VCMIPICAM"},
        {}
};

MODULE_DEVICE_TABLE(acpi, vc_acpi_ids);
#endif

static const struct i2c_device_id vc_id[] = {
        { "vc-mipi-cam", 0 },
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, vc_id);

static const struct of_device_id vc_dt_ids[] = {
        { .compatible = "vc,vc_mipi" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vc_dt_ids);

static struct i2c_driver vc_i2c_driver = {
        .driver = {
                .name = "vc-mipi-cam",
#ifdef ENABLE_PM
                .pm = &vc_pm_ops,
#endif
                .acpi_match_table = ACPI_PTR(vc_acpi_ids),
                .of_match_table = vc_dt_ids,
        },
        .id_table = vc_id,
        .probe_new = vc_probe,
        .remove   = vc_remove,
};

module_i2c_driver(vc_i2c_driver);

MODULE_VERSION(VERSION);
MODULE_DESCRIPTION("Vision Components GmbH - VC MIPI CSI-2 driver");
MODULE_AUTHOR("Peter Martienssen, Liquify Consulting <peter.martienssen@liquify-consulting.de>");
MODULE_LICENSE("GPL v2");