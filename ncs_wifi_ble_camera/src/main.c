/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Network Camera Demo
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(NetCam, CONFIG_LOG_DEFAULT_LEVEL);

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>

#include "snapshot.h"
#include "socket_util.h"

#define VIDEO_DEV_SW "VIDEO_SW_GENERATOR"

/**********External Resources START**************/
extern struct sockaddr_in pc_addr;
/**********External Resources END**************/

#define RESET_CAMERA 0XFF
#define SET_PICTURE_RESOLUTION 0X01
#define SET_VIDEO_RESOLUTION 0X02
#define SET_BRIGHTNESS 0X03
#define SET_CONTRAST 0X04
#define SET_SATURATION 0X05
#define SET_EV 0X06
#define SET_WHITEBALANCE 0X07
#define SET_SPECIAL_EFFECTS 0X08
#define SET_FOCUS_ENABLE 0X09
#define SET_EXPOSURE_GAIN_ENABLE 0X0A
#define SET_WHITE_BALANCE_ENABLE 0X0C
#define SET_MANUAL_GAIN 0X0D
#define SET_MANUAL_EXPOSURE 0X0E
#define GET_CAMERA_INFO 0X0F
#define TAKE_PICTURE 0X10
#define SET_SHARPNESS 0X11
#define DEBUG_WRITE_REGISTER 0X12
#define STOP_STREAM 0X21
#define GET_FRM_VER_INFO 0X30
#define GET_SDK_VER_INFO 0X40
#define SET_IMAGE_QUALITY 0X50
#define SET_LOWPOWER_MODE 0X60
#define SNAPSHOT 0x70

/*
** Arducam mega communication protocols
** https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/HostCommunicationProtocol.html
*/

const struct device *video;
struct video_buffer *vbuf;
struct video_format fmt;
struct video_caps caps;

enum client_type { client_type_NONE, client_type_BLE, client_type_SOCKET };

enum app_command_type {
  APPCMD_CAM_COMMAND,
  APPCMD_TAKE_PICTURE,
  APPCMD_START_STOP_STREAM,
  APPCMD_SOCKET_RX
};

struct app_command_t {
  enum app_command_type type;
  enum client_type mode;
  uint8_t cam_cmd;
  uint8_t data[6];
};

K_MSGQ_DEFINE(msgq_app_commands, sizeof(struct app_command_t), 8, 4);

struct client_state {
  uint8_t req_stream : 1;
  uint8_t req_stream_stop : 1;
  uint8_t stream_active : 1;
};

struct client_state client_state_socket = {0};

static void on_timer_count_bytes_func(struct k_timer *timer);
K_TIMER_DEFINE(m_timer_count_bytes, on_timer_count_bytes_func, NULL);
static int counted_bytes_sent = 0;

/* actucal command not count start(0x55) and stop(0xAA) codes*/
uint8_t socket_head_and_tail[] = {0xff, 0xaa, 0x00, 0xff, 0xbb};

// static uint8_t current_resolution;
static uint8_t take_picture_fmt = 0x1a;

void cam_to_pc_command_send(uint8_t type, uint8_t *buffer, uint32_t length) {
  socket_head_and_tail[2] = type;
  cam_send(&socket_head_and_tail, 3);
  if (length != 0) {
    cam_send((uint8_t *)&length, 4);
    cam_send(buffer, length);
  }
  cam_send(&socket_head_and_tail[3], 2);
}

int set_mega_resolution(uint8_t sfmt, enum client_type source_client) {
  printk("set mega resolution: %u\n", sfmt);
  return 0;
}

void cam_send_picture_data_socket(uint8_t *data, int length) {
  counted_bytes_sent += length;
  while (length >= 1024) {
    cam_send(data, 1024);
    data += 1024;
    length -= 1024;
  }
  if (length > 0) {
    cam_send(data, length);
  }
}

static void client_request_single_capture(struct client_state *state) {
  if (!state->stream_active) {
    state->req_stream = 1;
    state->req_stream_stop = 1;
  }
}

static void client_request_stream_start_stop(struct client_state *state,
                                             bool start) {
  if (start) {
    state->req_stream = 1;
  } else {
    state->req_stream_stop = 1;
  }
}

static int client_check_start_request(struct client_state *state) {
  if (state->req_stream && !state->stream_active) {
    state->req_stream = 0;
    state->stream_active = 1;
    return 1;
  }
  return 0;
}

static int client_check_stop_request(struct client_state *state) {
  if (state->req_stream_stop && state->stream_active) {
    state->req_stream_stop = 0;
    state->stream_active = 0;
    return 1;
  }
  return 0;
}

void video_preview(void) {
  int err;
  // static bool capture_flag = false;
  err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
  if (err) {
    LOG_INF("Unable to dequeue video buf");
    return;
  }
  counted_bytes_sent += vbuf->bytesused;
  cam_send(vbuf->buffer, vbuf->bytesused);
  err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
  if (err) {
    LOG_ERR("Unable to requeue video buf");
    return;
  }
  // test
  //   socket_head_and_tail[2] = 0x01;
  //   cam_send(&socket_head_and_tail[0], 3);
  //   if (ret && ret != -EAGAIN) {
  //     /* client disconnected */
  //     printk("\nTCP: Client disconnected %d\n", ret);
  //   }
  // test end

  //   f_status = vbuf->flags;

  //   LOG_DBG("CF1: f_status %i, bytesframe %i, bytesused %i. cf %i", f_status,
  //           vbuf->bytesframe, vbuf->bytesused, capture_flag);

  //   if (capture_flag) {
  //     capture_flag = false;

  //     client_check_start_request(&client_state_socket);

  //     if (client_state_socket.stream_active) {
  //       socket_head_and_tail[2] = 0x01;
  //       cam_send(&socket_head_and_tail[0], 3);
  //       cam_send((uint8_t *)&vbuf->bytesframe, 4);
  //       LOG_DBG("bytesframe:%d", vbuf->bytesframe);
  //     }
  //   }

  //   if (client_state_socket.stream_active) {
  //     cam_send_picture_data_socket(vbuf->buffer, vbuf->bytesused);
  //     LOG_DBG("bytesused:%d", vbuf->bytesused);
  //   }

  //   if (f_status == VIDEO_BUF_EOF) {
  //     capture_flag = true;

  //     if (client_state_socket.stream_active) {
  //       cam_send(&socket_head_and_tail[3], 2);
  //     }

  //     client_check_stop_request(&client_state_socket);
  //   }
}

int report_mega_info(void) {
  //   static char str_buf[400];
  //   uint32_t str_len;
  //   char *mega_type;

  //   switch (mega_info.camera_id) {
  //     case ARDUCAM_SENSOR_3MP_1:
  //     case ARDUCAM_SENSOR_3MP_2:
  //       mega_type = "3MP";
  //       break;
  //     case ARDUCAM_SENSOR_5MP_1:
  //       mega_type = "5MP";
  //       break;
  //     case ARDUCAM_SENSOR_5MP_2:
  //       mega_type = "5MP_2";cam_send(&socket_head_and_tail[0], 3);r\nCamera
  //       Type:%s\r\n"
  //           "Camera Support Resolution:%d\r\nCamera Support "
  //           "special effects:%d\r\nCamera Support Focus:%d\r\n"
  //           "Camera Exposure Value Max:%ld\r\nCamera Exposure Value "
  //           "Min:%d\r\nCamera Gain Value Max:%d\r\nCamera Gain Value "
  //           "Min:%d\r\nCamera Support Sharpness:%d\r\n",
  //           mega_type, mega_info.support_resolution,
  //           mega_info.support_special_effects, mega_info.enable_focus,
  //           mega_info.exposure_value_max, mega_info.exposure_value_min,
  //           mega_info.gain_value_max, mega_info.gain_value_min,
  //           mega_info.enable_sharpness);
  //   printk("%s", str_buf);
  //   str_len = strlen(str_buf);
  //   cam_to_pc_command_send(0x02, str_buf, str_len);
  return 0;
}

uint8_t recv_process(uint8_t *buff) {
  LOG_INF("recv_process: cmd %x, data %x", buff[0], buff[1]);
  switch (buff[0]) {
    case SNAPSHOT:
      SNAPSHOT_HANDLING_ASM();
      break;
    case SET_PICTURE_RESOLUTION:
      LOG_INF("camcmd: SET_PICTURE_RESOLUTION");
      if (set_mega_resolution(buff[1], client_type_SOCKET) == 0) {
        take_picture_fmt = buff[1];
      }
      break;
    case SET_VIDEO_RESOLUTION:
      LOG_INF("camcmd: SET_VIDEO_RESOLUTION");
      if (!client_state_socket.stream_active) {
        set_mega_resolution(buff[1] | 0x10, client_type_SOCKET);
        client_request_stream_start_stop(&client_state_socket, true);
      }
      break;
    case SET_BRIGHTNESS:
      printk("set bright %u\n", buff[1]);
      break;
    case SET_CONTRAST:
      printk("SET_CONTRAST %u\n", buff[1]);
      break;
    case SET_SATURATION:
      printk("SET_SATURATION %u\n", buff[1]);
      break;
    case SET_EV:
      printk("SET_EV %u\n", buff[1]);
      break;
    case SET_WHITEBALANCE:
      printk("SET_WHITEBALANCE %u\n", buff[1]);
      break;
    case SET_SPECIAL_EFFECTS:
      printk("SET_SPECIAL_EFFECTS %u\n", buff[1]);
      break;
    case SET_EXPOSURE_GAIN_ENABLE:
      printk("SET_EXPOSURE_GAIN_ENABLE %u\n", buff[1]);
    case SET_WHITE_BALANCE_ENABLE:
      printk("SET_WHITE_BALANCE_ENABLE %u\n", buff[1]);
      break;
    case SET_SHARPNESS:
      printk("SET_SHARPNESS %u\n", buff[1]);
      break;
    case SET_MANUAL_GAIN:
      uint16_t gain_value = (buff[1] << 8) | buff[2];

      printk("SET_MANUAL_GAIN %u\n", gain_value);
      break;
    case SET_MANUAL_EXPOSURE:
      uint32_t exposure_value = (buff[1] << 16) | (buff[2] << 8) | buff[3];

      printk("SET_MANUAL_EXPOSURE %u\n", exposure_value);
      break;
    case GET_CAMERA_INFO:
      report_mega_info();
      break;
    case TAKE_PICTURE:
      LOG_INF("Take picture");
      client_request_single_capture(&client_state_socket);
      break;
    case STOP_STREAM:
      if (client_state_socket.stream_active) {
        LOG_INF("Stop video stream");
        client_request_stream_start_stop(&client_state_socket, false);
      }
      break;
    case RESET_CAMERA:
      printk("RESET_CAMERA\n");
      break;
    case SET_IMAGE_QUALITY:
      printk("SET_IMAGE_QUALITY %u\n", buff[1]);
      break;
    case SET_LOWPOWER_MODE:
      printk("SET_LOWPOWER_MODE %u\n", buff[1]);
      break;
    default:
      break;
  }

  return buff[0];
}

static void register_app_command(const struct app_command_t *command) {
  if (k_msgq_put(&msgq_app_commands, command, K_NO_WAIT) != 0) {
    LOG_ERR("Command buffer full!");
  }
}

static void on_timer_count_bytes_func(struct k_timer *timer) {
  if (counted_bytes_sent > 0) {
    LOG_INF("Data transferred: %i kbps", (counted_bytes_sent * 8) / 1024);
    counted_bytes_sent = 0;
  }
}

void socket_rx_callback(uint8_t *data, uint16_t len) {
  static struct app_command_t app_cmd_socket = {.type = APPCMD_SOCKET_RX};
  LOG_DBG("SOCKET RX callback");
  memcpy(app_cmd_socket.data, data, len);
  register_app_command(&app_cmd_socket);
}

int main(void) {
  int ret;
  struct video_buffer *buffers[2];
  k_sleep(K_SECONDS(1));
  printk("Starting %s with CPU frequency: %d MHz\n", CONFIG_BOARD,
         SystemCoreClock / MHZ(1));

  video = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));

  if (!device_is_ready(video)) {
    LOG_ERR("%s: video device not ready.", video->name);
    return 0;
  }

  /* Get capabilities */
  if (video_get_caps(video, VIDEO_EP_OUT, &caps)) {
    LOG_ERR("Unable to retrieve video capabilities");
    return 0;
  }

  /* Get default/native format */
  if (video_get_format(video, VIDEO_EP_OUT, &fmt)) {
    LOG_ERR("Unable to retrieve video format");
    return 0;
  }

  printk("Video device detected, format: %s %ux%u\n",
         VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height);

  if (caps.min_line_count != LINE_COUNT_HEIGHT) {
    LOG_ERR("Partial framebuffers not supported by this sample");
    return 0;
  }

  // video_stream_stop(video);
  LOG_INF("Device %s is ready!", video->name);

  net_util_set_callback(socket_rx_callback);

  /* Alloc video buffers and enqueue for capture */
  for (int i = 0; i < ARRAY_SIZE(buffers); i++) {
    buffers[i] = video_buffer_alloc(fmt.pitch * fmt.height, K_FOREVER);
    if (buffers[i] == NULL) {
      LOG_ERR("Unable to alloc video buffer");
      return -1;
    }
    video_enqueue(video, VIDEO_EP_OUT, buffers[i]);
  }

  /* Start video capture */
  if (video_stream_start(video)) {
    LOG_ERR("Unable to start video");
    return 0;
  }

  k_timer_start(&m_timer_count_bytes, K_MSEC(1000), K_MSEC(1000));

  set_mega_resolution(0x11, client_type_NONE);

  // stuck, wait for connection
  while (cam_send(&socket_head_and_tail, 0) == -1) {
    k_sleep(K_SECONDS(1));
  }

  while (1) {
    struct app_command_t new_command;

    if (k_msgq_get(&msgq_app_commands, &new_command, K_USEC(50)) == 0) {
      switch (new_command.type) {
        case APPCMD_TAKE_PICTURE:
          LOG_INF("TAKE PICTURE");
          client_request_single_capture(&client_state_socket);
          break;
        case APPCMD_CAM_COMMAND:
          switch (new_command.cam_cmd) {
            case SET_PICTURE_RESOLUTION:
              LOG_INF("Change resolution to 0x%x", new_command.data[0]);
              set_mega_resolution(new_command.data[0], client_type_BLE);
              break;
          }
          break;
        case APPCMD_START_STOP_STREAM:
          bool enable = new_command.data[0] > 0;

          if (enable) {
            LOG_INF("Starting stream!");
            client_request_stream_start_stop(&client_state_socket, true);
          } else {
            LOG_INF("Stopping stream");
            client_request_stream_start_stop(&client_state_socket, false);
          }
          break;
        case APPCMD_SOCKET_RX:
          uint8_t socket_cmd_buf[CAM_COMMAND_MAX_SIZE - 2];
          ret = process_socket_rx_buffer(new_command.data, socket_cmd_buf);
          if (ret > 0) {
            LOG_INF("Valid command received. Length:%d", ret);
            LOG_HEXDUMP_INF(socket_cmd_buf, ret, "Data: ");
            recv_process(socket_cmd_buf);
          } else {
            LOG_INF("Invalid SOCKET command received:%d", ret);
          }
          break;
      }
    }

    video_preview();
  }
  return 0;
}