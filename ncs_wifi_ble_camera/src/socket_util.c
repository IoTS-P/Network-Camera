/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(socket_util, CONFIG_LOG_DEFAULT_LEVEL);

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/types.h>
/* Macro called upon a fatal error, reboots the device. */
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

#include "socket_util.h"

#define FATAL_ERROR()                            \
  LOG_ERR("Fatal error! Rebooting the device."); \
  LOG_PANIC();                                   \
  IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

/* size of stack area used by each thread */
#define STACKSIZE 8192
/* scheduling priority used by each thread */
#define PRIORITY 3
/* Same as COMMAND_MAX_SIZE*/
#define BUFFER_MAX_SIZE 6

// #define pc_port  60000
#define cam_port 60010  // use for either udp or tcp server

int udp_socket;
int tcp_server_listen_fd;
int tcp_server_socket = -1;
struct sockaddr_in cam_addr;
bool socket_connected = false;

char pc_addr_str[32];
struct sockaddr_in pc_addr;
socklen_t pc_addr_len = sizeof(pc_addr);

/* Define the semaphore net_ready */
struct k_sem net_ready;

uint8_t socket_recv_buf[BUFFER_MAX_SIZE];
K_MSGQ_DEFINE(socket_recv_queue, sizeof(socket_recv_buf), 1, 4);

static net_util_socket_rx_callback_t socket_rx_cb = 0;

void net_util_set_callback(net_util_socket_rx_callback_t socket_rx_callback) {
  socket_rx_cb = socket_rx_callback;
  // If any messages are waiting in the queue, forward them immediately
  uint8_t buf[6];
  while (k_msgq_get(&socket_recv_queue, buf, K_NO_WAIT) == 0) {
    socket_rx_cb(buf, 6);
    LOG_HEXDUMP_INF(buf, 6, "socket_rx_cb");
  }
}

uint8_t process_socket_rx_buffer(char *socket_rx_buf, char *command_buf) {
  uint8_t command_length = 0;

  // Check if socket_rx_buf contains start code (0x55)
  if (socket_rx_buf[0] == 0x55) {
    // Find the end code (0xAA) and extract the command
    for (uint8_t i = 1; i < CAM_COMMAND_MAX_SIZE; i++) {
      if (socket_rx_buf[i] == 0xAA) {
        // Copy the command to command_buf
        for (uint8_t j = 1; j < i; j++) {
          command_buf[j - 1] = socket_rx_buf[j];
        }
        command_length = i - 1;  // Length of the command
        break;
      }
    }
  }
  return command_length;
}

static void trigger_socket_rx_callback_if_set() {
  if (socket_rx_cb != 0) {
    socket_rx_cb(socket_recv_buf, 6);
  } else {
    k_msgq_put(&socket_recv_queue, &socket_recv_buf, K_NO_WAIT);
  }
}

static ssize_t sendall(int sock, const void *buf, size_t len)
{
	while (len) {
		ssize_t out_len = send(sock, buf, len, 0);

		if (out_len < 0) {
			return out_len;
		}
		buf = (const char *)buf + out_len;
		len -= out_len;
	}

	return 0;
}

int cam_send(const void *buf, size_t len) {
#if defined(CONFIG_NET_TCP)
  if (tcp_server_socket == -1) {
    return -1;
  }
  if (len == 0) {
    return 0;
  }
  int ret = sendall(tcp_server_socket, buf, len);
  if (ret && ret != -EAGAIN) {
    printk("Sending failed %d\n", ret);
    close(tcp_server_socket);
    FATAL_ERROR();
    return -2;
  }
#else
  sendto(udp_socket, buf, len, 0, (struct sockaddr *)&pc_addr, sizeof(pc_addr));
#endif
  return 0;
}

/* Thread to setup Network, Sockets step by step */
static void net_sockets(void) {
  int ret;
  ssize_t bytes_recived;

  k_sem_init(&net_ready, 0, 1);
  k_sleep(K_SECONDS(3));

  (void)memset(&cam_addr, 0, sizeof(cam_addr));
  cam_addr.sin_family = AF_INET;
  cam_addr.sin_port = htons(cam_port);

#if defined(CONFIG_NET_TCP)
  tcp_server_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (tcp_server_listen_fd < 0) {
    LOG_ERR("Failed to create socket: %d", -errno);
    FATAL_ERROR();
    return;
  }

  ret = bind(tcp_server_listen_fd, (struct sockaddr *)&cam_addr,
             sizeof(cam_addr));
  if (ret < 0) {
    LOG_ERR("bind, error: %d", -errno);
    FATAL_ERROR();
    return;
  }

  ret = listen(tcp_server_listen_fd, 5);
  if (ret < 0) {
    LOG_ERR("listen, error: %d", -errno);
    FATAL_ERROR();
    return;
  }

  while (1) {
    printk("TCP: Waiting for client....\n");
    tcp_server_socket =
        accept(tcp_server_listen_fd, (struct sockaddr *)&pc_addr, &pc_addr_len);
    if (tcp_server_socket < 0) {
      LOG_ERR("accept, error: %d", -errno);
      FATAL_ERROR();
      return;
    }
    LOG_INF("Accepted connection from client\n");
    inet_ntop(pc_addr.sin_family, &pc_addr.sin_addr, pc_addr_str,
              sizeof(pc_addr_str));
    LOG_INF("Connect with PC through Ethernet, PC IPAddr = %s, Port = %d\n",
            pc_addr_str, ntohs(pc_addr.sin_port));
    // Handle the client connection
    while ((bytes_recived = recv(tcp_server_socket, socket_recv_buf,
                                 sizeof(socket_recv_buf), 0)) > 0) {
      trigger_socket_rx_callback_if_set(socket_recv_buf);
    }
    if (bytes_recived == -1) {
      LOG_ERR("Receiving failed");
    } else if (bytes_recived == 0) {
      LOG_INF("Client disconnected.\n");
    }
    close(tcp_server_socket);
  }
  // Close the server socket (should be unreachable  as the server runs
  // indefinitely)
  close(tcp_server_listen_fd);
#else
  udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_socket < 0) {
    LOG_ERR("Failed to create socket: %d", -errno);
    FATAL_ERROR();
    return;
  }
  ret = bind(udp_socket, (struct sockaddr *)&cam_addr, sizeof(cam_addr));
  if (ret < 0) {
    LOG_ERR("bind, error: %d", -errno);
    FATAL_ERROR();
    return;
  }
  while ((bytes_recived =
              recvfrom(udp_socket, socket_recv_buf, sizeof(socket_recv_buf), 0,
                       (struct sockaddr *)&pc_addr, &pc_addr_len)) > 0) {
    if (socket_connected == false) {
      inet_ntop(pc_addr.sin_family, &pc_addr.sin_addr, pc_addr_str,
                sizeof(pc_addr_str));
      LOG_INF("Connect with PC through Ethernet, PC IPAddr = %s, Port = %d\n",
              pc_addr_str, ntohs(pc_addr.sin_port));
      socket_connected = true;
    }
    trigger_socket_rx_callback_if_set(socket_recv_buf);
  }
  if (bytes_recived == -1) {
    LOG_ERR("Receiving failed");
  } else if (bytes_recived == 0) {
    LOG_INF("Client disconnected.\n");
  }

  close(udp_socket);
  socket_connected = false;

#endif
}

K_THREAD_DEFINE(net_sockets_id, STACKSIZE, net_sockets, NULL, NULL, NULL,
                PRIORITY, 0, 0);