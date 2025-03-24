//
//
// Copyright 2025 University of Washington

#include "argus_stereo_sync/v4l_device.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

namespace argus_stereo_sync {

#define IOCTL_TRIES 3
#define CLEAR(x) memset(&(x), 0, sizeof(x))

static int xioctl(int fd, int request, void *arg) {
  int r;
  int tries = IOCTL_TRIES;

  do {
    r = ioctl(fd, request, arg);
  } while (--tries > 0 && r == -1 && EINTR == errno);

  return r;
}

static int query_ext_ctrl_ioctl(int fd, struct v4l2_query_ext_ctrl *qctrl) {
  struct v4l2_queryctrl qc;
  int rc;

  if (true) {
    rc = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, qctrl);
    if (errno != ENOTTY) return rc;
  }
  qc.id = qctrl->id;
  rc = ioctl(fd, VIDIOC_QUERYCTRL, &qc);

  if (rc == 0) {
    qctrl->type = qc.type;
    memcpy(qctrl->name, qc.name, sizeof(qctrl->name));
    qctrl->minimum = qc.minimum;
    if (qc.type == V4L2_CTRL_TYPE_BITMASK) {
      qctrl->maximum = (__u32)qc.maximum;
      qctrl->default_value = (__u32)qc.default_value;
    } else {
      qctrl->maximum = qc.maximum;
      qctrl->default_value = qc.default_value;
    }
    qctrl->step = qc.step;
    qctrl->flags = qc.flags;
    qctrl->elems = 1;
    qctrl->nr_of_dims = 0;
    memset(qctrl->dims, 0, sizeof(qctrl->dims));
    switch (qctrl->type) {
      case V4L2_CTRL_TYPE_INTEGER64:
        qctrl->elem_size = sizeof(__s64);
        break;
      case V4L2_CTRL_TYPE_STRING:
        qctrl->elem_size = qc.maximum + 1;
        break;
      default:
        qctrl->elem_size = sizeof(__s32);
        break;
    }
    memset(qctrl->reserved, 0, sizeof(qctrl->reserved));
  }
  qctrl->id = qc.id;
  return rc;
}

static std::string name2var(const char *name) {
  std::string s;
  int add_underscore = 0;

  while (*name) {
    if (isalnum(*name)) {
      if (add_underscore) s += '_';
      add_underscore = 0;
      s += std::string(1, tolower(*name));
    } else if (s.length()) {
      add_underscore = 1;
    }
    name++;
  }
  return s;
}

V4LDevice::V4LDevice(const std::string &device)
    : device_(device), trigger_mode_v4l2_id_(-1) {
  initializeV4L2Ctrls();
}

V4LDevice::~V4LDevice() {}

bool V4LDevice::initializeV4L2Ctrls() {
  std::cout << "Querying controls for " << device_ << std::endl;

  int fd = open(device_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    std::cout << "Failed to open the camera" << std::endl;
    return false;
  }

  const int show_menus = 0;

  const unsigned next_fl =
      V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  struct v4l2_query_ext_ctrl qctrl;
  int id;

  memset(&qctrl, 0, sizeof(qctrl));
  qctrl.id = next_fl;
  while (query_ext_ctrl_ioctl(fd, &qctrl) == 0) {
    // print_control(fd, qctrl, show_menus);
    auto id_name = name2var(qctrl.name);

    if (strncmp(id_name.c_str(), "trigger_mode", 12) == 0) {
      std::cerr << "Found \"trigger_mode\" at id " << std::hex << qctrl.id
                << std::endl;
      trigger_mode_v4l2_id_ = qctrl.id;
    }

    qctrl.id |= next_fl;
  }

  // For now, we know the relevant controls are in the _ext_ended list,
  // so don't query these non-extended
  // if (qctrl.id != next_fl) return;
  // for (id = V4L2_CID_USER_BASE; id < V4L2_CID_LASTP1; id++) {
  //   qctrl.id = id;
  //   if (query_ext_ctrl_ioctl(fd, qctrl) == 0)
  //     print_control(fd, qctrl, show_menus);
  // }

  // for (qctrl.id = V4L2_CID_PRIVATE_BASE; query_ext_ctrl_ioctl(fd, qctrl) ==
  // 0;
  //      qctrl.id++) {
  //   print_control(fd, qctrl, show_menus);
  // }

  // On success
  if (trigger_mode_v4l2_id_ > 0) return true;

  return false;
}

bool V4LDevice::setTrigger(const TriggerType trigger_type) {
  if (trigger_mode_v4l2_id_ < 0) {
    std::cerr << "Could not find trigger_mode V4L2 control, cannot set trigger"
              << std::endl;
    return false;
  }

  int fd = open(device_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    std::cout << "Failed to open the camera" << std::endl;
    return -1;
  }

  struct v4l2_ext_controls extCtrls;
  CLEAR(extCtrls);
  extCtrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
  extCtrls.which = V4L2_CTRL_WHICH_CUR_VAL;
  extCtrls.count = 1;

  struct v4l2_ext_control extCtrl;
  CLEAR(extCtrl);
  extCtrls.controls = &extCtrl;

  extCtrl.id = trigger_mode_v4l2_id_;
  extCtrl.size = sizeof(int);

  if (trigger_type == TriggerType::External) {
    extCtrl.value = 1;
  } else {
    extCtrl.value = 0;
  }

  std::cout << "Setting trigger mode to " << extCtrl.value << std::endl;

  if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrls) != 0) {
    std::cout << "Failed to set trigger"
              << "\nerror (" << errno << "): " << strerror(errno) << std::endl;

    close(fd);

    return false;
  }

  close(fd);
  return true;
}

// bool V4LDevice::initializeV4L2Ctrls(uint32_t id, int val) {
//   int fd = open(device_.c_str(), O_RDWR | O_NONBLOCK);
//   if (fd == -1) {
//     std::cout << "Failed to open the camera" << std::endl;
//     return -1;
//   }

//   struct v4l2_control ctrl;
//   CLEAR(ctrl);

//   ctrl.id = id;
//   ctrl.value = val;
//   if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
//     std::cout << "Failed to set ctrl with id " << id << " to value " << val
//               << "\nerror (" << errno << "): " << strerror(errno) <<
//               std::endl;

//     close(fd);

//     return false;
//   }

//   close(fd);
//   return true;
// }

}  // namespace argus_stereo_sync
