//
//
// Copyright 2025 University of Washington

#include "vc_argus_ros2/v4l_device.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

namespace vc_argus_ros2 {

static int xioctl(int fd, int request, void *arg) {
  int r;

  const int IOCTL_TRIES = 3;
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

// V4L names are strings with spaces replaced underscores
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

//===================================================================

V4LDevice::V4LDevice(const std::string &devnode)
    : device_(devnode), trigger_mode_v4l2_id_(-1) {
  initializeV4L2CtrlIds();
}

bool V4LDevice::initializeV4L2CtrlIds() {
  auto fd = open(device_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << "Failed to open the camera" << std::endl;
    return false;
  }

  const unsigned next_fl =
      V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  struct v4l2_query_ext_ctrl qctrl;
  int id;

  memset(&qctrl, 0, sizeof(qctrl));
  qctrl.id = next_fl;
  while (query_ext_ctrl_ioctl(fd, &qctrl) == 0) {
    auto id_name = name2var(qctrl.name);

    if (strncmp(id_name.c_str(), "trigger_mode", 12) == 0) {
      trigger_mode_v4l2_id_ = qctrl.id;
    }

    qctrl.id |= next_fl;
  }

  // Have we successfully found all of the ids we need?
  if (trigger_mode_v4l2_id_ > 0) return true;

  return false;
}

// As we need to access more v4l controls, this could be refactored into a more
// generic setter function
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

  struct v4l2_ext_controls ext_ctrls;
  memset(&ext_ctrls, 0, sizeof(struct v4l2_ext_controls));
  ext_ctrls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
  ext_ctrls.which = V4L2_CTRL_WHICH_CUR_VAL;
  ext_ctrls.count = 1;

  struct v4l2_ext_control ext_ctrl;
  memset(&ext_ctrl, 0, sizeof(struct v4l2_ext_control));
  ext_ctrls.controls = &ext_ctrl;

  ext_ctrl.id = trigger_mode_v4l2_id_;
  ext_ctrl.size = sizeof(int);

  if (trigger_type == TriggerType::External) {
    ext_ctrl.value = 1;
  } else {
    ext_ctrl.value = 0;
  }

  if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls) != 0) {
    std::cerr << "Failed to set trigger"
              << "\nerror (" << errno << "): " << strerror(errno) << std::endl;

    close(fd);
    return false;
  }

  close(fd);
  return true;
}

}  // namespace vc_argus_ros2
