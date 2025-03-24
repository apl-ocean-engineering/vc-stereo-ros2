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

// static std::string name2var(const char *name) {
//   std::string s;
//   int add_underscore = 0;

//   while (*name) {
//     if (isalnum(*name)) {
//       if (add_underscore) s += '_';
//       add_underscore = 0;
//       s += std::string(1, tolower(*name));
//     } else if (s.length())
//       add_underscore = 1;
//     name++;
//   }
//   return s;
// }

// static std::string safename(const unsigned char *name) {
//   std::string s;

//   while (*name) {
//     if (*name == '\n') {
//       s += "\\n";
//     } else if (*name == '\r') {
//       s += "\\r";
//     } else if (*name == '\f') {
//       s += "\\f";
//     } else if (*name == '\\') {
//       s += "\\\\";
//     } else if ((*name & 0x7f) < 0x20) {
//       char buf[3];

//       sprintf(buf, "%02x", *name);
//       s += "\\x";
//       s += buf;
//     } else {
//       s += *name;
//     }
//     name++;
//   }
//   return s;
// }

// static std::string safename(const char *name) {
//   return safename((const unsigned char *)name);
// }

// static void print_qctrl(int fd, struct v4l2_query_ext_ctrl *queryctrl,
//                         struct v4l2_ext_control *ctrl, int show_menus) {
//   struct v4l2_querymenu qmenu;
//   std::string s = name2var(queryctrl->name);
//   unsigned i;

//   memset(&qmenu, 0, sizeof(qmenu));
//   qmenu.id = queryctrl->id;
//   switch (queryctrl->type) {
//     case V4L2_CTRL_TYPE_INTEGER:
//       printf("%31s %#8.8x (int)    : min=%lld max=%lld step=%lld
//       default=%lld",
//              s.c_str(), queryctrl->id, queryctrl->minimum,
//              queryctrl->maximum, queryctrl->step, queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_INTEGER64:
//       printf("%31s %#8.8x (int64)  : min=%lld max=%lld step=%lld
//       default=%lld",
//              s.c_str(), queryctrl->id, queryctrl->minimum,
//              queryctrl->maximum, queryctrl->step, queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_STRING:
//       printf("%31s %#8.8x (str)    : min=%lld max=%lld step=%lld", s.c_str(),
//              queryctrl->id, queryctrl->minimum, queryctrl->maximum,
//              queryctrl->step);
//       break;
//     case V4L2_CTRL_TYPE_BOOLEAN:
//       printf("%31s %#8.8x (bool)   : default=%lld", s.c_str(), queryctrl->id,
//              queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_MENU:
//       printf("%31s %#8.8x (menu)   : min=%lld max=%lld default=%lld",
//       s.c_str(),
//              queryctrl->id, queryctrl->minimum, queryctrl->maximum,
//              queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_INTEGER_MENU:
//       printf("%31s %#8.8x (intmenu): min=%lld max=%lld default=%lld",
//       s.c_str(),
//              queryctrl->id, queryctrl->minimum, queryctrl->maximum,
//              queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_BUTTON:
//       printf("%31s %#8.8x (button) :", s.c_str(), queryctrl->id);
//       break;
//     case V4L2_CTRL_TYPE_BITMASK:
//       printf("%31s %#8.8x (bitmask): max=0x%08llx default=0x%08llx",
//       s.c_str(),
//              queryctrl->id, queryctrl->maximum, queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_U8:
//       printf("%31s %#8.8x (u8)     : min=%lld max=%lld step=%lld
//       default=%lld",
//              s.c_str(), queryctrl->id, queryctrl->minimum,
//              queryctrl->maximum, queryctrl->step, queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_U16:
//       printf("%31s %#8.8x (u16)    : min=%lld max=%lld step=%lld
//       default=%lld",
//              s.c_str(), queryctrl->id, queryctrl->minimum,
//              queryctrl->maximum, queryctrl->step, queryctrl->default_value);
//       break;
//     case V4L2_CTRL_TYPE_U32:
//       printf("%31s %#8.8x (u32)    : min=%lld max=%lld step=%lld
//       default=%lld",
//              s.c_str(), queryctrl->id, queryctrl->minimum,
//              queryctrl->maximum, queryctrl->step, queryctrl->default_value);
//       break;
//     default:
//       printf("%31s %#8.8x (unknown): type=%x", s.c_str(), queryctrl->id,
//              queryctrl->type);
//       break;
//   }
//   if (queryctrl->nr_of_dims == 0) {
//     switch (queryctrl->type) {
//       case V4L2_CTRL_TYPE_INTEGER:
//       case V4L2_CTRL_TYPE_BOOLEAN:
//       case V4L2_CTRL_TYPE_MENU:
//       case V4L2_CTRL_TYPE_INTEGER_MENU:
//         printf(" value=%d", ctrl->value);
//         break;
//       case V4L2_CTRL_TYPE_BITMASK:
//         printf(" value=0x%08x", ctrl->value);
//         break;
//       case V4L2_CTRL_TYPE_INTEGER64:
//         printf(" value=%lld", ctrl->value64);
//         break;
//       case V4L2_CTRL_TYPE_STRING:
//         printf(" value='%s'", safename(ctrl->string).c_str());
//         break;
//       default:
//         break;
//     }
//   }
//   if (queryctrl->nr_of_dims) {
//     printf(" ");
//     for (i = 0; i < queryctrl->nr_of_dims; i++)
//       printf("[%u]", queryctrl->dims[i]);
//   }
//   // if (queryctrl->flags)
//   //    printf(" flags=%s", ctrlflags2s(queryctrl->flags).c_str());
//   printf("\n");
//   if ((queryctrl->type == V4L2_CTRL_TYPE_MENU ||
//        queryctrl->type == V4L2_CTRL_TYPE_INTEGER_MENU) &&
//       show_menus) {
//     for (i = queryctrl->minimum; i <= queryctrl->maximum; i++) {
//       qmenu.index = i;
//       if (ioctl(fd, VIDIOC_QUERYMENU, &qmenu)) continue;
//       if (queryctrl->type == V4L2_CTRL_TYPE_MENU)
//         printf("\t\t\t\t%d: %s\n", i, qmenu.name);
//       else
//         printf("\t\t\t\t%d: %lld (0x%llx)\n", i, qmenu.value, qmenu.value);
//     }
//   }
// }

// static int print_control(int fd, struct v4l2_query_ext_ctrl &qctrl,
//                          int show_menus) {
//   struct v4l2_control ctrl;
//   struct v4l2_ext_control ext_ctrl;
//   struct v4l2_ext_controls ctrls;

//   memset(&ctrl, 0, sizeof(ctrl));
//   memset(&ext_ctrl, 0, sizeof(ext_ctrl));
//   memset(&ctrls, 0, sizeof(ctrls));
//   if (qctrl.flags & V4L2_CTRL_FLAG_DISABLED) return 1;
//   if (qctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS) {
//     printf("\n%s\n\n", qctrl.name);
//     return 1;
//   }
//   ext_ctrl.id = qctrl.id;
//   if ((qctrl.flags & V4L2_CTRL_FLAG_WRITE_ONLY) ||
//       qctrl.type == V4L2_CTRL_TYPE_BUTTON) {
//     print_qctrl(fd, &qctrl, &ext_ctrl, show_menus);
//     return 1;
//   }
//   if (qctrl.type >= V4L2_CTRL_COMPOUND_TYPES) {
//     print_qctrl(fd, &qctrl, NULL, show_menus);
//     return 1;
//   }
//   ctrls.which = V4L2_CTRL_ID2WHICH(qctrl.id);
//   ctrls.count = 1;
//   ctrls.controls = &ext_ctrl;
//   if (qctrl.type == V4L2_CTRL_TYPE_INTEGER64 ||
//       qctrl.type == V4L2_CTRL_TYPE_STRING ||
//       (V4L2_CTRL_ID2WHICH(qctrl.id) != V4L2_CTRL_CLASS_USER &&
//        qctrl.id < V4L2_CID_PRIVATE_BASE)) {
//     if (qctrl.type == V4L2_CTRL_TYPE_STRING) {
//       ext_ctrl.size = qctrl.maximum + 1;
//       ext_ctrl.string = (char *)malloc(ext_ctrl.size);
//       ext_ctrl.string[0] = 0;
//     }
//     if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls)) {
//       printf("error %d getting ext_ctrl %s\n", errno, qctrl.name);
//       return 0;
//     }
//   } else {
//     ctrl.id = qctrl.id;
//     if (ioctl(fd, VIDIOC_G_CTRL, &ctrl)) {
//       printf("error %d getting ctrl %s\n", errno, qctrl.name);
//       return 0;
//     }
//     ext_ctrl.value = ctrl.value;
//   }
//   print_qctrl(fd, &qctrl, &ext_ctrl, show_menus);
//   if (qctrl.type == V4L2_CTRL_TYPE_STRING) free(ext_ctrl.string);
//   return 1;
// }

V4LDevice::V4LDevice(const std::string &device)
    : device_(device), trigger_mode_v4l2_id_(-1) {
  initializeV4L2Ctrls();

  //     memset(&queryctrl, 0, sizeof(queryctrl));

  //     for (queryctrl.id = V4L2_CID_USER_BASE;
  //         queryctrl.id < V4L2_CID_LASTP1;
  //         queryctrl.id++) {

  //             std::cerr << "Control " << std::hex << queryctrl.id <<
  //             std::endl;
  //        if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
  //            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  //                continue;

  //            std::cerr << "Control: " << queryctrl.name << std::endl;

  //            if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
  //             std::cerr<<"  Menu items:" << std::endl;

  //             memset(&querymenu, 0, sizeof(querymenu));
  //             querymenu.id = queryctrl.id;

  //             for (querymenu.index = queryctrl.minimum;
  //                 querymenu.index <= queryctrl.maximum;
  //                 querymenu.index++) {
  //                 if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
  //                     std::cerr << "  %s" << querymenu.name << std::endl;
  //                 }
  //             }
  //         }
  //        } else {
  //            if (errno == EINVAL)
  //                continue;

  //            perror("VIDIOC_QUERYCTRL");
  //            exit(EXIT_FAILURE);
  //        }
  //    }

  //    for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;
  //         queryctrl.id++) {
  //             std::cerr << "Private " << std::hex << queryctrl.id <<
  //             std::endl;

  //        if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
  //            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  //                continue;

  //                std::cerr << "Control: " << queryctrl.name << std::endl;

  //         if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
  //             std::cerr<<"  Menu items:" << std::endl;

  //             memset(&querymenu, 0, sizeof(querymenu));
  //             querymenu.id = queryctrl.id;

  //             for (querymenu.index = queryctrl.minimum;
  //                 querymenu.index <= queryctrl.maximum;
  //                 querymenu.index++) {
  //                 if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
  //                     std::cerr << "  %s" << querymenu.name << std::endl;
  //                 }
  //             }
  //         }
  //        } else {
  //            if (errno == EINVAL)
  //                break;

  //            perror("VIDIOC_QUERYCTRL");
  //            exit(EXIT_FAILURE);
  //        }
  //    }

  // queryctrl.id = V4L2_CTRL_CLASS_USER | V4L2_CTRL_FLAG_NEXT_CTRL;
  // while (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
  //     if (V4L2_CTRL_ID2CLASS(queryctrl.id) != V4L2_CTRL_CLASS_USER)
  //         break;
  //     if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  //         continue;

  //     std::cerr << "Control " << queryctrl.name << std::endl;

  //     if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
  //         std::cerr<<"  Menu items:" << std::endl;

  //         memset(&querymenu, 0, sizeof(querymenu));
  //         querymenu.id = queryctrl.id;

  //         for (querymenu.index = queryctrl.minimum;
  //             querymenu.index <= queryctrl.maximum;
  //             querymenu.index++) {
  //             if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
  //                 std::cerr << "  %s" << querymenu.name << std::endl;
  //             }
  //         }
  //     }

  //     queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  // }
  // if (errno != EINVAL) {
  //     perror("VIDIOC_QUERYCTRL");
  //     exit(EXIT_FAILURE);
  // }

  //  close(fd);
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

    std::cerr << qctrl.name << std::endl;

    if (strncmp(qctrl.name, "Trigger Mode", 12) == 0) {
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

  if (trigger_type == TriggerType::External)
    extCtrl.value = 1;
  else
    extCtrl.value = 0;

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
