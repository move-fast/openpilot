#include <cstdio>
#include <cstdlib>
#include <mutex>

#include <eigen3/Eigen/Dense>

#include "selfdrive/common/mat.h"
#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/common/clutil.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/common/params.h"
#include "selfdrive/hardware/hw.h"


ExitHandler do_exit;

void run_model(VisionIpcClient &vipc_client) {
  // messaging
  SubMaster sm({"roadCameraState"});

  uint32_t frame_id = 0;
  uint32_t run_count = 0;

  while (!do_exit) {
    VisionIpcBufExtra extra = {};
    VisionBuf *buf = vipc_client.recv(&extra);
    if (buf == nullptr) continue;

    // TODO: path planner timeout?
    sm.update(0);
    frame_id = sm["roadCameraState"].getRoadCameraState().getFrameId();

    run_count++;

    double mt1 = millis_since_boot();
    // ModelDataRaw model_buf = model_eval_frame(&model, buf->buf_cl, buf->width, buf->height, model_transform, vec_desire);
    LOGW("ran iteration with buffer size: %d (%d x %d)", buf->len, buf->width, buf->height);



    double mt2 = millis_since_boot();
    float model_execution_time = (mt2 - mt1) / 1000.0;
    LOGW("model_execution_time: %.2f", model_execution_time);
  }
}


int main(int argc, char **argv) {
  set_realtime_priority(54);

  if (Hardware::EON()) {
    set_core_affinity(2);
  } else if (Hardware::TICI()) {
    set_core_affinity(7);  
  }
  bool wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;

  // cl init
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  cl_context context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  // init the models
  VisionIpcClient vipc_client = VisionIpcClient("camerad", wide_camera ? VISION_STREAM_YUV_WIDE : VISION_STREAM_YUV_BACK, true, device_id, context);
  while (!do_exit && !vipc_client.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  // vipc_client.connected is false only when do_exit is true
  if (vipc_client.connected) {
    const VisionBuf *b = &vipc_client.buffers[0];
    LOGW("connected with buffer size: %d (%d x %d)", b->len, b->width, b->height);
    run_model(vipc_client);
  }

  CL_CHECK(clReleaseContext(context));
  return 0;
}
