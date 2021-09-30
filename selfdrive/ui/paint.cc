#include "selfdrive/ui/paint.h"

#include <cassert>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/ui.h"

static void ui_draw_text(const UIState *s, float x, float y, const char *string, float size, NVGcolor color, const char *font_name) {
  nvgFontFace(s->vg, font_name);
  nvgFontSize(s->vg, size);
  nvgFillColor(s->vg, color);
  nvgText(s->vg, x, y, string, NULL);
}

static void ui_draw_circle(UIState *s, float x, float y, float size, NVGcolor color) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, x, y, size);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
}

static void ui_draw_speed_sign(UIState *s, float x, float y, int size, float speed, const char *subtext, 
                               float subtext_size, const char *font_name, bool is_map_sourced, bool is_active) {
  NVGcolor ring_color = is_active ? COLOR_RED : COLOR_BLACK_ALPHA(.2f * 255);
  NVGcolor inner_color = is_active ? COLOR_WHITE : COLOR_WHITE_ALPHA(.35f * 255);
  NVGcolor text_color = is_active ? COLOR_BLACK : COLOR_BLACK_ALPHA(.3f * 255);

  ui_draw_circle(s, x, y, float(size), ring_color);
  ui_draw_circle(s, x, y, float(size) * 0.8, inner_color);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

  const std::string speedlimit_str = std::to_string((int)std::nearbyint(speed));
  ui_draw_text(s, x, y, speedlimit_str.c_str(), 120, text_color, font_name);
  ui_draw_text(s, x, y + 55, subtext, subtext_size, text_color, font_name);

  if (is_map_sourced) {
    const int img_size = 35;
    const int img_y = int(y - 55);
    ui_draw_image(s, {int(x - (img_size / 2)), img_y - (img_size / 2), img_size, img_size}, "map_source_icon", 
                  is_active ? 1. : .3);
  }
}

static void draw_chevron(UIState *s, float x, float y, float sz, NVGcolor fillColor, NVGcolor glowColor) {
  // glow
  float g_xo = sz/5;
  float g_yo = sz/10;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, NVGcolor color, float img_alpha) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  const int img_size = radius * 1.5;
  ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(s, center_x, center_y, radius, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}

static void draw_lead(UIState *s, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const vertex_data &vd) {
  // Draw lead car indicator
  auto [x, y] = vd;

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  float d_rel = lead_data.getX()[0];
  float v_rel = lead_data.getV()[0];
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  x = std::clamp(x, 0.f, s->fb_w - sz / 2);
  y = std::fmin(s->fb_h - sz * .6, y);
  draw_chevron(s, x, y, sz, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
}

static void ui_draw_line(UIState *s, const line_vertices_data &vd, NVGcolor *color, NVGpaint *paint) {
  if (vd.cnt == 0) return;

  const vertex_data *v = &vd.v[0];
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < vd.cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene &scene = s->scene;
  NVGpaint track_bg;
  if (!scene.end_to_end) {
    // paint lanelines
    for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 1.0, 1.0, scene.lane_line_probs[i]);
      ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
    }

    // paint road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
      ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
    }
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE, COLOR_WHITE_ALPHA(0));
  } else {
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_RED, COLOR_RED_ALPHA(0));
  }
  // paint path
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  if (s->scene.longitudinal_control) {
    auto lead_one = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[0];
    auto lead_two = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[1];
    if (lead_one.getProb() > .5) {
      draw_lead(s, lead_one, s->scene.lead_vertices[0]);
    }
    if (lead_two.getProb() > .5 && (std::abs(lead_one.getX()[0] - lead_two.getX()[0]) > 3.0)) {
      draw_lead(s, lead_two, s->scene.lead_vertices[1]);
    }
  }
  nvgResetScissor(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = (*s->sm)["controlsState"].getControlsState().getVCruise();
  const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
  if (is_cruise_set && !s->scene.is_metric) { maxspeed *= 0.6225; }

  const Rect rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, rect.centerX(), 118, "MAX", 26 * 2.5, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), "sans-regular");
  if (is_cruise_set) {
    const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
    ui_draw_text(s, rect.centerX(), 212, maxspeed_str.c_str(), 48 * 2.5, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX(), 212, "N/A", 42 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}

static void ui_draw_vision_speedlimit(UIState *s) {
  auto longitudinal_plan = (*s->sm)["longitudinalPlan"].getLongitudinalPlan();
  const float speedLimit = longitudinal_plan.getSpeedLimit();
  const float speedLimitOffset = longitudinal_plan.getSpeedLimitOffset();

  if (speedLimit > 0.0 && s->scene.engageable) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    const Rect speed_sign_rect = {maxspeed_rect.centerX() - speed_sgn_r, maxspeed_rect.bottom() + bdr_s,
                                  2 * speed_sgn_r, 2 * speed_sgn_r};
    const float speed = speedLimit * (s->scene.is_metric ? 3.6 : 2.2369362921);
    const float speed_offset = speedLimitOffset * (s->scene.is_metric ? 3.6 : 2.2369362921);

    auto speedLimitControlState = longitudinal_plan.getSpeedLimitControlState();
    const bool force_active = s->scene.speed_limit_control_enabled && 
                              seconds_since_boot() < s->scene.last_speed_limit_sign_tap + 2.0;
    const bool inactive = !force_active && (!s->scene.speed_limit_control_enabled || 
                          speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::INACTIVE);
    const bool temp_inactive = !force_active && (s->scene.speed_limit_control_enabled && 
                               speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE);

    const int distToSpeedLimit = int(longitudinal_plan.getDistToSpeedLimit() * 
                                     (s->scene.is_metric ? 1.0 : 3.28084) / 10.0) * 10;
    const bool is_map_sourced = longitudinal_plan.getIsMapSpeedLimit();
    const std::string distance_str = std::to_string(distToSpeedLimit) + (s->scene.is_metric ? "m" : "f");
    const std::string offset_str = speed_offset > 0.0 ? "+" + std::to_string((int)std::nearbyint(speed_offset)) : "";
    const std::string inactive_str = temp_inactive ? "TEMP" : "";
    const std::string substring = inactive || temp_inactive ? inactive_str : 
                                                              distToSpeedLimit > 0 ? distance_str : offset_str;
    const float substring_size = inactive || temp_inactive || distToSpeedLimit > 0 ? 30.0 : 50.0;

    ui_draw_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), speed_sgn_r, speed, substring.c_str(), 
                       substring_size, "sans-bold", is_map_sourced, !inactive && !temp_inactive);

    s->scene.speed_limit_sign_touch_rect = Rect{speed_sign_rect.x - speed_sgn_touch_pad, 
                                                speed_sign_rect.y - speed_sgn_touch_pad,
                                                speed_sign_rect.w + 2 * speed_sgn_touch_pad, 
                                                speed_sign_rect.h + 2 * speed_sgn_touch_pad};
  }
}

static void ui_draw_vision_speed(UIState *s) {
  const float speed = std::max(0.0, (*s->sm)["carState"].getCarState().getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363));
  const std::string speed_str = std::to_string((int)std::nearbyint(speed));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, s->fb_w/2, 210, speed_str.c_str(), 96 * 2.5, COLOR_WHITE, "sans-bold");
  ui_draw_text(s, s->fb_w/2, 290, s->scene.is_metric ? "km/h" : "mph", 36 * 2.5, COLOR_WHITE_ALPHA(200), "sans-regular");
}

static void ui_draw_vision_event(UIState *s) {
  auto longitudinal_plan = (*s->sm)["longitudinalPlan"].getLongitudinalPlan();
  auto visionTurnControllerState = longitudinal_plan.getVisionTurnControllerState();
  if (s->scene.show_debug_ui &&
      visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED && 
      s->scene.engageable) {
    // draw a rectangle with colors indicating the state with the value of the acceleration inside.
    const int size = 184;
    const Rect rect = {s->fb_w - size - bdr_s, int(bdr_s * 1.5), size, size};
    ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);

    auto source = longitudinal_plan.getLongitudinalPlanSource();
    const int alpha = source == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN ? 255 : 100;
    const QColor &color = tcs_colors[int(visionTurnControllerState)];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), alpha);
    ui_draw_rect(s->vg, rect, nvg_color, 10, 20.);
    
    const float vision_turn_speed = longitudinal_plan.getVisionTurnSpeed() * (s->scene.is_metric ? 3.6 : 2.2369363);
    std::string acc_str = std::to_string((int)std::nearbyint(vision_turn_speed));
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    ui_draw_text(s, rect.centerX(), rect.centerY(), acc_str.c_str(), 56, COLOR_WHITE_ALPHA(alpha), "sans-bold");
  } else if (s->scene.engageable) {
    // draw steering wheel
    const int radius = 96;
    const int center_x = s->fb_w - radius - bdr_s * 2;
    const int center_y = radius  + (bdr_s * 1.5);
    const QColor &color = bg_colors[s->status];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
    ui_draw_circle_image(s, center_x, center_y, radius, "wheel", nvg_color, 1.0f);

    // draw hands on wheel pictogram under wheel pictogram.
    auto handsOnWheelState = (*s->sm)["driverMonitoringState"].getDriverMonitoringState().getHandsOnWheelState();
    if (handsOnWheelState >= cereal::DriverMonitoringState::HandsOnWheelState::WARNING) {
      NVGcolor color = COLOR_RED;
      if (handsOnWheelState == cereal::DriverMonitoringState::HandsOnWheelState::WARNING) {
        color = COLOR_YELLOW;
      } 
      const int wheel_y = center_y + bdr_s + 2 * radius;
      ui_draw_circle_image(s, center_x, wheel_y, radius, "hands_on_wheel", color, 1.0f);
    }
  }
}

static void ui_draw_vision_bottom_bar(UIState *s) {
  auto liveMapData = (*s->sm)["liveMapData"].getLiveMapData();
  const std::string road_name = liveMapData.getCurrentRoadName();

  if (!road_name.empty() && s->scene.show_debug_ui) {
    const int h = 60;
    const Rect rect = {0, s->fb_h - h, s->fb_w, h};
    ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 0.);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    ui_draw_text(s, rect.centerX(), rect.centerY(), road_name.c_str(), 52., COLOR_WHITE_ALPHA(200), "sans-bold");
  }
}

static void ui_draw_vision_face(UIState *s) {
  const int radius = 96;
  const int center_x = radius + (bdr_s * 2);
  const int center_y = s->fb_h - footer_h / 2;
  ui_draw_circle_image(s, center_x, center_y, radius, "driver_face", s->scene.dm_active);
}

static void ui_draw_vision_header(UIState *s) {
  NVGpaint gradient = nvgLinearGradient(s->vg, 0, header_h - (header_h / 2.5), 0, header_h,
                                        nvgRGBAf(0, 0, 0, 0.45), nvgRGBAf(0, 0, 0, 0));
  ui_fill_rect(s->vg, {0, 0, s->fb_w , header_h}, gradient);
  ui_draw_vision_maxspeed(s);
  ui_draw_vision_speedlimit(s);
  ui_draw_vision_speed(s);
  ui_draw_vision_event(s);
  ui_draw_vision_bottom_bar(s);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  // Draw augmented elements
  if (scene->world_objects_visible) {
    ui_draw_world(s);
  }
  // Set Speed, Current Speed, Status/Events
  ui_draw_vision_header(s);
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE) {
    ui_draw_vision_face(s);
  }
}

void ui_draw(UIState *s, int w, int h) {
  // Update intrinsics matrix after possible wide camera toggle change
  if (s->fb_w != w || s->fb_h != h) {
    ui_resize(s, w, h);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  ui_draw_vision(s);
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha) {
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
  nvgRect(s->vg, r.x, r.y, r.w, r.h);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  nvgStrokeColor(vg, color);
  nvgStrokeWidth(vg, width);
  nvgStroke(vg);
}

static inline void fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor *color, const NVGpaint *paint, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  if (color) nvgFillColor(vg, *color);
  if (paint) nvgFillPaint(vg, *paint);
  nvgFill(vg);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius) {
  fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius) {
  fill_rect(vg, r, nullptr, &paint, radius);
}

void ui_nvg_init(UIState *s) {
  // on EON, we enable MSAA
  s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }

  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    {"wheel", "../assets/img_chffr_wheel.png"},
    {"driver_face", "../assets/img_driver_face.png"},
    {"hands_on_wheel", "../assets/img_hands_on_wheel.png"},
    {"map_source_icon", "../assets/img_world_icon.png"},
  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }
}

void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;

  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  nvgTranslate(s->vg, width / 2, height / 2 + y_offset);
  // 2) Apply same scaling as video
  nvgScale(s->vg, zoom, zoom);
  // 3) Put (0, 0) in top left corner of video
  nvgTranslate(s->vg, -intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);

  nvgCurrentTransform(s->vg, s->car_space_transform);
  nvgResetTransform(s->vg);
}
