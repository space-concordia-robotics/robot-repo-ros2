# Video Stream Widget

The video stream widget is used to display a camera from the rover.

## Video Stream

The video stream widget is based on [GStreamer](../misc/gstreamer.md) and uses RTSP streams.

Because GStreamer is [GLib](https://docs.gtk.org/glib/)-based[^glib-sucks], in order to make it easier to work
with, [peel](https://bugaevc.pages.gitlab.gnome.org/peel/) is used to provide C++ bindings for it. peel uses
GStreamer's [GIR](https://gi.readthedocs.io/en/latest/)[^gstreamer-gir] to do codegen to generate C++ bindings for things like smart pointers. The runtime cost
of peel should be zero, as the compiler should be able to optimize it all away.

[^glib-sucks]: GLib just kinda sucks to work with.

<!-- @formatter:off -->
[^gstreamer-gir]: GStreamer's GIR is kind of broken in older versions, such as the one available on Ubuntu 24.04.\
    Due to this, some methods are not available by default in peel, and some tweaks must be applied in order to fix this.

    See: the `API_TWEAKS_FILE` variable in the `CMakeLists.txt` file.
<!-- @formatter:on -->

When working with GStreamer, a search engine will be your best friend. If that fails, looking at the GStreamer source code for any examples can sometimes help.\
If you still cannot figure it out, I recommend that you join the GStreamer matrix space. This is where you can ask for help relates to GStreamer.

## Overlays

Currently, the video streamer has the following overlays implemented:

- Navigation path overlay:\
  Draws paths from the :topic:`/plan` and :topic:`/local_plan` topics.
- ArUco tag overlay:\
  Draws an outline and orientation arrows for any detected ArUco tags in the :topic:`/aruco_detections` topic.
- Video stats overlay[^video-stats]:\
  Overlays video stats such as bandwidth, fps, etc.
- Crosshair overlay:
  Draws a crosshair in the middle of the screen.
- Minimap overlay:\
  Overlays a minimap in the top-right corner of the screen, showing a compass using the :topic:`/imu/rpy` topic.\
  In the future, this overlay will also show things like the current path, positions of ArUco tags, and more.

[^video-stats]: This is currently broken, and thus disabled.

## Settings

The video widgets expose several different settings to the user:

- Source RTSP url: this is the url that GStreamer connects to in order to get the video data.
- Camera topic: this is used by the widget in order to determine the camera transform and the camera's intrinsics.
- RTSP latency & jitterbuffer latency: these two are used to configure the duration of data that gstreamer buffers in order to smooth out the data rate and
  avoid frame drops.\
  The two of these might actually be redundant, and you only need one. To be determined.
- Minimap: toggle the minimap on/off.

## Filters

The video widgets supports several different filters, which can be applied to the video stream:

- brightness
- contrast
- saturation
- gamma
- sharpness
- flip/rotation

These filters exist so that even in bad lighting conditions, we can adjust them to be able to continue operating.
