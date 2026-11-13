# GStreamer

If you're debugging gstreamer code and stuck,
read [Basic tutorial 11: Debugging tools](https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html).

!!! question inline end "Getting Help"

    GStreamer is quite complex, so often times searching something up is the best and quickest way to solve a problem.
    You will spend a lot of time searching things up and referencing documentation with GStreamer.
    If you do not know how to do something, try to search it, there are many examples online.\
    If that fails, looking at the [GStreamer source code](https://gitlab.freedesktop.org/gstreamer/gstreamer) for any examples can sometimes help
    (look in the `gst-examples` subproject and the `tests` directory in any relevant subproject).\
    If you still cannot figure it out, I recommend that you join the [GStreamer matrix space](https://matrix.to/#/#gstreamer:gstreamer.org).
    This is where you can ask for help related to GStreamer.

With gstreamer, the command line is your friend. The following are probably quite useful to you:

- `GST_DEBUG` environment variable (see above)
- `gst-inspect-1.0`
- `gst-launch-1.0`

The following tutorials will also likely be of significant help to you:

- [Basic Tutorial 2: GStreamer concepts](https://gstreamer.freedesktop.org/documentation/tutorials/basic/concepts.html)
- [Basic Tutorial 3: Dynamic pipelines](https://gstreamer.freedesktop.org/documentation/tutorials/basic/dynamic-pipelines.html)
- [Basic Tutorial 7: Multithreading and Pad Availability](https://gstreamer.freedesktop.org/documentation/tutorials/basic/multithreading-and-pad-availability.html)
- [Basic tutorial 8: Short-cutting the pipeline](https://gstreamer.freedesktop.org/documentation/tutorials/basic/short-cutting-the-pipeline.html)
- [Basic Tutorial 10: GStreamer tools](https://gstreamer.freedesktop.org/documentation/tutorials/basic/gstreamer-tools.html)
- [Basic Tutorial 16: Handy Elements](https://gstreamer.freedesktop.org/documentation/tutorials/basic/handy-elements.html)
- [Playback tutorial 8: Hardware-accelerated video decoding](https://gstreamer.freedesktop.org/documentation/tutorials/playback/hardware-accelerated-video-decoding.html)

On the NVIDIA Jetson, we can also do hardware-accelerated encoding. Check
out [Accelerated GStreamer - NVIDIA Jetson Linux Developer Guide](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)
for more information.
The main elements that concern us are:

- `nvv4l2h264enc` - h264 encoder
- `nvv4l2h265enc` - h265 encoder
- `nvv4l2av1enc` - av1 encoder
