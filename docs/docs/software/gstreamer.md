# GStreamer

If you're debugging gstreamer code and stuck, read: https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html?gi-language=c

With gstreamer, the command line is your friend.

You can use `gst-inspect-1.0` to inspect all the properties for a gstreamer element, for example

```bash
gst-inspect-1.0 rtspsrc
```

To launch test pipelines, use `gst-launch-1.0`.

If you have an issue with a gstreamer pipeline, try setting the `GST_DEBUG` environment variable to something like `2`.
