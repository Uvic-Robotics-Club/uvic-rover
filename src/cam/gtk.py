import gi
gi.require_version('Gst', '1.0')
gi.require_version('Gtk', '3.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, Gtk, GLib, GstVideo

Gst.init(None)

class GTK_Main:
    def __init__(self):
        window = Gtk.Window(Gtk.WindowType.TOPLEVEL)
        window.set_title("GStreamer Player")
        window.set_default_size(300, -1)
        window.connect("destroy", Gtk.main_quit)

        self.player = Gst.ElementFactory.make("playbin", "player")
        self.player.set_property("uri", "udp://127.0.0.1:5000")

        video_sink = Gst.ElementFactory.make("gtksink", "video_sink")
        self.player.set_property("video-sink", video_sink)

        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        window.add(vbox)
        vbox.pack_start(video_sink.props.widget, True, True, 0)

        self.player.set_state(Gst.State.PLAYING)
        window.show_all()

GTK_Main()
Gtk.main()