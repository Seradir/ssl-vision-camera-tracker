sudo modprobe v4l2loopback video_nr=20 card_label="ball_zoom" exclusive_caps=1

python3 ball_zoom_stream.py \
  --pb-port 10010 \
  --pb-multicast-group 224.5.23.2 \
  --decklink-device 0 \
  --decklink-connection sdi \
  --decklink-mode 1080p25 \
  --output-device /dev/video20 \
  --zoom-factor 2.0 \
  --map 140 120 -4.5 -3.0 \
  --map 1780 120  4.5 -3.0 \
  --map 140 960 -4.5  3.0 \
  --map 1780 960  4.5  3.0
