!/bin/bash

ffmpeg -framerate 8 -i fig%02d.png -r 30 -pix_fmt yuv420p video.mp4
