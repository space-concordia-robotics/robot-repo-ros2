#!/bin/bash
# to use the args add the flags before them in any order 
# use ~ at least for after -d example : ~/newDirectory, /newDirectory DOES NOT WORK
#The quality might need to be adjusted later
#Example to run ./cam_recorder.sh -d ~/Test -o test.mp4
# Default values
DEFAULT_DEVICE="/dev/video0"
DEFAULT_DIR="$HOME/Videos"
DEFAULT_FILENAME="output_$(date+'%Y-%m-%d_%H-%M-%S').avi"

# Parse command-line arguments
while getopts "d:v:o:" opt; do
  case $opt in
    d) DIR="$OPTARG" ;;  # Directory
    v) DEVICE="$OPTARG" ;;  # Device name
    o) FILENAME="$OPTARG" ;;  # Output filename
    *) echo "Usage: $0 [-d directory] [-v device] [-o filename]"; exit 1 ;;
  esac
done

# Set to default if not provided
DIR="${DIR:-$DEFAULT_DIR}"
DEVICE="${DEVICE:-$DEFAULT_DEVICE}"
FILENAME="${FILENAME:-$DEFAULT_FILENAME}"

# Ensure the output directory exists
if [ ! -d "$DIR" ]; then
  echo "Directory $DIR does not exist. Creating..."
  mkdir -p "$DIR"
fi

# Full path for the output file
OUTPUT_FILE="$DIR/$FILENAME"

# Start recording using ffmpeg in the background
echo "Recording from device $DEVICE, output file: $OUTPUT_FILE"
ffmpeg -f v4l2 -i "$DEVICE" -c:v mpeg4 -q:v 5 -y "$OUTPUT_FILE"

# Capture the process ID of ffmpeg
FFMPEG_PID=$!

# Set up a trap to handle clean shutdown when the user presses Ctrl+C
trap "echo 'Stopping recording...'; kill -SIGINT $FFMPEG_PID; wait $FFMPEG_PID; echo 'Recording saved to $OUTPUT_FILE'; exit 0" SIGINT

# Wait for the ffmpeg process to finish (this will happen when the user presses Ctrl+C)
wait $FFMPEG_PID

# After ffmpeg finishes, check the result
if [ $? -eq 0 ]; then
  echo "Recording saved to $OUTPUT_FILE"
else
  echo "Error during recording. Please check the device and try again."
  exit 1
fi
