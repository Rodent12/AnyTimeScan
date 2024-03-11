import cv2
import os

def extract_frames(input_video_path, output_folder, num_frames=72):
    # Open the video file
    cap = cv2.VideoCapture(input_video_path)

    # Get the frames per second (fps) of the video
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Get the total number of frames in the video
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(total_frames)
    # Calculate the interval between frames
    frame_interval = total_frames // num_frames

    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Loop through frames and extract at regular intervals
    for i in range(0, total_frames, frame_interval):
        # Set the frame position
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)

        # Read the frame
        ret, frame = cap.read()

        # If the frame is read successfully, save it
        if ret:
            output_path = os.path.join(output_folder, f'frame_{i}.jpg')
            cv2.imwrite(output_path, frame)

    # Release the video capture object
    cap.release()

if __name__ == "__main__":
    # Replace 'input_video.mp4' and 'output_frames' with your file paths
    input_video_path = 'video.mp4'
    output_folder = 'output_frames'

    extract_frames(input_video_path, output_folder, num_frames=72)
