import os
import sys
import subprocess

def generate_video(rgb_dir, output_video="output.mp4", fps=20):
    if not os.path.isdir(rgb_dir):
        print(f"Error: Directory '{rgb_dir}' does not exist.")
        sys.exit(1)

    cmd = [
        "ffmpeg",
        "-y",  # Overwrite output if exists
        "-framerate", str(fps),
        "-i", os.path.join(rgb_dir, "%04d.jpg"),
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        output_video
    ]

    subprocess.run(cmd, check=True)
    print(f"Video generated: {output_video}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python generate_video.py <rgb_image_folder> [output.mp4]")
        sys.exit(1)

    rgb_path = sys.argv[1]
    output_name = sys.argv[2] if len(sys.argv) > 2 else "output.mp4"
    generate_video(rgb_path, output_video=output_name, fps=5)
