import subprocess

for i in range(2, 16):
    cmd_str = f'convert Marker{i}.png -interpolate Integer -filter point -resize "3000%" Marker{i}.png'
    subprocess.run(cmd_str, shell=True)
