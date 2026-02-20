import sys
import time

# Add the SDK folder to Python path
# sys.path.append(r"C:\Program Files\Vicon\DataStream SDK\Win64\Python")

from vicon_dssdk.vicon_dssdk import ViconDataStream

# -----------------------
# Configuration
# -----------------------
SERVER_IP = "192.168.10.1:801"   # Your Vicon server IP
SUBJECT_NAME = "OriginsX"       # Change to your tracked subject name
# -----------------------

# Connect
client = ViconDataStream.Client()
client.Connect(SERVER_IP)
client.EnableSegmentData()
client.EnableMarkerData()

print(f"Connected to {SERVER_IP}, streaming pose for subject: {SUBJECT_NAME}")

# Main loop
while True:
    if client.GetFrame():
        # Make sure the subject exists in this frame
        subjects = client.GetSubjectNames()
        if SUBJECT_NAME in subjects:
            # Get global translation (x, y, z in mm)
            translation = client.GetSegmentGlobalTranslation(SUBJECT_NAME, SUBJECT_NAME)
            pos, _ = translation  # unpack the tuple
            x, y, z = pos

            # Get global rotation quaternion (x, y, z, w)
            rotation = client.GetSegmentGlobalRotationQuaternion(SUBJECT_NAME, SUBJECT_NAME)
            rot, _ = rotation
            qx, qy, qz, qw = rot


            # Print nicely
            print(f"Translation: x={x:.2f} y={y:.2f} z={z:.2f}")
            print(f"Rotation Quaternion: x={qx:.4f} y={qy:.4f} z={qz:.4f} w={qw:.4f}")
        else:
            print(f"Subject '{SUBJECT_NAME}' not found in this frame")
    else:
        print("Waiting for frame...")
        time.sleep(0.01)
