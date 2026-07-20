import paramiko
import os

ip = '10.145.82.141'
username = 'veena_pi'
password = 'veena3699'
remote_dir = '/home/veena_pi/robotex_ai'
local_dir = r'C:\Users\Veena\robotex_ai'

files_to_push = [
    'best.pt',
    'master_brain.py',
    'robot_hardware.py',
    'pid_kinematics.py',
    'slam_mapping.py',
    'yolo_vision.py',
    'install_service.sh',
    # 'rl_brain_master.zip',  # Already on Pi
    'robot_env.py',
    "vision_test.py",
    "record_dataset.py",
    "test_tof_matrix.py",
    "test_gyro_drift.py",
    "test_pid_waypoint.py",
    "test_yolo_diag.py",
    "test_camera_diag.py",
    "test_vision_deep.py",
    'test_vision_suite.py',
    'test_hardware_suite.py',
    'test_final_hardware.py',
    'test_system_integrity.py',
    'test_laser_hardware.py',
    'test_flame_diagnostics.py',
    'sensor_test.py',
    'stream_test.py',
    'test_avoidance.py',
    'vision_test.py',         
    'record_dataset.py',      
    'test_tof_matrix.py',
    'test_vision_live.py'
]

print("Connecting to Robot...")
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

try:
    client.connect(ip, username=username, password=password, timeout=10)
    print("Connected! Pushing updated Master Architecture files to the robot...")
    
    # Create the directory just in case it doesn't exist
    client.exec_command(f'mkdir -p {remote_dir}')
    
    sftp = client.open_sftp()
    
    for file in files_to_push:
        local_path = os.path.join(local_dir, file)
        remote_path = remote_dir + "/" + file
        
        if os.path.exists(local_path):
            print(f"Uploading {file}...")
            sftp.put(local_path, remote_path)
        else:
            print(f"Skipping {file} (Not found locally)")
            
    sftp.close()
    
    # BUG FIX #7: Extract RL Brain zip so PPO.load() can find the directory
    print("Extracting RL Brain on the robot...")
    stdin, stdout, stderr = client.exec_command(f'cd {remote_dir} && unzip -o rl_brain_master.zip 2>/dev/null; echo done')
    stdout.read()  # Wait for completion
    
    print("SUCCESS! All files have been beamed to the robot.")
    
except Exception as e:
    print(f"FAILED TO CONNECT: {e}")
    print("Make sure the Raspberry Pi is turned on and connected to the hotspot!")
finally:
    client.close()
