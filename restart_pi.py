import paramiko
import sys

ip = '10.145.82.141'
username = 'veena_pi'
password = 'veena3699'

print(f"Connecting to Raspberry Pi at {ip} to restart the service...")
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

try:
    client.connect(ip, username=username, password=password, timeout=10)
    print("Connected successfully!")
    
    print("Restarting robotex.service...")
    # Sudo requires a PTY or password, but usually sudo systemctl doesn't prompt if configured, or we can pass password
    stdin, stdout, stderr = client.exec_command(f'echo {password} | sudo -S systemctl restart robotex.service')
    error = stderr.read().decode().strip()
    out = stdout.read().decode().strip()
    
    if "incorrect password" in error.lower() or "not allowed" in error.lower():
        print(f"Sudo error: {error}")
    else:
        print("Service restarted successfully!")
        if out: print(out)
        if error: print(f"Note: {error}")
        
except Exception as e:
    print(f"FAILED TO CONNECT: {e}")
    print("Make sure the Raspberry Pi is turned on and connected to the hotspot!")
finally:
    client.close()
