#!/bin/bash
echo "Creating systemd service for Robotex AI..."

cat << 'EOF' | sudo tee /etc/systemd/system/robotex.service
[Unit]
Description=Robotex AI Master Brain
After=network.target

[Service]
ExecStart=/bin/bash -lc 'python3 /home/veena_pi/robotex_ai/master_brain.py'
WorkingDirectory=/home/veena_pi/robotex_ai
StandardOutput=inherit
StandardError=inherit
Restart=always
User=veena_pi

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling and starting the service..."
sudo systemctl daemon-reload
sudo systemctl enable robotex.service
sudo systemctl start robotex.service

echo ""
echo "=========================================================="
echo "SUCCESS! The Robotex code will now run automatically"
echo "every time you turn on the Raspberry Pi!"
echo "=========================================================="
echo ""
echo "Useful Commands:"
echo "To view live logs: sudo journalctl -u robotex.service -f"
echo "To stop it: sudo systemctl stop robotex.service"
echo "To start it: sudo systemctl start robotex.service"
