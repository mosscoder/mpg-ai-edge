# SSH instructions

Below are the steps to connect to the Jetson, prepare the environment, find the robot, and run the script. Add or adjust details as needed.

One-time setup: prioritize the test network on the Jetson
- Open the Jetson network settings and ensure the test hotspot or WiFi is marked to auto-connect.
- If supported, set that network to a higher priority than other saved networks (or forget competing networks).
- Reboot or toggle WiFi to confirm it auto-connects to the test network.
- If you have access to a keyboard and monitor, it might be useful to write down the MAC address[es] of the Jetson before moving it into the "headless" (no monitor) setup if there are issues later on with finding the IP address. 

One-time setup: configure the robot to join the test network via STA
- Open the Unitree Go App, select the robot, then go to device settings > Robot Dog Settings.
- Under "Change another connection," choose "WiFi Mode," then enter the network name and password.
- Note: This was tested with a Google Pixel and an iPhone; only the Pixel worked.
- Note: The "register to cloud" step timed out, but the STA connection still worked.

1. Connect all devices to the network
   - Connect the Jetson, robot, and your laptop to the same hotspot or WiFi used for testing.
   - If the network is a phone hotspot, keep it awake and on the same SSID throughout the session.

2. Discover the IP address of the Jetson
   - For MPG Jetsons, you can use the [mpg.commsat.org/api/clients](https://mpg.commsat.org/api/clients?password=robotanist) endpoint on a browser or CURL request to get the local IP of the Jetson, as long as it's running the posting service. Look for the nickname of the Jetson you want to connect to, and run the command `ssh [username]@[ip]' to connect to the Jetson. 
   - Otherwise, you can also use a network scan, e.g. `nmap -sn 192.168.1.0/24`, then look for the Jetson hostname or MAC.
   - If you don't have nmap installed, you can also try `arp -a` to see if you can identify the IP address of the jetson this way. 

3. SSH into the Jetson
   - From your laptop, run: `ssh <user>@<jetson_ip>`.
   - If the key is not set up, follow the prompt to accept the host key and enter the password.
   - To SSH from VS Code, press the button with the two opposite arrows in the very bottom-left corner, select "Connect Current Window to Host", then choose the IP address if you've entered it before. If not, select "Add New SSH Host", then enter the full ssh command. 

4. Activate the `mpg-jetson` environment
   - Change into the repo or project directory on the Jetson.
   - Activate the environment, for example:
     - Conda: `conda activate mpg-jetson`
      (Nvidia Jetpack 5)
     - venv: `source .venv/bin/activate`
      (Nvidia Jetpack 6)
   - Verify by checking `python --version` or `which python`.

5. Discover the IP of the robot (scan for movement-script ports)
   - The movement scripts use the local WebRTC SDP endpoints on TCP **8081** (legacy `/offer`) or TCP **9991** (`/con_notify` + `/con_ing_*`).
   - From the Jetson (or any device on the same network), run a targeted scan for those ports:
     - `nmap -sT -p 8081,9991 --open 192.168.1.0/24`
   - The IP that shows **8081** or **9991** open is the Go2. If multiple IPs appear, disambiguate via the router DHCP list (hostname/MAC/vendor).
   - (For MPG - on the Pixel Hotspot our Go2 got the local IP 192.168.1.105)

6. Record and export the robot IP in the scripts
   - Set the environment variable expected by the scripts, for example:
     - `export ROBOT_IP=<robot_ip>`
   - It's a good idea to add it to your shell profile or a `.env` file so it persists.

7. Run the python script
   - From the appropriate project directory, run the script, e.g.:
     - `python path/to/script.py`
   - Confirm it connects to the robot and logs expected output.
   - Watch the output, the robot should start actions in < 10 seconds 
