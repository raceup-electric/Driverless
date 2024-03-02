# CONNECT WINDOWS LAPTOP TO ENBEDDED SYSTEM


## EMBEDDED SYSTEM CONFIGURATION
1. First step is to clone the create_ap repostory and install it
```
git clone https://github.com/oblique/create_ap
cd create_ap
sudo make install
```
3. now you have to change the settings
```
sudo gedit /etc/create_ap.config
```
in particular make sure you change this line:
```
SHARE_METHOD=none
INTERNET_IFACE=
SSID=BFFT_VEHICLE_AP
PASSPHRASE=XXXXXXXX
```
5. for enabling the AP every time you start the system, run this command
```
systemctl enable create_ap
```
7. For connect with ssh to the system you have to install  opnessh
```
sudo apt install openssh-server
```

***

## WINDOWS LAPTOP CONFIGURATION
1. To install Ubuntu, or rather to run bash commands on Windows, the Windows Subsystem for Linux (WSL) must first be enabled. To do this, open PowerShell as admin and run following commands:
```
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```
3. Now we can adjust the permissions for the private SSH key. However, since WSL cannot store metadata for files in windows without the appropriate customization, we must first create a WSL.conf file and configure it accordingly to make this possible
```
wsl vi /mnt/c/windows/system32/drivers/etc/wsl.conf
```
write the following lines and then save it:
```
[automount]
options = "metadata"
```

5. if you want now you can chnage the permission of the file
```
wsl chmod 400 /mnt/c/users/administrator/.ssh/id_rsa.txt
```

7. Now you can finally connect to the embedded system with the following command:
```
ssh -i C:\Users\Administrator\.ssh\id_rsa.txt bfft-agx-1@10.0.0.1
```

***

## TRUBLESHOOTING

it's possible that the AP don't work proprely. If this append you can resolve it using a specific command for start the ap with create_ap
```
create_ap -n wlan0 MyAccessPoint MyPassPhrase
```
or
```
create_ap wlan0 eth0 MyAccessPoint MyPassPhrase
```
obiuvsly you have to change the wifi interface, name and password of the AP
