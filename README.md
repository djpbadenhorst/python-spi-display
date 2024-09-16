Install image:
radxa-zero_ubuntu_bullseye_cli_b23.img.xz
before.conf -> change user

rsetup
activate connection, change hostname, change password
ip a
rsetup
###########################
rsetup - update
sudo apt install cinnamon -y
#########
rsetup - spi display overlay
###########################
sudo apt install python3-dev
sudo pip install spidev
sudo pip install gpiod
sudo pip install ipdb
sudo pip install numpy

###########################
import spidev
import gpiod
spi = spidev.SpiDev()
spi.open(1, 0)
        self.GPIO.setup(self.RST_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.DC_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.BL_PIN, self.GPIO.OUT)
        self._pwm=self.GPIO.PWM(self.BL_PIN,self.BL_freq)
        self._pwm.start(100)
        if self.SPI!=None :
            self.SPI.max_speed_hz = self.SPEED        
            self.SPI.mode = 0b00     
        return 0

def set_pin_val(chip_name, pin_offset, pin_val):
    try:
        with gpiod.Chip(chip_name) as chip:
            line = chip.get_line(pin_offset)
            line.request(consumer="my_gpio_script", type=gpiod.LINE_REQ_DIR_OUT)
            line.set_value(pin_val)  # Set the pin to low
            line.release()
    except Exception as e:
        print("Error:", e)

set_pin_val("gpiochip3", 12, 1)

sudo gpioget $(sudo gpiofind PIN_12)
sudo gpioset -m signal $(sudo gpiofind PIN_12)=1
sudo gpioset -m signal $(sudo gpiofind PIN_12)=0

# Call the function to set the pin to low
set_pin_val(chip_name, pin_offset, pin_val)
#########333


with gpiod.Chip("/dev/gpiochip1") as chip:
    info = chip.get_info()
    print(f"{info.name} [{info.label}] ({info.num_lines} lines)")






radxa-zero_ubuntu_bullseye_cli_b23.img.xz
rsetup - change password and hostname and upgrade
ffmpeg -video_size 800x480 -framerate 10 -f x11grab -i :0 output.mp4

ffmpeg -f x11grab -i :0 -re -listen 1 -vcodec copy -f flv http://localhost:8080

ffmpeg -re  -f x11grab -i :0 -vcodec libx264 -f mpegts udp://127.0.0.1:8000?pkt_size=1316
ffmpeg -re  -f x11grab -i :0 -vcodec libx264 -f mpegts tcp://djpb-yoga.local:9999?pkt_size=1316
vlc udp://@127.0.0.1:1234?pkt_size=1316


ffmpeg -f x11grab -i :0 -rtsp_transport tcp -c:v libx264 -preset ultrafast -tune zerolatency -b:v 500k -c:a aac -strict experimental -f rtsp rtsp://127.0.0.1:2000


ffmpeg -re   -i :0 -f mpegts tcp://127.0.0.1:2000\live?listen


vlc tcp://ar.local:2000\?listen


ffmpeg -rtsp_transporttcp -i  "rtsp:20000" -f image2pipe "udp://localhost:3333"
ffmpeg -i Stingray.264 -f mp4 -movflags isml+frag_keyframe -vcodec copy tcp://10.99.19.224:8888






radxa-zero_ubuntu_jammy_cli_2023-03-01T0518+0000_msdos.img.xz
radxa-zero_ubuntu_jammy-test_kde_t4.img.xz
radxa-zero_ubuntu_jammy-test_cli_t4.img.xz

sudo apt install -y ubuntu-desktop lightdm 
sudo apt install xubuntu-core
sudo apt -y install cinnamon-desktop
sudo apt install lightdm 
sudo apt -y install task-cinnamon-desktop
sudo add-apt-repository universe 
sudo apt install cinnamon-desktop-environment 




sudo apt install slim
sudo apt install lxdm


sudo systemctl start lightdm.service
sudo service ligthdm start




Change config.before
User=djpb
WIFI settings
Enable SSH

ssh radxa-zero.local

####
scp output.mp4 192.168.68.113:~/Workspace

/etc/sddm.conf.d/autologin.conf
[Autologin]
User=djpb
Session=X11


