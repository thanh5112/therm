192.168# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2019. All rights reserved.
#
import sys
sys.path.append("/home/test/myenv/lib/python3.11/site-packages")
import os
import signal
from smbus import SMBus
from spidev import SpiDev
import argparse

try:
    from gpiozero import Pin, DigitalInputDevice, DigitalOutputDevice
except:
    print("Please install the 'gpiozero' library to monitor "
          "the MI48 DATA_READY pin. For example, by:")
    print("pip3 install gpiozero")
    sys.exit()

import time
import logging
import numpy as np
import cv2 as cv
import socket
import base64
import subprocess

from senxor.mi48 import MI48, DATA_READY, format_header, format_framestats
from senxor.utils import data_to_frame, cv_filter
from senxor.interfaces import SPI_Interface, I2C_Interface

# This will enable mi48 logging debug messages
logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))

def get_filename(tag, ext=None):
    ts = time.strftime('%Y%m%d-%H%M%S', time.localtime())
    filename = "{}--{}".format(tag, ts)
    if ext is not None:
        filename += '.{}'.format(ext)
    return filename

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--record', default=False, dest='record',
                        action='store_true', help='Record data')
    args = parser.parse_args()
    return args

def write_frame(outfile, arr):
    if arr.dtype == np.uint16:
        outstr = ('{:n} '*arr.size).format(*arr.ravel(order='C')) + '\n'
    else:
        outstr = ('{:.2f} '*arr.size).format(*arr.ravel(order='C')) + '\n'
    try:
        # assume outfile is a handle to a file open for write
        outfile.write(outstr)
        # we have a relatively large outstr (~5K * 7ASCII chars per pixel)
        # since the OS has by default ~8KB buffer, it will be good to
        # flush so as to not output incomplete frame to the file
        # (which may happen if we early terminate for some reason)
        # or else it may lead to partially output frame and problems
        # upon read.
        outfile.flush()
        return None
    except AttributeError:
        # assume outfile is a name to a file
        # this should automatically flush the buffer
        with open(outfile, 'a') as fh:
            fh.write(outstr)
        return None
    except IOError:
        logger.critical('Cannot write to {} (IOError)'.format(outfile))
        sys.exit(106)

def cv_display(img, resize,
               colormap,title='', interpolation=cv.INTER_CUBIC):
    """
    Display image using OpenCV-controled window.

    Data is a 2D numpy array of type uint8,

    Image is coloured and resized
    """
    cvcol = cv.applyColorMap(img, colormap)
    cvresize = cv.resize(cvcol, resize, interpolation=interpolation)
    cv.imshow(title, cvresize)

# Main starts here; ideally we shall isolate that in a function
# -------------------------------------------------------------
def thermal(fps, colormap, min_temp, max_temp, resize):

    args = parse_args()

    host = '192.168.4.111'
    port_1 = 9999
    BUFF_SIZE = 65536
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)

    server_socket.bind((host, port_1))
    print('Server listening at:', (host, port_1))

    # ls /dev/*i2c* should indicate which i2c channels are available to user space
    RPI_GPIO_I2C_CHANNEL = 1

    # ls /dev/*spi*
    # should indicate which spi bus and what devices are available:
    # e.g. /dev/spidev<bus>.<device>
    # /dev/spidev0.0  /dev/spidev0.1
    RPI_GPIO_SPI_BUS = 0

    # MI48A CS is routed to CE1 of the RPI on the uHAT development board
    # adapt that value according to your setup
    RPI_GPIO_SPI_CE_MI48 = 0

    # =======================
    # MI48 I2C Address:
    # =======================
    # could be 0x40 or 0x41, depending on how ADDR pin of the chip is tied.
    # use
    # $i2cdetect -y 1
    # on the command prompt to confirm which address the device responds to
    MI48_I2C_ADDRESS = 0x40

    # =======================
    # MI48 SPI Stuff:
    # =======================
    MI48_SPI_MODE = 0b00
    MI48_SPI_BITS_PER_WORD = 8 # cannot be handled with /dev/spidev-x.y and python on RPi 3B+; must work with default 8
    MI48_SPI_LSBFIRST = False # this appears to be a read-only on RPi
    MI48_SPI_CSHIGH = True

    # MI48_SPI_MAX_SPEED_HZ = 7800000
    # MI48_SPI_MAX_SPEED_HZ = 15600000
    MI48_SPI_MAX_SPEED_HZ = 31200000
    MI48_SPI_CS_DELAY = 0.0001 # delay between asserting/deasserting CS_N and initiating/stopping clock/data

    def close_all_interfaces():
        try:
            spi.close()
        except NameError:
            pass
        try:
            i2c.close()
        except NameError:
            pass

    # define a signal handler to ensure clean closure upon CTRL+C
    # or kill from terminal
    def signal_handler(sig, frame):
        logger.info("Exiting due to SIGINT or SIGTERM")
        mi48.stop(poll_timeout=0.25, stop_timeout=1.2)
        time.sleep(0.5)
        cv.destroyAllWindows()
        logger.info("Done.")
        sys.exit(0)

    # Define the signals that should be handled to ensure clean exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # create an I2C interface object
    i2c = I2C_Interface(SMBus(RPI_GPIO_I2C_CHANNEL), MI48_I2C_ADDRESS)

    # ==============================
    # Create an SPI interface object
    # ==============================
    #
    # One needs to chose a buffer size for transfer; Optimal size may be
    # different depending on target FPS and host's resources
    #
    # cat /sys/module/spidev/parameters/bufsiz to check default size
    # Default size can potentially be changed via /boot/cmdline.txt on RPI
    # spidev.bufsize=<NEEDED BUFFER SIZE>
    # Preferred way may be with the initialisation of the spi object.
    # We chose 160 bytes which corresponds to 1 row on MI08xx
    SPI_XFER_SIZE_BYTES = 160
    spi = SPI_Interface(SpiDev(RPI_GPIO_SPI_BUS, RPI_GPIO_SPI_CE_MI48),
                    xfer_size=SPI_XFER_SIZE_BYTES)

    spi.device.mode = MI48_SPI_MODE
    spi.device.max_speed_hz = MI48_SPI_MAX_SPEED_HZ
    spi.device.bits_per_word = 8
    spi.device.lsbfirst = False # seems to be a read-only value;
                            # likely reflecting cpu endianness

    #spi.device.cshigh = MI48_SPI_CSHIGH
    # in linux kernel 5.x.x+ ioctl module does not handle the SPI CS
    # any more, since it is thought that it is a devcie property,
    # not a bus property. We therefore leave the CS to a GPIO handling.
    # Note that on the uHat board that we have with MI48 and Bobcat,
    # the CS is on GPIO-7 (J8 connector Pin 26).                       
    spi.cshigh = True
    spi.no_cs = True
    mi48_spi_cs_n = DigitalOutputDevice("BCM7", active_high=False,
                                    initial_value=False)

    # ===============================
    # Configure DATA_READY and RESET
    # ===============================

    # Assuming that DATA_READY is connected
    # NOTABENE:
    # The MI48.DATA_READY pin is routed to BROADCOM.GPIO.24,
    # which is pin 18 on the 40-pin header.
    # The gpiozero library uses the BROADCOM convention, hence we have
    # "BCM24" below, or just 24.
    #
    # Change this to False to test DATA_READY flag, instead of pin
    use_data_ready_pin = True
    if use_data_ready_pin:
        mi48_data_ready = DigitalInputDevice("BCM24", pull_up=False)

    # connect the reset line to allow to drive it by SW (GPIO23, J8:16)
    mi48_reset_n = DigitalOutputDevice("BCM23", active_high=False,
                                   initial_value=True)

    class MI48_reset:
        def __init__(self, pin,
                 assert_seconds=0.000035,
                 deassert_seconds=0.050):
            self.pin = pin
            self.assert_time = assert_seconds
            self.deassert_time = deassert_seconds

        def __call__(self):
            print('Resetting the MI48...')
            self.pin.on()
            time.sleep(self.assert_time)
            self.pin.off()
            time.sleep(self.deassert_time)
            print('Done.')

    # ==============================
    # Create an MI48 interface object
    # ==============================
    mi48 = MI48([i2c, spi], data_ready=mi48_data_ready,
            reset_handler=MI48_reset(pin=mi48_reset_n))

    # print out camera info
    camera_info = mi48.get_camera_info()
    logger.info('Camera info:')
    logger.info(camera_info)

    # set desired FPS
    # TODO: investigate issue at > 9 FPS on R-Pi 3B+
    mi48.set_fps(fps)

    # see if filtering is available in MI48 and set it up
    if int(mi48.fw_version[0]) >= 2:
        mi48.enable_filter(f1=True, f2=True, f3=False)

        # If needed, set a temperature offset across entire frame
        # e.g. if overall accuracy (at product level) seems to be 
        # 0.7 above the blackbody, then we need to subtract 0.7 
        # from the readout of the MI48:
        # mi48.set_offset_corr(-5.55)
        #
        # However, for most applications the factory level, per pixel
        # calibration is sufficient, so keep offset 0
        mi48.set_offset_corr(0.0)

    # initiate continuous frame acquisition
    with_header = True

    # enable saving to a file
    if args.record:
        filename = get_filename(mi48.camera_id_hex)
        fd_data = open(os.path.join('.', filename+'.dat'), 'w')

    mi48.start(stream=True, with_header=with_header)

    # change this to false if not interested in the image
    GUI = False

    try:
        print("Waiting for a client...")
        while True:
            msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
            print('Connection from:', client_addr)

            while True:
                # wait for data_ready pin (or poll for STATUS.DATA_READY /fw 2.1.X+)
                if hasattr(mi48, 'data_ready'):
                    mi48.data_ready.wait_for_active()
                else:
                    data_ready = False
                    while not data_ready:
                        time.sleep(0.01)
                        data_ready = mi48.get_status() & DATA_READY
 
                # read the frame
                # assert the spi_cs, delay a bit then read
                mi48_spi_cs_n.on()
                time.sleep(MI48_SPI_CS_DELAY)
                data, header = mi48.read()

                if data is None:
                    logger.critical('NONE data received instead of GFRA')
                    mi48.stop(stop_timeout=1.0)
                    sys.exit(1)

                # delay a bit, then deassert spi_cs
                time.sleep(MI48_SPI_CS_DELAY)
                mi48_spi_cs_n.off()

                if args.record:
                    write_frame(fd_data, data)

                img = data_to_frame(data, mi48.fpa_shape)

                if header is not None:
                    logger.debug('  '.join([format_header(header),
                                format_framestats(data)]))
                else:
                    logger.debug(format_framestats(data))

                img8u = ((img - min_temp) / (max_temp - min_temp) * 255).clip(0, 255).astype('uint8')
                img8u = cv_filter(img8u, parameters={'blur_ks': 3}, use_median=False, use_bilat=True, use_nlm=False)
                img8u = cv.applyColorMap(img8u, colormap=getattr(cv, f'COLORMAP_{colormap.upper()}'))
                img8u = cv.resize(img8u, resize, interpolation=cv.INTER_CUBIC)
                _, buffer = cv.imencode('.jpg', img8u, [cv.IMWRITE_JPEG_QUALITY, 50])
                message = base64.b64encode(buffer)

                if len(message) > BUFF_SIZE:
                    print("Frame size too large, skipping transmission.")
                    continue

                server_socket.sendto(message, client_addr)

                if GUI:
                    cv_display(img8u, resize, colormap=getattr(cv, f'COLORMAP_{colormap.upper()}'))
                    key = cv.waitKey(1) # & 0xFF
                    if key == ord("q"):
                        break
    #    time.sleep(1)
    except KeyboardInterrupt:
        print("\nServer stopped by user.")

    finally:
        # stop capture and quit
        server_socket.close()
        mi48.stop(stop_timeout=0.5)
        try:
            fd_data.close()
        except NameError:
        # file descriptor was closed or never created
            pass
        cv.destroyAllWindows()

def start_udp_server(host, port_cli):

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((host, port_cli))
            print(f"Server listening on {host}:{port_cli}")

            while True:
                # Receive data from the client
                data, client_address = server_socket.recvfrom(1024)  # Buffer size is 1024 bytes
                message_data = data.decode().strip()
                print(f"Received from {client_address}: {message_data}")

                try:
                    # Split the message into Thermal_parameter and message
                    Thermal_parameter, message = message_data.split(";", 1)  # Split only on the first comma
                except ValueError:
                    # If there's no comma, treat the whole data as just the message
                    message = message_data
                    Thermal_parameter = None

                print(f"Thermal_parameter: {Thermal_parameter}, message: {message}")

                # Handle different messages
                if message == "Connecting":
                    response = "Connected, ready to work"
                elif message == "Initialize" and Thermal_parameter != " ":
                    # Ensure that Thermal_parameter is a valid command or path
                    dir = "/home/test/pysenxor-master/example"
                    print(Thermal_parameter)
                    
                    # Split the Thermal_parameter and convert to required types
                    try:
                        # Assuming the format: "fps/colormap/resize_x,resize_y/min_temp/max_temp"
                        params = Thermal_parameter.split("/")

                        # Extract the parameters from the split values
                        fps = float(params[0])  # FPS as a float
                        colormap = params[1]  # Colormap as string
                        resize = tuple(map(int, params[2].split(',')))  # Resize as a tuple of integers
                        min_temp = float(params[3])  # Minimum temperature as a float
                        max_temp = float(params[4])  # Maximum temperature as a float
                        response = "params_sent"
                        # Start the subprocess for Thermal_parameter
                        print(f"Starting thermal camera with fps={fps}, colormap={colormap}, resize={resize}, min_temp={min_temp}, max_temp={max_temp}")
                        thermal(fps=fps, colormap=colormap, resize=resize, min_temp=min_temp, max_temp=max_temp)
                        print(f"Thermal parameters received and camera started.")

                    except Exception as e:
                        response = f"Error starting subprocess: {str(e)}"
                        print(f"Error starting subprocess: {str(e)}")

                elif message == "bye":
                    response = "Goodbye!"

                elif message == "Initialized" and Thermal_parameter == 1:
                    response = "Camera connected!"


                elif message == "disconnect":  # Check if message is "disconnect"
                    response = "Disconnecting server..."
                    print("Disconnect message received. Shutting down the server.")
                    server_socket.sendto(response.encode(), client_address)
                    break  # Exit the while loop to stop the server
                else:
                    response = "Unknown command."

                # Send the response back to the client
                print(f"Sending response to {client_address}: {response}")
                try:
                    server_socket.sendto(response.encode(), client_address)
                except Exception as e:
                    print(f"Error sending response to {client_address}: {e}")

    except Exception as e:
        print(f"Server error: {e}")
    finally:
        print("Server shutting down.")



start_udp_server(host='192.168.4.111', port_cli=1111)