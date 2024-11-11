from controller import Robot, Keyboard, Display, GPS
from vehicle import Driver
import math

TIME_STEP = 50

# Initialize the driver for BMW X5
try:
    driver = Driver()
except Exception as e:
    print("Error: Unable to initialize the driver for BMW X5:", e)
    exit(1)

# Flags to enable specific features
enable_collision_avoidance = False
enable_display = False
has_gps = False
has_camera = False

# Initialize camera
camera = None
camera_width = -1
camera_height = -1
camera_fov = -1.0

# SICK laser (LIDAR)
sick = None
sick_width = -1
sick_range = -1.0
sick_fov = -1.0

# Speedometer (display)
display = None
display_width = 0
display_height = 0
speedometer_image = None

# GPS
gps = None
gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0

# Miscellaneous variables
speed = 0.0
steering_angle = 0.0

# Initialize and check available devices
for i in range(driver.getNumberOfDevices()):
    device = driver.getDeviceByIndex(i)
    name = device.getName()
    
    # Check for specific devices by name
    if name == "Sick LMS 291":  # LIDAR for collision avoidance
        enable_collision_avoidance = True
        sick = driver.getDevice("Sick LMS 291")
        sick.enable(TIME_STEP)
        sick_width = sick.getHorizontalResolution()
        sick_range = sick.getMaxRange()
        sick_fov = sick.getFov()
        print("LIDAR enabled.")
    
    elif name == "display":  # Speedometer display
        enable_display = True
        display = driver.getDevice("display")
        speedometer_image = display.imageLoad("speedometer.png")
        print("Display enabled.")
    
    elif name == "gps":  # GPS device
        has_gps = True
        gps = driver.getDevice("gps")
        gps.enable(TIME_STEP)
        print("GPS enabled.")
    
    elif name == "camera":  # Camera device
        has_camera = True
        camera = driver.getDevice("camera")
        camera.enable(TIME_STEP)
        camera_width = camera.getWidth()
        camera_height = camera.getHeight()
        camera_fov = camera.getFov()
        print("Camera enabled.")

# Start the car's systems (lights, wipers, etc.)
driver.setHazardFlashers(True)
driver.setDippedBeams(True)
driver.setAntifogLights(True)
driver.setWiperMode(Driver.SLOW)

# Initialize keyboard for manual driving
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Print help message
def print_help():
    print("You can drive the BMW X5 using the arrow keys:")
    print("[UP] - Accelerate forward")
    print("[DOWN] - Reverse")
    print("[LEFT]/[RIGHT] - Steer")
    print("Release UP/DOWN to stop accelerating or reversing.")
    print("Release LEFT/RIGHT to stop steering.")

print_help()

# Function to set the speed
def set_speed(kmh):
    global speed
    max_speed = 150.0  # Limit max speed
    min_speed = -20.0  # Limit reverse speed
    speed = min(max_speed, max(min_speed, kmh))
    driver.setCruisingSpeed(speed)

# Function to set the steering angle
def set_steering_angle(angle):
    global steering_angle
    max_steering = 0.5  # Maximum steering angle in radians
    steering_angle = max(-max_steering, min(max_steering, angle))
    driver.setSteeringAngle(steering_angle)

# Function to process keyboard input
def check_keyboard():
    global speed, steering_angle

    key = keyboard.getKey()

    # Default to no movement
    move_forward = False
    move_backward = False
    turn_left = False
    turn_right = False

    while key != -1:
        if key == Keyboard.UP:  # Accelerate forward when UP arrow is pressed
            move_forward = True
        elif key == Keyboard.DOWN:  # Reverse when DOWN arrow is pressed
            move_backward = True
        elif key == Keyboard.LEFT:  # Steer left when LEFT arrow is pressed
            turn_left = True
        elif key == Keyboard.RIGHT:  # Steer right when RIGHT arrow is pressed
            turn_right = True
        key = keyboard.getKey()

    # Set speed based on key press
    if move_forward:
        set_speed(50.0)  # Forward speed
    elif move_backward:
        set_speed(-10.0)  # Reverse speed
    else:
        set_speed(0.0)  # Stop if neither UP nor DOWN is pressed

    # Set steering based on key press
    if turn_left:
        set_steering_angle(-0.3)  # Steer left
    elif turn_right:
        set_steering_angle(0.3)  # Steer right
    else:
        set_steering_angle(0.0)  # Go straight if neither LEFT nor RIGHT is pressed

# Main control loop
while driver.step() != -1:
    # Get keyboard input for manual control
    check_keyboard()

    # If GPS is enabled, update GPS position and speed
    if has_gps:
        gps_coords = gps.getValues()
        gps_speed = gps.getSpeed() * 3.6  # Convert from m/s to km/h

    # If display is enabled, update the speedometer display
    if enable_display:
        display.imagePaste(speedometer_image, 0, 0, False)
        current_speed = driver.getCurrentSpeed()
        if math.isnan(current_speed):
            current_speed = 0.0
        alpha = current_speed / 260.0 * 3.72 - 0.27
        x = -50.0 * math.cos(alpha)
        y = -50.0 * math.sin(alpha)
        display.drawLine(100, 95, int(100 + x), int(95 + y))

    # Update camera and LIDAR data if enabled
    if has_camera:
        camera_image = camera.getImage()
    if enable_collision_avoidance:
        sick_data = sick.getRangeImage()
