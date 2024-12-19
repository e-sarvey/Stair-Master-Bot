import threading
import dearpygui.dearpygui as dpg
import paho.mqtt.client as mqtt


MQTT_BROKER = "10.243.82.33"
COUNTERWEIGHT_TOPIC = "hardware/stepper/front"
LIFT_TOPIC = "hardware/stepper/rear"
FRONT_MOTOR_TOPIC = "hardware/motors/front"
REAR_MOTOR_TOPIC = "hardware/motors/rear"

# Step values for original group
STEPS_TO_FRONT = -500 
STEPS_TO_BACK = -STEPS_TO_FRONT  
STEPS_PER_STAIR = 2400 

# Step values for duplicated group
FIXED_STEP = 50

button_width = 120
button_height = 60
x_offset = 50  # Offset to move everything to the right

# MQTT Setup
client = mqtt.Client()
client.connect(MQTT_BROKER)

# Run MQTT client in a separate thread
def mqtt_loop():
    client.loop_forever()

mqtt_thread = threading.Thread(target=mqtt_loop)
mqtt_thread.daemon = True
mqtt_thread.start()

# Publish Messages for Counterweight (Original Group)
def move_weight_to_front():
    message = str(STEPS_TO_FRONT)
    client.publish(COUNTERWEIGHT_TOPIC, message)
    print(f"Published to {COUNTERWEIGHT_TOPIC}: {message}")

def move_weight_to_back():
    message = str(STEPS_TO_BACK)
    client.publish(COUNTERWEIGHT_TOPIC, message)
    print(f"Published to {COUNTERWEIGHT_TOPIC}: {message}")

# Publish Messages for Lift (Original Group)
def lift_one_stair():
    message = str(STEPS_PER_STAIR)
    client.publish(LIFT_TOPIC, message)
    print(f"Published to {LIFT_TOPIC}: {message}")

def lower_one_stair():
    message = str(-1 * (STEPS_PER_STAIR-FIXED_STEP))
    client.publish(LIFT_TOPIC, message)
    print(f"Published to {LIFT_TOPIC}: {message}")

# Publish Messages for Counterweight (Duplicated Group)
def move_fixed_weight_to_front():
    message = str(FIXED_STEP)
    client.publish(COUNTERWEIGHT_TOPIC, message)
    print(f"Published to {COUNTERWEIGHT_TOPIC}: {message}")

def move_fixed_weight_to_back():
    message = str(-1 * FIXED_STEP)
    client.publish(COUNTERWEIGHT_TOPIC, message)
    print(f"Published to {COUNTERWEIGHT_TOPIC}: {message}")

# Publish Messages for Lift (Duplicated Group)
def lift_fixed_stair():
    message = str(FIXED_STEP)
    client.publish(LIFT_TOPIC, message)
    print(f"Published to {LIFT_TOPIC}: {message}")

def lower_fixed_stair():
    message = str(-1 * FIXED_STEP)
    client.publish(LIFT_TOPIC, message)
    print(f"Published to {LIFT_TOPIC}: {message}")

# Publish Messages for Motors
def turn_front_motor_on():
    message = "on"
    client.publish(FRONT_MOTOR_TOPIC, message)
    print(f"Published to {FRONT_MOTOR_TOPIC}: {message}")

def turn_rear_motor_on():
    message = "on"
    client.publish(REAR_MOTOR_TOPIC, message)
    print(f"Published to {REAR_MOTOR_TOPIC}: {message}")

dpg.create_context()

with dpg.window(label="Control Panel", width=800 + x_offset, height=600):
    # Original Group

    # Top Button - Lift One Stair
    dpg.add_button(label="Up One Stair", width=button_width, height=button_height, callback=lift_one_stair, pos=[button_height + x_offset, button_height])
    # Left and Right Buttons - Counterweight Controls
    dpg.add_button(label="Weight to Back", width=button_width, height=button_height, callback=move_weight_to_back, pos=[-20 + x_offset, 130])
    dpg.add_button(label="Weight to Front", width=button_width, height=button_height, callback=move_weight_to_front, pos=[120 + x_offset, 130])
    # Bottom Button - Lower One Stair
    dpg.add_button(label="Down One Stair", width=button_width, height=button_height, callback=lower_one_stair, pos=[button_height + x_offset, 210])

    # Duplicated Group (Shifted to the right)
    offset_x = 300  # Space between groups

    # Top Button - Lift Fixed Stair
    dpg.add_button(label="Lift +", width=button_width, height=button_height, callback=lift_fixed_stair, pos=[button_height + offset_x + x_offset, button_height])
    # Left and Right Buttons - Counterweight Controls
    dpg.add_button(label="Weight +", width=button_width, height=button_height, callback=move_fixed_weight_to_back, pos=[-20 + offset_x + x_offset, 130])
    dpg.add_button(label="Weight -", width=button_width, height=button_height, callback=move_fixed_weight_to_front, pos=[120 + offset_x + x_offset, 130])
    # Bottom Button - Lower Fixed Stair
    dpg.add_button(label="Lift -", width=button_width, height=button_height, callback=lower_fixed_stair, pos=[button_height + offset_x + x_offset, 210])

    # Motor Controls

    motor_offset_y = 300  # Position motors below the other controls
    dpg.add_button(label="Front Motor On", width=button_width, height=button_height, callback=turn_front_motor_on, pos=[button_height + x_offset, motor_offset_y])
    dpg.add_button(label="Rear Motor On", width=button_width, height=button_height, callback=turn_rear_motor_on, pos=[button_height + offset_x + x_offset, motor_offset_y])

# Finalizing and Running the Interface
dpg.create_viewport(title="Control Panel", width=600 + x_offset, height=500)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()

client.loop_stop()  # Stop MQTT loop when GUI is closed
