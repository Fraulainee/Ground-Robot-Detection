import math

# GPS reference point
ref_latitude = -35.363262
ref_longitude = 149.165237

# Drone's GPS location
drone_latitude = -35.3632598
drone_longitude = 149.1652393

# Earth's radius in meters
EARTH_RADIUS = 6371000

# Convert degrees to radians
ref_latitude_rad = math.radians(ref_latitude)
ref_longitude_rad = math.radians(ref_longitude)
drone_latitude_rad = math.radians(drone_latitude)
drone_longitude_rad = math.radians(drone_longitude)

# Calculate the distance between the reference point and the drone's location
delta_latitude = drone_latitude_rad - ref_latitude_rad
delta_longitude = drone_longitude_rad - ref_longitude_rad
distance = 2 * EARTH_RADIUS * math.asin(math.sqrt(
    math.sin(delta_latitude / 2) ** 2 + 
    math.cos(ref_latitude_rad) * math.cos(drone_latitude_rad) * 
    math.sin(delta_longitude / 2) ** 2
))

# Calculate the x and y offsets
x_offset = distance * math.cos(ref_latitude_rad) * math.cos(delta_longitude)
y_offset = distance * math.cos(ref_latitude_rad) * math.sin(delta_longitude)

print("X Offset:", x_offset)
print("Y Offset:", y_offset)
