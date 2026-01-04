# Parrot Sequoia Multispectral Sunshine Sensor Data
# Measures incoming light irradiance across different spectral bands

std_msgs/Header header

# Green band (550 nm ±40 nm)
float64[2] green

# Red band (660 nm ±40 nm)
float64[2] red

# Red Edge band (735 nm ±10 nm)
float64[2] red_edge

# Near Infrared band (790 nm ±40 nm)
float64[2] nir
