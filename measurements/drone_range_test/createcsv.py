from gmplot import gmplot

# Place map
gmap = gmplot.GoogleMapPlotter(63.40710333333333, 10.47754, 18, apikey="AIzaSyC2wE-bU54q_OK_1lLdk81gK4VglwUiCrU")


def convert(latitude, longitude):
    lat_heading = 1 if latitude[0] == 'N' else -1
    long_heading = 1 if longitude[0] == 'E' else -1
    lat_deg = (int(latitude[1:3]) + float(latitude[3:10]) / 60) * lat_heading
    long_deg = (int(longitude[1:4]) + float(longitude[4:11]) / 60) * long_heading
    return (lat_deg, long_deg)


current_position = (63.40833333, 10.47500000)
with open("data/raw/combined.csv") as file:
    for line in file.readlines():
        stripped = line.strip()
        values = line.split(";")
        if values[0] == "GPS":
            latitude, longitude = convert(values[1], values[2])
            gmap.circle(latitude, longitude, 2, color="red", face_alpha=0.1)
            current_position = (latitude, longitude)
        if values[0] == "RADIO":
            gmap.circle(current_position[0], current_position[1], 2)

# Draw
gmap.draw("my_map.html")



