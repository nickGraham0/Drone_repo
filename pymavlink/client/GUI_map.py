from GUI_handlers import on_drone_loc

import asyncio
import folium


async def refresh_map(map_label, root):

    while True: 
        map_label.set_html(open("gui_map.html").read())
        root.after(1000, refresh_map)  # Refresh every second (adjust as needed)


async def update_drone_loc(rate=1):
    #In a loop
    #Ask for drone location
    #Update the map

    drone_loc = on_drone_loc()
    my_map1 = folium.Map(location=drone_loc, zoom_start=12)
    marker = folium.Marker(location=drone_loc, popup='Drone Location')
    marker.add_to(my_map1)

    while True:
        drone_loc = on_drone_loc()


        # CircleMarker with radius
        marker.location = drone_loc  # Update marker location

        #Add Line
        #folium.PolyLine(locations = [loc, loc2], line_opacity = 0.5).add_to(my_map1)

        # save method of Map object will create a map
        my_map1.save("gui_map.html")
        await asyncio.sleep(rate)

