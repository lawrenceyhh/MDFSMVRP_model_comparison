import folium


def visualize_tours(network, vertices, tours, name, color="green"):
    LOCATIONS = {
        "Paris": [48.864716, 2.349014],
        "Shanghai": [31.224361, 121.469170],
        "NewYork": [40.730610, -73.935242]
    }

    # Create map object
    m = folium.Map(location=LOCATIONS[network.name], zoom_start=12)

    # Create markers
    for id in vertices:
        if id == 'D0':
            folium.Marker(
                [network.node_dict[id].location[1], network.node_dict[id].location[0]],
                popup='<strong>' + str(id) + '</strong>',
                icon=folium.Icon(color='black', icon='home')
            ).add_to(m)
            continue

        folium.CircleMarker(
            [network.node_dict[id].location[1], network.node_dict[id].location[0]],
            popup='<strong>' + str(id) + '</strong>',
            color="#8e2228",
            fill=True,
            fill_color="#3139cc",
            radius=3
        ).add_to(m)
    
    # Create later
    shapesLayer = folium.FeatureGroup(name="Vector Shapes").add_to(m)
    
    if name == "solution_F3":
        line_color = "green"
    elif name == "solution_F4":
        line_color = "blue"
    elif name == "solution_F5":
        line_color = "red"
    else:
        line_color = color
    
    for (i, j) in tours:
        folium.PolyLine([(network.node_dict[i].location[1], network.node_dict[i].location[0]),
                         (network.node_dict[j].location[1], network.node_dict[j].location[0])],
                        color=line_color, weight=3).add_to(shapesLayer)
    map_name = name + ".html"
    m.save(map_name)