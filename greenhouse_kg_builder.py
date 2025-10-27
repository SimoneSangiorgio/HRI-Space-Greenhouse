import networkx as nx

def create_greenhouse_graph():
    """Creates and returns the detailed STATIC greenhouse knowledge graph."""
    graph = nx.DiGraph()

    # Define Nodes (Entities in the world)
    nodes = [
        # The Robot
        {"id": "tiago_robot", "label_node": "Robot", "name": "TIAGo Robot", "battery_percent": None},
        
        # Resources
        {"id": "water_tank_1", "label_node": "ResourceTank", "name": "Water Tank 1", "level_percent": None},
        {"id": "water_tank_2", "label_node": "ResourceTank", "name": "Water Tank 2", "level_percent": None},
        {"id": "water_tank_3", "label_node": "ResourceTank", "name": "Water Tank 3", "level_percent": None},
        {"id": "solar_storage", "label_node": "ResourceStorage", "name": "Solar Battery", "energy_percent": None},

        # Plants in Bays
        {"id": "plant_1_basil", "label_node": "Plant", "name": "Basil", "species": "Basil", "bay_id": 1, "soil_moisture_percent": None, "leaf_color": None, "lux_level": None},
        {"id": "plant_2_lemon", "label_node": "Plant", "name": "Lemon Tree", "species": "Lemon", "bay_id": 2, "soil_moisture_percent": None, "leaf_color": None, "lux_level": None},
        {"id": "plant_3_moss", "label_node": "Plant", "name": "Moss Bed", "species": "Moss", "bay_id": 3, "soil_moisture_percent": None, "leaf_color": None, "lux_level": None},

        # Environmental Sensors
        {"id": "env_sensors", "label_node": "SensorSet", "name": "Greenhouse Ambience Sensors", "temperature_celsius": None, "humidity_percent": None}
    ]

    for node in nodes:
        node_id = node.pop('id') # Use 'id' as the node key, the rest as attributes
        graph.add_node(node_id, **node)

    # Define Edges (Relationships between entities)
    # This is simple for now but can be expanded (e.g., which tank waters which plant)
    graph.add_edge("water_tank_1", "plant_1_basil", label_edge="PROVIDES_WATER_TO")
    graph.add_edge("water_tank_2", "plant_2_lemon", label_edge="PROVIDES_WATER_TO")
    graph.add_edge("water_tank_3", "plant_3_moss", label_edge="PROVIDES_WATER_TO")
    
    print(f"Static greenhouse graph created with {graph.number_of_nodes()} nodes and {graph.number_of_edges()} edges.")
    return graph

if __name__ == '__main__':
    # For testing this script directly
    greenhouse_graph = create_greenhouse_graph()
    print("\nExample Node Data (Water Tank 1):")
    print(greenhouse_graph.nodes['water_tank_1'])