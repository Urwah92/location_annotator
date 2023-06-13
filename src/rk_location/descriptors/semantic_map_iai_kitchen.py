from robokudo.semantic_map import SemanticMapEntry, BaseSemanticMap


class SemanticMap(BaseSemanticMap):
    def __init__(self):
        super().__init__()
        # SEMANTIC MAP ENTRIES BEGIN
        semantic_map_entries = [SemanticMapEntry(name="kitchen_island", frame_id="map", type="CounterTop",
                         position_x=2.30, position_y=2.72, position_z=1.40,
                         orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                         x_size=1.0, y_size=3.18, z_size=0.85),
                                SemanticMapEntry(name="A", frame_id="map", type="CounterTop",
                                                 position_x=2.30, position_y=2.52, position_z=1.40,
                                                 orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                                                 x_size=1.0, y_size=0.18, z_size=0.85),
                                SemanticMapEntry(name="B", frame_id="map", type="CounterTop",
                                                 position_x=2.30, position_y=2.72, position_z=1.40,
                                                 orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                                                 x_size=1.0, y_size=0.18, z_size=0.85),
                                SemanticMapEntry(name="C", frame_id="map", type="CounterTop",
                                                 position_x=2.30, position_y=2.97, position_z=1.40,
                                                 orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                                                 x_size=1.0, y_size=0.18, z_size=0.85),

                                SemanticMapEntry(name="D", frame_id="map", type="CounterTop",
                                                 position_x=2.30, position_y=2.35, position_z=1.40,
                                                 orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                                                 x_size=1.0, y_size=0.18, z_size=0.85),
                                SemanticMapEntry(name="E", frame_id="map", type="CounterTop",
                                                 position_x=2.30, position_y=2.15, position_z=1.40,
                                                 orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                                                 x_size=1.0, y_size=0.18, z_size=0.85),
                                ]

        # SEMANTIC MAP ENTRIES END
        self.add_entries(semantic_map_entries)

