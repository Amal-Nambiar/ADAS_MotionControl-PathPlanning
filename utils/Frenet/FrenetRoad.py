class FrenetRoad:

    def __init__(self,frenet_converter, lane_width=3.5,n_lanes=3):

        self.frenet = frenet_converter
        self.ref = frenet_converter.reference_line
        self.lane_width = lane_width
        self.n_lanes = n_lanes

    def create_lanes(self):
        lanes = {}
        center_idx = self.n_lanes // 2
        for lane_idx in range(self.n_lanes):

            d_offset = (lane_idx - center_idx) * self.lane_width
            lane_x = []
            lane_y = []
            for s_val in self.ref.s:
                x, y = (self.frenet.frenet_to_cartesian( s_val, d_offset))
                lane_x.append(x)
                lane_y.append(y)
            lanes[f"lane_{lane_idx}"] = (lane_x,lane_y)
        return lanes