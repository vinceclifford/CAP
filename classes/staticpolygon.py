class StaticPolygon:

    def __init__(self, vertices, distance_of_influence, attraction, no_interference=10) -> None:
        """
        Args: vertices (list): list of tuples with two entries each. A x and a y coordinate distance_of_influence (
        int): Until which distance does attraction (int): How much influence does the polygon got. A "typical" value
        for this attribute is 100, the bigger the value the more influence it will have no_interference (int,
        optional): Distance around polygon that the robot must not interfere with the polygon. Defaults to 10.
        """
        self.distance_of_influence = distance_of_influence
        self.attraction = attraction
        self.vertices = vertices
        self.no_interference = no_interference
        self.x_max = vertices[0][0]
        self.y_max = vertices[0][1]
        self.x_min = vertices[0][0]
        self.y_min = vertices[0][1]

        for vertex in vertices:
            if vertex[0] < self.x_min:
                self.x_min = vertex[0]
            if vertex[0] > self.x_max:
                self.x_max = vertex[0]

            if vertex[1] < self.y_min:
                self.y_min = vertex[1]
            if vertex[1] > self.y_max:
                self.y_max = vertex[1]

        looking_for_duplicate = set()
        for vertex in vertices:
            if vertex in self.vertices:
                exit("One of the polygons has got a duplicate vertex in the vertex list. This is not allowed!")