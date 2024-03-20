class Static_Object: 
    def __init__(self, x, y, radius, influence, attraction) -> None:
        self.vektor = (x,y)
        self.radius_of_influence = radius
        self.influence = influence
        self.attraction = attraction 