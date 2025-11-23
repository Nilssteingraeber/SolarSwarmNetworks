from numpy import array
from numpy.linalg import norm
from random import randint

class MockPosition(object):
    def __init__(self):
        # mock routes for mock coordinates
        # decimals calculated as d(a°b'c") = a + b/60 + c/3600
        # movement is calculated by treating coordinates as points/vectors in a two-dimensional vector space 
        start = array([51.450388888888895, 7.276472222222222]) # 51°27'01.4"N 7°16'35.3"E
        self.__goal = 1 # index of next point in self.points
        self.__max_vec_len = 5.288731323495855e-05 # = distance between start and arbitrary point nearby/12
        self.__mock_routes = [ # has to have at least one route
            # sample route (line) using start as a starting point
            [start, # pass by value
                array([51.44927777777777, 7.2773055555555555])], # 51°26'57.4"N 7°16'38.3"E
            # sample route (triangle) using start as a starting point
            [start,
                array([51.451, 7.276833333333333]), # 51°27'03.6"N 7°16'36.6"E
                array([51.45080555555556, 7.276083333333333])], # 51°27'02.9"N 7°16'33.9"
            # sample route (quadrangle) using start as a starting point
            [start,
                array([51.45063888888889, 7.277055555555555]), # 51°27'02.3"N 7°16'37.4"E
                array([51.451, 7.276833333333333]), # 51°27'03.6"N 7°16'36.6"E
                array([51.45080555555556, 7.276083333333333])], # 51°27'02.9"N 7°16'33.9"E
        ]
        route_variant = randint(0, len(self.__mock_routes)-1)
        self.get_logger().debug('Route %d selected' % (route_variant,))
        self.__points = self.__mock_routes[route_variant]
        self.__current = self.__points[self.__goal] # array is passed by value/copied
    

    # read-only properties
    @property
    def mock_routes(self):
        return self.__mock_routes

    # read-write properties
    @property
    def points(self):
        return self.__points
    @points.setter # can be replaced with own list of points
    def points(self, p):
        if type(p) == type(self.__points) and len(p) > 0: # must be (not empty) list
            for point in p:
                if type(point).__name__ == 'ndarray': # each element must be of numpy.ndarray
                    if len(point) == len(self.__points[0]) and type(point[0]).__name__ == 'float64': # each element must have same dimension and content type (content type identical across an ndarray)
                        self.__points = p
                        return
        raise ValueError('Failed to set new points.')
    @property
    def goal(self):
        return self.__goal
    @goal.setter
    def goal(self, g):
        if type(g).__name__ == 'int' and g >= 0 and g < len(self.__points):
            self.__goal = g # valid type and value range
    @property
    def max_vec_len(self):
        return self.__max_vec_len
    @max_vec_len.setter
    def max_vec_len(self, m):
        if type(m).__name__ == 'float' and m >= 0.0:
            self.__max_vec_len = m # valid type and value
    @property
    def current(self):
        return self.__current
    @current.setter
    def current(self, a):
        if type(a).__name__ == 'ndarray' and len(a) == len(self.__current):
            # only checks type and length but not element type
            # as current is set frequently, this setter should not be too expensive
            self.__current = a
    
    def advance_position(self) -> bool:
        vec_to_point = self.points[self.goal] - self.current # calculate vector to next point
        vec_len = norm(vec_to_point) # calculate vector length

        if vec_len > self.max_vec_len:
            self.get_logger().debug('Vector: %s' % (str(vec_to_point),))
            vec_to_point = vec_to_point * (self.max_vec_len/vec_len) # shorten vector to max_vec_len
            # if, for instance, vec_len is twice as large as max_vec_len, vec_to_point will be halved
            self.current = self.current + vec_to_point # move towards goal
            return False # did not reach goal
        else:
            self.current = self.points[self.goal] # move if distance is less or equal max_vec_len towards goal
            self.get_logger().debug('Reached point %d' % (self.goal,))
            return True # reached goal
