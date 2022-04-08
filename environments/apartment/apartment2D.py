class Apartment:
    def __init__(self, livingroom, bedroom, kitchen, restroom):
        self.livingroom = livingroom
        self.bedroom = bedroom 
        self.kitchen = kitchen 
        self.restroom = restroom 

    def get_regions(self):
        # regions = {'tv-couch-space': [[(x1, y1), (x2, y2)]]}
        pass
    
    def get_obstacles(self):
        # obstacles = [all_walls_dimensions_in_list + tv_pose]
        pass 

    def get_movable_objects(self):
        #{'yogamat' : (x, y)}
        pass 





'''
apartment : [(0, 0), (800, 800)]
first_section : [(0, 0), (400, 800)], [(0, 0), (800, 400)]
second_section : [(400, 0), (800, 800)], [(0, 400), (800, 800)]
big1 = [[(0, 0), (400, 800)], [(400, 0), (800, 800)]]
big2 = [[(0, 0), (800, 400)], [(0, 400), (800, 800)]]

four_partitions = [[(0, 400), (0, 400)], [(400, 0), (800, 400)], [(0, 400), (400, 800)], [(400, 400), (800, 800)]]
three_partitions = random.select(four_partitions, 3)






livingroom = {'walls' :[[(x1, y1), (x2, y2)], [(x1, y1), (x2, y2)]], 
                'objects': {'fixed': {'tv' : [(x1, y1), (x2, y2)], 'couch' :[(x1, y1), (x2, y2)], 'tv-couch-space':[(x1, y1), (x2, y2)],
                            'office-desk':[(x1, y1), (x2, y2)],
                            'yogaspace' :[(x1, y1), (x2, y2)], 'gym-equipments' :[(x1, y1), (x2, y2)],
                            'dining-area' :[(x1, y1), (x2, y2)], 'guitar-stand' :[(x1, y1), (x2, y2)]}, 
                'movable': {'arm-chair' :[(x1, y1)], 'yogamat': [(x1, y1), (x2, y2)], 'legrest': [(x1, y1), (x2, y2)], 
                            'bowls' : [(x1, y1), (x2, y2)], 'water-bottle': [(x1, y1)]}}}
bedroom = {'walls' :[[(x1, y1), (x2, y2)], [(x1, y1), (x2, y2)]], 'objects': {'fixed': {'bed' : [(x1, y1), (x2, y2)]}, 'movable': {'guitar' :[(x1, y1)]}}}
kitchen = {'walls' :[[(x1, y1), (x2, y2)], [(x1, y1), (x2, y2)]], 'objects': {'fixed': {'dishwasher' : [(x1, y1), (x2, y2)], 'oven' : [(x1, y1), (x2, y2)]}, 
            'movable': {'water-bottle' :[(x1, y1)]}}}
restroom = {'walls' :[[(x1, y1), (x2, y2)], [(x1, y1), (x2, y2)]], 'objects': {'fixed': {'cabinet' : [(x1, y1), (x2, y2)]}, 
            'movable': {'nailtrimmer' :[(x1, y1)]}}}

rooms: tan,
fixed: brown,
walls : black, 
robot: light_grey,
guitar: light_red, 
arm-chair: light brown, 
yogamat: light-blue, 
legrest: light green, 
dishes: white, circular,
water-bottle: pink, 
nailtrimmer: white

task-distribution: [[0.8, [("guitar", "guitar-stand"), ("yogamat", "yogaspace"), ("bowls", dishwasher),
                            ("nailtrimmer", "cabinet")]], [0.2, [("yogamat", "tv-couch-space")]]]
'''
