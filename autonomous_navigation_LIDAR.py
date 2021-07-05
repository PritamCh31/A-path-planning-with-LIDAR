import pygame
from PIL import Image
from numpy import *
import numpy as np
import heapq
import math
from pygame import gfxdraw

pygame.init()
pygame.display.set_caption('operation window') # caption of our pygame window
image = pygame.image.load(r'C:\Users\User\Desktop\lidar_images\pic6.JPG') # backgroung image is loaded
LENGTH = image.get_width()
BREDTH = image.get_height()
print("the length of the window is = ", LENGTH, "\nthe bredth of the window is = ", BREDTH)
window = pygame.display.set_mode((LENGTH, BREDTH)) # dimension of the pygame window
clock = pygame.time.Clock()

#different colour codes

red = (255, 0, 0)
black = (0, 0, 0)
grey = (128, 128, 128)
purple = (128, 0, 128)         
orange = (255, 165, 0)
green = (0, 255, 0)

im = Image.open(r"C:\Users\User\Desktop\lidar_images\pic6.JPG") # same image is loaded again to convert it into binary array
temp = im.convert("1") # the image is now converted
A = np.array(temp) # array is initialized
new_A = empty((A.shape[0], A.shape[1]), None)

for i in range(len(A)):
    for j in range(len(A[i])):
        if A[i][j] == True:
            new_A[i][j] = 1
        else:
            new_A[i][j] = 0
#2d array of 1 and 0 is formed
x = array((new_A), dtype=int)
grid = np.array(new_A)
print(x)
#the background image is set from 0, 0    
window.blit(image, (0, 0))     

def heuristic(a, b):# calculates the heuristic distane between two points
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar_algorithm(grid, start, goal):# main a_star algorithm
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(-1,1),(-1,-1),(1,-1)] #coordinates of the neighbour node
    close_set = set() #close set initializes
    came_from = {} #a special set to store the parent nodes
    gscore = {start:1}
    fscore = {start:heuristic(start, goal)} # f cost is calculated
    oheap = [] # open list initializes
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 1 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[0] < grid.shape[0]:                
                    if grid[neighbor[1]][neighbor[0]] == 1:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False
import math
import pygame
from pymunk.vec2d import Vec2d #custom library
pygame.init() #initializing all pygame modules
import math

# creating a class that sets a vehicle up, update it's positions and orientation


#colours in RGB
done = False
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
purple = (128, 0, 128) 
GREEN = (0, 200, 0)
RED = (255, 0, 0)
mouseX = 0 #mouse drag
mouseY = 0
path = []

ratio = 7
numbers = 70

size = ratio * numbers #pygame window size
window = pygame.display.set_mode((LENGTH, BREDTH)) #set up pygame window
pygame.display.set_caption("Path Following") #Set the current window caption
clock = pygame.time.Clock() #set a variable for how quick the game runs

def a(x, y):
    return x
def b(x, y):
    return y
class Vec2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self, new):
        self.x += new.x
        self.y += new.y

    def add_vect(self, new):
        return Vec2d(self.x + new.x, self.y + new.y)

    def sub(self, new):
        self.x -= new.x
        self.y -= new.y

    def sub_vect(self, new):
        return Vec2d(self.x - new.x, self.y - new.y)

    def angle(self):
        return math.atan2(-self.y,self.x)

    def limit(self, max_):
        if self.mag() > max_:
            self.set_mag(max_)

    def mag(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def set_mag(self, value):
        self.x, self.y = value*self.x/self.mag(), value*self.y/self.mag()

    def distance(self, new):
        return math.sqrt((self.x - new.x) * (self.x - new.x) + (self.y - new.y) * (self.y - new.y))

def show_path():
    pygame.draw.circle(window, purple, (a(*path[0]), b(*path[0])), 5) #circle(surface, color, center, radius)
    for i in range(path.__len__()-1):
        #for c, d in path:
        pygame.draw.line(window, RED, (a(*path[i]), b(*path[i])), (a(*path[i + 1]), b(*path[i + 1]))) #line(surface,colour,start position,end posion)

class Vehicle:
    def __init__(self, x, y): #initializing variables
        self.pos = Vec2d(x, y) #position
        self.vel = 3 #velocity
        self.acc = 0 #acceleration
        self.theta = 0 #orientation
        self.delta = 0 #steering angle
        self.alpha = 0 #angle between vehicle's body and look ahead line
        self.length = 8 #length of vehicle
        self.kaapa = 0 #curvature
        self.desired = 0.1 #look forward gain
        self.ld = 2 #look ahead distance

    def update(self): #updating values
        if self.delta > 1:
            self.delta = 1
        elif self.delta < -1:
            self.delta = -1
        self.vel += self.acc #kinematic formulas
        self.pos.x += self.vel * math.cos(-self.theta)
        self.pos.y += self.vel * math.sin(-self.theta)
        self.theta += self.vel * (math.tan(self.delta) / self.length)
        

    def seek_2(self, point): #pure pursuit implementation
        #self.ld = self.pos.distance(point)
        self.ld = int(math.sqrt((self.pos.x - a(*point)) ** 2 + (self.pos.y - b(*point)) ** 2))
        self.alpha = (path[0].sub_vect(Vec2d(self.pos.x - self.length * math.cos(-self.theta),
                                                  self.pos.y - self.length * math.sin(-self.theta)))).angle() - self.theta
        #self.alpha = 5
                                                  
        self.kaapa = (2 * math.sin(self.alpha))
        if math.atan2(self.kaapa * self.length, 1) - self.delta > 0.02:
            self.delta += 0.03
        elif self.delta - math.atan2(self.kaapa * self.length, 1) > 0.02:
            self.delta -= 0.03

    def show_vehicle(self): #path the vehicle
        pygame.draw.polygon(window, GREEN, rect(self.pos.x + (self.length/2) * math.cos(-self.theta),
                                                self.pos.y + (self.length/2) * math.sin(-self.theta),
                                                -self.theta, self.length, 8))


def rect(x, y, angle, w, h):
    return [translate(x, y, angle, -w/2,  h/2),
            translate(x, y, angle,  w/2,  h/2),
            translate(x, y, angle,  w/2, -h/2),
            translate(x, y, angle, -w/2, -h/2)]


def translate(x, y, angle, px, py):
    x1 = x + px * math.cos(angle) - py * math.sin(angle)
    y1 = y + px * math.sin(angle) + py * math.cos(angle)
    return [x1, y1]

def main():
    image = pygame.image.load(r'C:\Users\User\Desktop\lidar_images\pic6.JPG')
    window.blit(image, (0, 0))
    global path, path1
    grid = array(new_A)
    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            elif pygame.mouse.get_pressed()[0]:
                pos1 = pygame.mouse.get_pos()
                pygame.draw.circle(window, purple, pos1, 2, 2)
                print("start position is = ", pos1)
                start = pos1
            elif pygame.mouse.get_pressed()[2]:
                pos2 = pygame.mouse.get_pos()
                pygame.draw.circle(window, orange, pos2, 2, 2)
                print("target position is = ", pos2)
                goal = pos2
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    path = astar_algorithm(grid, start, goal)
                    path += [start]
                    path = path[:: - 1]
                    m = array(path)
                    path1 = m.flatten()
                    print("the planned coordinates are = \n", path)
                    #print(path1)
                    pygame.draw.lines(window, green, False, path, 2)
                    
                if event.key == pygame.K_r:
                    window.blit(image, (0, 0))
                    vehicle = Vehicle(a(*pos1), b(*pos1))
                    ld = 10
                    if len(path):
                        show_path()
                        #set_target()
                        while int(math.sqrt((a(*path[0]) - a(*pos1)) ** 2 + (b(*path[0]) - b(*pos1)) ** 2)) < ld:
                            if len(path) == 1:
                                break
                            path.pop(0)
                        
                        vehicle.seek_2(path[0])
                    vehicle.update()
                    
                    vehicle.show_vehicle()
                    pygame.display.flip()
                                    
        pygame.display.update()
    pygame.quit()

if __name__ == "__main__":
    main()