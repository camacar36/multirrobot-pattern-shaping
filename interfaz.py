#!/usr/bin/env python

from lib_interface import *

from geometry_msgs.msg import Polygon, Point32
import rospy

import pygame, sys

# Initializing ros node
rospy.init_node("interface_node")

sol_pub = rospy.Publisher('/goal_points', Polygon, queue_size=5)

# Initializing pygame
pygame.init()

# Initializing the main window
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption(config_caption)
pygame.display.set_icon(pygame.image.load(config_icon))

# Creating the panel for the buttons
panel   = pygame.Rect(PANEL_L, PANEL_T, PANEL_WIDTH, PANEL_HEIGHT)
border  = pygame.Rect(BORDER_L,BORDER_T,BORDER_WIDTH,BORDER_HEIGHT)

# Initializing the form and titles of the main window
title_t = pygame.image.load(config_title).get_rect()
pt_rb_t = pygame.image.load(config_pt_rbt).get_rect()
info_t  = pygame.image.load(config_info).get_rect()
coord_s = pygame.image.load(config_axes).get_rect()

# Creating the buttons to use
btn_delete  = Button("BORRAR",  190,50,(15,520), 3,pygame.font.Font(None,20),superlightgrey,grey)
btn_draw    = Button("DIBUJAR", 190,50,(230,520),3,pygame.font.Font(None,20),superlightgrey,grey)
btn_send    = Button("ENVIAR",  190,50,(445,520),3,pygame.font.Font(None,20),superlightgrey,grey)

# Declaration and definition of the functions of the buttons and others
def fill_super_grid():
    global super_grid
    for i in range(0,SUPER_GRID_WIDTH):
        aux = []
        for j in range(0,SUPER_GRID_HEIGHT):
            aux.append(False)
        super_grid.append(aux)

def clean_mat():
    global super_grid,sol,dic
    for i in range(len(super_grid)):
        for j in range(len(super_grid[i])):
            super_grid[i][j] = False
    sol = []
    dic = {}

def print_path():
    global sol
    global path
    global dic

    # Preprocessing of the dic list
    k = list(dic.keys())
    print(k)
    keys = []
    for i in k:
        x = int(i/100)
        y = i%100
        keys.append(Point(x,y))
    
    for _k in keys:
        print(_k.x, _k.y)

    # print(len(path))
    sol = []
    if len(keys) <= NUM_ROBOTS:
        sol = keys.copy() 
    else:
        i = aux = (len(keys) - (len(keys) / NUM_ROBOTS)) / (NUM_ROBOTS - 2)
        sol.append(keys[0])
        while(i <= len(keys)):
            aux2 = int(i)
            sol.append(keys[aux2])
            i = i + aux
        sol.append(keys[-1])
        
    print("-----")
    print(len(sol), len(keys), len(path))
    for i in sol:
        print(i.x, i.y)

def send_sol():

    poly = Polygon()
    for s in sol:
        poly.points.append(Point32(s.y*2/10,s.x*2/10,0))
        
   
    sol_pub.publish(poly)
# Initializing the solution grid
fill_super_grid()


# Infinite loop where the screen updates
while True:

    # First, check the event list
    for event in pygame.event.get():
        # Check if there's any QUIT event
        if event.type == pygame.QUIT: sys.exit()
    

    # Fill screen with background color, panel for buttons and border
    screen.fill(superlightgrey)
    pygame.draw.rect(screen,lightgrey,panel, PANEL_BORDER_WIDTH)
    pygame.draw.rect(screen,lightgrey,border,BORDER_BORDER_WIDTH)

    # Placing the titles
    title_t.x, title_t.y    =   65, 10
    pt_rb_t.x, pt_rb_t.y    =   70, 55
    info_t.x,  info_t.y     =   20, 465
    coord_s.x, coord_s.y    =   -2, -2
    screen.blit(pygame.image.load(config_title), title_t)
    screen.blit(pygame.image.load(config_pt_rbt),pt_rb_t)
    screen.blit(pygame.image.load(config_info),  info_t)
    screen.blit(pygame.image.load(config_axes),coord_s)
    
    # Fill the screen with the widgets and configuration
    btn_delete.draw(screen, clean_mat)
    btn_draw.draw(screen, print_path)
    btn_send.draw(screen, send_sol)

    # Drawing the grid
    for i in range(GRID_L,GRID_W,CELL_W):
        for j in range(GRID_T,GRID_H,CELL_H):
            # Drawing the main grid
            grid = pygame.Rect(i,j,CELL_W,CELL_H)
            pygame.draw.rect(screen,lightgrey, grid,CELL_BORDER_WIDTH)

            # Getting the mouse events in order to draw or delete the path - this path is saved at super_grid
            izq = pygame.mouse.get_pressed()[0]
            dch = pygame.mouse.get_pressed()[2] 
            if izq or dch:
                x,y = pygame.mouse.get_pos()
                if x > i and x < i+CELL_W and y > j and y < j+CELL_H:
                    if izq:                  
                        super_grid[int((i-GRID_L)/CELL_W)][int((j-GRID_T)/CELL_H)] = True
                        dic.update({int((i-GRID_L)/CELL_W)*100 + int((j-GRID_T)/CELL_H):True})
                    if dch:
                        super_grid[int((i-GRID_L)/CELL_W)][int((j-GRID_T)/CELL_H)] = False
                        try:
                            dic.pop(int((i-GRID_L)/CELL_W)*100 + int((j-GRID_T)/CELL_H))
                        except KeyError:
                            continue
                        sol = []
    
    # Drawing the selected grids (Done when right button of the mouse is pressed)
    path = []
    for i in range(0,SUPER_GRID_WIDTH):
        for j in range(0,SUPER_GRID_HEIGHT):
            if super_grid[i][j] == True: 
                rect = pygame.Rect(i*CELL_W+GRID_L,j*CELL_H+GRID_T,CELL_W,CELL_W)
                pygame.draw.rect(screen,white,rect,0)
                path.append(Point(i,j))

    # Drawing the 10 points (in case it had been selected 10 or more points on the grid) 
    # where the robots will place ()
    for s in sol:
        rect = pygame.Rect(s.x*CELL_W+GRID_L,s.y*CELL_H+GRID_T,CELL_W,CELL_H)
        pygame.draw.rect(screen,lightblue,rect,0)

    mx, my = pygame.mouse.get_pos()
    if mx > 20 and mx < 35 and my > 465 and my < 480:
        info_panel = pygame.Rect(25, 330, 520, 115)
        pygame.draw.rect(screen, light_white, info_panel,0)

        triangle_points = [(25, 445),(25, 462),(40, 445)]
        pygame.draw.polygon(screen, light_white, triangle_points, 0)

        for t in range(len(info_text)):
            our_font = pygame.font.Font(current_file_path + "/fonts/ARCADE_R.TTF",8)
            if t == 0:

                our_font.set_underline(True)
                our_font_surface = our_font.render(info_text[t],True,'#000000')
                our_font_text_rect = our_font_surface.get_rect()
                our_font_text_rect.x, our_font_text_rect.y = 200, 336 + t * 20

            else:

                our_font_surface = our_font.render(info_text[t],True,'#000000')
                our_font_text_rect = our_font_surface.get_rect()
                our_font_text_rect.x, our_font_text_rect.y = 29, 336 + t * 20

            screen.blit(our_font_surface, our_font_text_rect)


        

    # Updating the app
    pygame.display.flip()