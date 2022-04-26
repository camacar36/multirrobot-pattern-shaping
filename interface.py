from module import *
from features import *

import pygame, sys

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
    global super_grid,sol
    for i in range(len(super_grid)):
        for j in range(len(super_grid[i])):
            super_grid[i][j] = False
    sol = []

def print_path():
    global sol
    global path
    print(len(path))
    sol = []
    if len(path) <= NUM_ROBOTS:
        sol = path.copy() 
    else:
        i = aux = (len(path) - (len(path) / NUM_ROBOTS)) / (NUM_ROBOTS - 2)
        sol.append(path[0])
        while(i <= len(path)):
            aux2 = int(i)
            sol.append(path[aux2])
            i = i + aux
        sol.append(path[-1])
        
    print("-----")
    print(len(sol), len(path))
    for i in sol:
        print(i.x, i.y)

def send_sol():
    print("enviando solución")

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
    title_t.x, title_t.y    =   52, 15
    pt_rb_t.x, pt_rb_t.y    =   56, 60 
    info_t.x,  info_t.y     =   20, 465
    screen.blit(pygame.image.load(config_title), title_t)
    screen.blit(pygame.image.load(config_pt_rbt),pt_rb_t)
    screen.blit(pygame.image.load(config_info),  info_t)
    
    # Fill the screen with the widgets and configuration
    btn_delete.draw(screen, clean_mat)
    btn_draw.draw(screen, print_path)
    btn_send.draw(screen, send_sol)

    # Drawing the grid
    for i in range(GRID_L,GRID_W,CELL_W):
        for j in range(GRID_T,GRID_H,CELL_H):
            grid = pygame.Rect(i,j,CELL_W,CELL_H)
            pygame.draw.rect(screen,lightgrey, grid,CELL_BORDER_WIDTH)
            izq = pygame.mouse.get_pressed()[0]
            dch = pygame.mouse.get_pressed()[2] 
            if izq or dch:
                x,y = pygame.mouse.get_pos()
                if x > i and x < i+CELL_W and y > j and y < j+CELL_H:
                    if izq:                  
                        super_grid[int((i-GRID_L)/CELL_W)][int((j-GRID_T)/CELL_H)] = True
                    if dch:
                        super_grid[int((i-GRID_L)/CELL_W)][int((j-GRID_T)/CELL_H)] = False
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

    # Updating the app
    pygame.display.flip()


