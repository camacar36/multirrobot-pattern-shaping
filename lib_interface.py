#!/usr/bin/env python

import pygame

import os

current_file_path = os.path.dirname(os.path.abspath(__file__))

WINDOW_SIZE = WINDOW_WIDTH, WINDOW_HEIGHT   = [650,600]
GRID_SIZE   = GRID_WIDTH,  GRID_HEIGHT      = [600,400]

PANEL_L             = 0
PANEL_T             = 500
PANEL_WIDTH         = WINDOW_WIDTH
PANEL_HEIGHT        = WINDOW_HEIGHT-PANEL_T
PANEL_BORDER_WIDTH  = 0

BORDER_L            = 0
BORDER_T            = 0
BORDER_WIDTH        = WINDOW_WIDTH
BORDER_HEIGHT       = WINDOW_HEIGHT - PANEL_HEIGHT
BORDER_BORDER_WIDTH = 15

CELL_W              = 10
CELL_H              = 10
CELL_BORDER_WIDTH   = 1

GRID_L              = 75 #125
GRID_W              = 575#525
GRID_T              = 80
GRID_H              = 480

SUPER_GRID_WIDTH    = 50
SUPER_GRID_HEIGHT   = 40

NUM_ROBOTS  = 10

config_loc      = current_file_path + "/config/icons/"
config_caption  = "Pattern shaping w Turtlebot2"
config_icon     = config_loc + "shapingpattern_icon.png"
config_title    = config_loc + "title.png"
config_pt_rbt   = config_loc + "points_and_robots.png"
config_info     = config_loc + "info.png"
config_axes     = config_loc + "coordinate_system.png"

white           = [255,255,255]
light_white     = [200,200,200]
superlightgrey  = [50,50,50]
lightgrey       = [30,25,25]
grey            = [18,18,18]
black           = [0,0,0]
lightblue       = [100,100,255]

super_grid      = []
path            = []
sol             = []

dic             = {}

info_text       = ["Información de uso:", 
                    "Pulse click izquierdo para dibujar",  
                    "Pulse click derecho para borrar", 
                    "Pulse dibujar para generar los puntos representativos del dibujo", 
                    "Pulse enviar para que los turtlebots comiencen a moverse"]
 

class Point: 
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Button:
    def __init__(self,text,width,height,pos,elevation,font, color_on, color_off):
    #Core attributes 
        self.pressed = False
        self.elevation = elevation
        self.dynamic_elecation = elevation
        self.original_y_pos = pos[1]
        self.color_on = color_on
        self.color_off = color_off

        # top rectangle 
        self.top_rect = pygame.Rect(pos,(width,height))
        self.top_color = self.color_off

        # bottom rectangle 
        self.bottom_rect = pygame.Rect(pos,(width,height))
        self.bottom_color = '#354B5E'
        #text
        self.text_surf = font.render(text,True,'#FFFFFF')
        self.text_rect = self.text_surf.get_rect(center = self.top_rect.center)

    def draw(self, screen, function):
    # elevation logic 
        self.top_rect.y = self.original_y_pos - self.dynamic_elecation
        self.text_rect.center = self.top_rect.center 

        self.bottom_rect.midtop = self.top_rect.midtop
        self.bottom_rect.height = self.top_rect.height + self.dynamic_elecation

        pygame.draw.rect(screen,self.bottom_color, self.bottom_rect,border_radius = 12)
        pygame.draw.rect(screen,self.top_color, self.top_rect,border_radius = 12)
        screen.blit(self.text_surf, self.text_rect)
        self.check_click(function)

    def check_click(self, function):
        mouse_pos = pygame.mouse.get_pos()
        if self.top_rect.collidepoint(mouse_pos):
            self.top_color = self.color_on
            if pygame.mouse.get_pressed()[0]:
                self.dynamic_elecation = 0
                self.pressed = True
            else:
                self.dynamic_elecation = self.elevation
                if self.pressed == True:
                    function()
                    self.pressed = False
        else:
            self.dynamic_elecation = self.elevation
            self.top_color = self.color_off