import pygame
import sys
from pygame.locals import *
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
def main():
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f'Button {event.button} pressed')
            elif event.type == pygame.JOYHATMOTION:
                print(f'Hat {event.hat} moved')
            pygame.time.wait(10)
            joystick_axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            joystick_buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            print(f'Axes: {joystick_axes} Buttons: {joystick_buttons}')
if __name__ == '__main__':
    main()