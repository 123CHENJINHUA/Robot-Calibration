import pygame
import time

class XboxController:
    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Initialize the joystick
        pygame.joystick.init()

        # Check for joystick
        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick connected")
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick name: {self.joystick.get_name()}")

        self.dir_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        self.drill = 0

    def read_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.JOYAXISMOTION:
                # Read axis (analog stick) values
                # left_stick_x = 0 if abs(self.joystick.get_axis(0)) < 0.05 else self.joystick.get_axis(0) * -1.5
                # left_stick_y = 0 if abs(self.joystick.get_axis(1)) < 0.05 else self.joystick.get_axis(1) * -1.5
                # right_stick_x = 0 if abs(self.joystick.get_axis(3)) < 0.3 else self.joystick.get_axis(3) * -0.1
                # right_stick_y = 0 if abs(self.joystick.get_axis(4)) < 0.3 else self.joystick.get_axis(4) * -0.1
                # up_stick_L = 0 if abs(self.joystick.get_axis(2)) < 0.05 else (self.joystick.get_axis(2) + 1)/8
                # up_stick_R = 0 if abs(self.joystick.get_axis(5)) < 0.05 else (self.joystick.get_axis(5) + 1)/8
                # deep = -up_stick_L + up_stick_R

                #bluetooth
                left_stick_x = 0 if abs(self.joystick.get_axis(0)) < 0.05 else self.joystick.get_axis(0) * -1.5
                left_stick_y = 0 if abs(self.joystick.get_axis(1)) < 0.05 else self.joystick.get_axis(1) * -1.5
                right_stick_x = 0 if abs(self.joystick.get_axis(2)) < 0.3 else self.joystick.get_axis(2) * -0.1
                right_stick_y = 0 if abs(self.joystick.get_axis(3)) < 0.3 else self.joystick.get_axis(3) * -0.1
                up_stick_L = 0 if abs(self.joystick.get_axis(4)) < 0.05 else (self.joystick.get_axis(4) + 1)/8
                up_stick_R = 0 if abs(self.joystick.get_axis(5)) < 0.05 else (self.joystick.get_axis(5) + 1)/8
                deep = -up_stick_L + up_stick_R

                #print(f"Left Stick: ({left_stick_x}, {left_stick_y}), Right Stick: ({right_stick_x}, {right_stick_y})")
                self.dir_ = [left_stick_x, deep, left_stick_y, right_stick_y, 0, right_stick_x]
            elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                # Read button states
                button_states = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                if button_states[2] == 1:
                    self.drill = 1
                elif button_states[1] == 1:
                    self.drill = 0
                #print(f"Button states: {button_states}")
        return True
    
    def getmotion(self):
        self.read_events()
        #print(self.dir_)
        return self.dir_
    
    def getdrill(self):
        self.read_events()
        #print(self.drill)
        return self.drill

    def close(self):
        pygame.quit()

if __name__ == "__main__":
    controller = XboxController()
    running = True
    while running:
        controller.getmotion()
        controller.getdrill()
        time.sleep(0.1)
    controller.close()