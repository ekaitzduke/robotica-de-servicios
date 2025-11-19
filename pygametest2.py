
# Packages to run pygames
import pygame
# from pygame.locals import *

import os  # Used just to check path existance when reading images for the initial static display
import argparse # Argument program treatment
import cv2 # Package for image collection and processing
# import gather_samples as gs # Custom external program to gather the images instead of just loading existing ones

# Packages needed for gesture recognition
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from math import acos, degrees # Used for thumb angle calculation 
import gesture_control as gc   # Some functions to get finger, palm and centroid coordinates given landmarks
import numpy as np             # To use arrays and linalg.form on finger calculation


SCREENSIZE = (1000,600)  # Screen size of the game

GAMEFPS = 40 # Game speed on frame per second
TIMER = 2 # Delay (in s) before trying to do a match in normal mode (it will still run on zoomed mode)
DELAYFRACTION = 0.5 # Fraction of the timer expected to pass to go from state 2 to 3 in a display


# Live camera positions and dimensions when normal and zoomed (Zoomed means updating gestures matchs)
CAMARAINITIALPOS = (750,450)
CAMARAINITIALDIM = (220,130)
CAMARAZOOMEDPOS = (0,0)
CAMARAZOOMEDDIM = (600,300)


# Positions and dimensions of the static displays of the gesture to match with the camera
DISPLAYINITIALPOS = (750,20)
DISPLAYINITIALDIM = (100,50)

DISPLAYDISTRIB = (2,4) # Distribution of the displays (Rows and columns are inverted for pygame)
DISPLAYOFFSET = (20,50) # Space between screens of the display
FILLERCOLOR = (255,255,255) # Color used to fill screen when no image is given
FONTCOLOR = (255, 255, 0) # Color of the font used below the screens

# Rectangle outlier colors for the state of the screens (1 -> Selected to change, 2 -> Matched, 3 -> Half the timer has passed since match)
DISPLAYSELCOLOR = [(0,109,180),(0,191,0),[255,221,17]]

# Color of the info text
COLORINFOTEXT = (255, 255, 0)

# Letters used to catch events (can be triggered with the mouse as well)
DISPLAYUPDATEKEY = pygame.K_s # Change gesture with the live camera one of the selected screen of the display o zoomed mode
CAMARACHANGEKEY = pygame.K_z # Change between normal and zoomed live camera mode
DISPLAYSELECTKEYS = [pygame.K_1,pygame.K_2,pygame.K_3,pygame.K_4,pygame.K_5,pygame.K_6,pygame.K_7,pygame.K_8] # Select a screen on zoomed mode
SHUTDOWNKEY = pygame.K_q # Quit the game

DIRTEXT = ["Up", "Right", "Down", "Left"] # Orders to send when the corresponding screen match

# Labels used by the classifier
MPGESID = ["None", "Pointing_Up", "Thumb_Up", "Closed_Fist", "Thumb_Down", "Open_Palm", "Victory", "ILoveYou"]

# Shortened labels so the font below the screen stays the same and reduced length
GESTEXT = ["Fail", "None", "Pont", "ThUp", "Fist", "ThDw", "Palm", "Vict", "Love"]

# Finger Index References
THUMB_POINTS = [1,2,4]
PALM_POINTS = [0,1,2,5,9,13,17]
FINGERTIPS_POINTS = [8,12,16,20]
FINGERBASE_POINTS = [6,10,14,18]


# Live camera screen class (Also used for the individual screens of the display) located on position (pos) with a certain dimension (dim)
# (Dimensions are assumed to be always a list of 2 positive integers, there is no further checks) (Positions are also assumed as 2 integers, with a check on sign)

# Surface pygame object used for display is stored on handle (Uses default solid color FILLERCOLOR when there is no/can't load image)
class ScreenDisplay():
    def __init__(self,pos,dim):
        self.pos = pos
        self.adapt(dim)
        self.handle.fill(FILLERCOLOR)

    # Draw the image (or the filler rectangle) on the pygame screen (screen) only if it fits completely
    # (No checks are done on the screen argument format)
    def draw(self,screen):
        if (min(self.pos) >= 0) and (self.pos[0] + self.dim[0] <= screen.get_size()[0]) and (self.pos[1] + self.dim[1] <= screen.get_size()[1]):
            screen.blit(self.handle, self.pos)

    # Update the image data (with image) used on the draw function (image assumed to be given on BGR numpy array format)
    # All the image must be confined on the expected screen size (screenDim) to work. A filler rectangle would be used if image is not given
    # (No checks are done on screenDim or image argument format)
    def update(self,image,screenDim):
        if (min(self.pos) >= 0) and (self.pos[0] + self.dim[0] <= screenDim[0]) and (self.pos[1] + self.dim[1] <= screenDim[1]):
            if image is not None:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image = cv2.resize(image,self.dim)
                image = cv2.flip(image,1)
                i = 0
                for x in image:
                    j = 0
                    for y in x:
                        self.handle.set_at((j,i),y)
                        j += 1
                    i += 1
            else:
                self.handle.fill(FILLERCOLOR)

    # Retrieve as a numpy narray (with opencv image format) the image on the screen
    # Also applies a resize using size values (No check for those)
    def retrieve(self,size=DISPLAYINITIALDIM):
        ivalue = []

        for i in range(self.dim[1]):
            ivalue.append([])
            for j in range(self.dim[0]):
                ivalue[i].append(self.handle.get_at((j,i)))

        image = np.array(ivalue)
        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        image = cv2.flip(image,1)
        image = cv2.resize(image,size)

        return image

    # Adapt screen to a resize (dim) and a reposition (pos) (reposition conditional to x and y values being positive)
    # Also recreates the surface of the new dimension
    def adapt(self,dim,pos=(-1,-1)):
        if min(pos) >= 0:
            self.pos = pos
        self.dim = dim
        self.handle = pygame.Surface(dim)

    # Return (boolean) if the mouse position (mousepos) is on the image
    def pressed(self,mousepos):
        if mousepos[1] >= self.pos[1] and mousepos[1] <= self.pos[1]+self.dim[1] and mousepos[0] >= self.pos[0] and mousepos[0] <= self.pos[0]+self.dim[0]:
                return True
        return False



# Display screens rectangular formation class

# Formation starts on a position (pos) with each screen having a certain dimension (dim) and with an offset between them (offset)
# Also a formation distribution is needed (size, rows and cols reversed. Only used on construction, screen list is flattened)
# The space occupied by this display is stored as total dimensions (tdim)

# data1 and data2 initialiation parameters can either be any pair of dim, offset and tdim
# datatype value is used to select which pair is used (<= 0 -> dim - offset / == 1 -> dim - tdim / else -> tdim - offset)
# The leftover parameter is calculated and stored on initialization for each case 
# (beware rounding leftover with int method on not <= 0 cases)

# An outlier rectangle with a specific color (set by colors) with a certain width (selwidth) would be draw using states values (states)
# Current value of the object screen sets the color (state  <= 0 will cause no outlier to be displayed)
# If a color list element is missing for given a state value, no outlier will be displayed
# (state list is always set as a vector of length equal to the number of screens on initialization, all 0 by default)

# A font text down the image will also be drawn with the text info (text) TODO: Needs more refining

# (pos, data1, data2 and size are assumed to be always a list of 2 positive integers, there is no further checks)
# (selwidth expected as 1 integer > 0, no checks) (colors are expected to be RGB formatted) 
# (text expected to be given on a list of 2 String lists)
class ScreenDisplayPanel():
    def __init__(self,pos,data1,data2,size,colors,selwidth=4,states=[],text = [], datatype = 0):
        self.pos = pos
        self.colors = colors
        self.selwidth = selwidth

        # Calculate dimensions for the individual screens (dim), offsets between them (offsets) and 
        # total dimensions of the display (tdim)
        self.dim = data1
        if datatype <= 0:
            self.tdim = [0,0]
            offsets = data2
            for k in range(2):
                if size[k] > 1:
                    self.tdim[k] = size[k]*(offsets[k]+self.dim[k])-offsets[k]
        elif datatype == 1:
            self.tdim = data2
            offsets = [0,0]
            for k in range(2):
                if size[k] > 1:
                    offsets[k] = int((self.tdim[k]-self.dim[k]*size[k])/(size[k]-1))
        else:
            self.tdim = data1
            offsets = data2
            self.dim = [0,0]
            for k in range(2):
                if size[k] > 1:
                    self.dim[k] = int((self.tdim[k]-offsets[k]*(size[k]-1))/(size[k]))

        # Here each screen object will be saved as a list 
        # (Order from left to right, then top to bottom once the current row is exhausted)
        self.displays = []

        # Fixed screen surface used for each screen instance as base
        self.selrect = pygame.Surface((self.dim[0]+self.selwidth*2,self.dim[1]+self.selwidth*2))

        # Initialize each screen object
        for i in range(size[1]):
            for j in range(size[0]):
                screenpos = [pos[0] + (self.dim[0]+ offsets[0])*j, pos[1] + (self.dim[1]+ offsets[1])*i]
                self.displays.append(ScreenDisplay(screenpos,self.dim))

        # Set states values (would be set to 0 if no data is given for the expected screen)
        if not states:
            self.states = []
            for i in range(size[0]*size[1]):
                self.states.append(0)
        else:
            self.states = list(states[0:size[0]*size[1]])
            for i in range(size[0]*size[1]-len(states)):
                self.states.append(0)
        
        # Set the text contents
        if not text:
            self.text = []
            for i in range(size[0]*size[1]):
                self.text.append([])
                self.text[i].append(GESTEXT[0])
                if i < len(DIRTEXT):
                    self.text[i].append(DIRTEXT[i])
                else:
                    self.text[i].append("")
        else:
            self.text = list(text[0:size[0]*size[1]])
            for i in range(size[0]*size[1]-len(text)):
                self.text.append([])
                self.text[i+len(text)].append(GESTEXT[0])
                self.text[i+len(text)].append("")

    # Draw all screens on a pygame screen (screen) with a text below and an outlier rectangle if it is requested by states value (0 -> No)
    def draw(self,screen):
        if (min(self.pos) >= 0) and (self.pos[0] + self.dim[0] <= screen.get_size()[0]) and (self.pos[1] + self.dim[1] <= screen.get_size()[1]):
            for i in range(len(self.displays)):
                if self.states[i] > 0 and len(self.colors) >= self.states[i]:
                    self.selrect.fill(self.colors[self.states[i]-1])
                    screen.blit(self.selrect,(self.displays[i].pos[0]-self.selwidth,self.displays[i].pos[1]-self.selwidth))
                self.displays[i].draw(screen)

                font = pygame.font.Font(None, 25)
                disptext = font.render(f'{self.text[i][0]}, {self.text[i][1]}', True, FONTCOLOR)
                screen.blit(disptext, [self.displays[i].pos[0]+10,self.displays[i].pos[1]+self.displays[i].dim[1]+10])

    # Same as update method of class ScreenDisplay, but aimed for an specific screen (screenNum, expected as integer)
    def update(self,image,screenDim,screenNum=0):
        if (min(self.pos) >= 0) and (self.pos[0] + self.dim[0] <= screenDim[0]) and (self.pos[1] + self.dim[1] <= screenDim[1]):
            screenNum = max(0, screenNum)
            screenNum = min(len(self.displays)-1, screenNum)
            self.displays[screenNum].update(image,screenDim)

    # Same as pressed method of class ScreenDisplay, but also changes the state of the pressed screen to statevalue 
    # (Erases all the others values. If no screen is pressed, all values would be erased)
    # TODO: Add an initial check to assure mousepos is even on the display. 
    # TODO: May consider not a lineal search of a selected screen. Use any sort algorithm
    def updatepressed(self,mousepos,statevalue=1):
        changed = False
        for i in range(len(self.displays)):
            self.states[i] = 0
            if (not changed) and self.displays[i].pressed(mousepos):
                changed = True
                self.states[i] = statevalue

        return changed

    # Update method adapted to only the text content (First element of the two only)
    def updatetext(self,text,screenNum=0):
        screenNum = max(0, screenNum)
        screenNum = min(len(self.displays)-1, screenNum)
        self.text[screenNum][0] = text
    
    # Erase all the state values (set to 0) and set the screen value selected (by screenNum) to statevalue
    # (If screenNum is negative, all values would be erased)
    def updatestates(self,screenNum=-1,statevalue=1):
        for i in range(len(self.states)):
            self.states[i] = 0

        if screenNum >= 0:
            screenNum = min(screenNum,len(self.states))
            self.states[screenNum] = statevalue

# Get gesture string from an image (expected as selfie) with a gesture model (recognizer) of mediapipe
# (Uses MACRO values MPGESID and GESTEXT to rename the gesture. Search them for more info)
def getgesture(image,recognizer):
    if image is not None:
        image = cv2.flip(image, 1)
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        recognition_result = recognizer.recognize(mp_image)
        if len(recognition_result.gestures) > 0:
            for i in range(len(MPGESID)):
                if MPGESID[i] == recognition_result.gestures[0][0].category_name:
                    return GESTEXT[i+1]
    return GESTEXT[0]


# Get finger string (decimal value as string of the binary array. A lifted finger is set as 1 and the order starts from thumb to pinky)
# Uses landmarks from a mediapipe hand detection (mp) and screen dimensions (screendim) to get coordinates for palm, thumb, tips and base fingers
def getfinger(screendim,lm):

    # Get coordinates
    coordinates_thumb = gc.getCoordinates(screendim[0], screendim[1], THUMB_POINTS, lm)
    coordinates_palm = gc.getCoordinates(screendim[0], screendim[1], PALM_POINTS, lm)
    coordinates_ft = gc.getCoordinates(screendim[0], screendim[1], FINGERTIPS_POINTS, lm)
    coordinates_fb = gc.getCoordinates(screendim[0], screendim[1], FINGERBASE_POINTS, lm)

    # THUMB
    p1, p2, p3, l1, l2, l3 = gc.getTriangle(coordinates_thumb)
    cos_angle = (l1**2 + l3**2 - l2**2) / (2 * l1 * l3)
    cos_angle = max(-1, min(1, cos_angle))  # clamp para evitar errores
    thumb_angle = degrees(acos(cos_angle))
    thumb_extened = thumb_angle > 160
                    
    # OTHER FINGERS
    nx, ny = gc.palmCentroid (coordinates_palm)
    coordinates_centroid = np.array([nx, ny])
    coordinates_fb = np.array(coordinates_fb)
    coordinates_ft = np.array(coordinates_ft)

    d_centrid_ft = np.linalg.norm(coordinates_centroid - coordinates_ft, axis=1)
    d_centrid_fb = np.linalg.norm(coordinates_centroid - coordinates_fb, axis=1)
    dif = d_centrid_ft - d_centrid_fb
    fingers = dif > 0
    fingers = np.append(thumb_extened, fingers)

    return fingers


def getFingerstrfromImage(image,hands):

    if image is not None:
        image = cv2.flip(image, 1)
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        results = hands.process(image)
        result_finger = []

        if results.multi_hand_landmarks:
            for lm in results.multi_hand_landmarks:
                fingers = getfinger(SCREENSIZE,lm)
                id = 0
                for j in range(len(fingers)):
                    if fingers[j]:
                        id += 2**j
                result_finger.append(str(id))
        else:
            result_finger.append(GESTEXT[0])

    else:
        result_finger = [GESTEXT[0]]

    return result_finger[0]



def testgame(displaypath,imgformat,cameraport,gathersamples,images,usefinger,debug,recognizer):

    # Timer (using frames as unit) to wait to start a recognition attempt (Does one, then resets)
    timer = TIMER*GAMEFPS

    running = True # Flag to shutdown pygame once it's running

    # Capture the video and read the first frame
    if not debug:
        video = cv2.VideoCapture(cameraport)

    # Initialize the hand detector of mediapipe
    hands = mp.solutions.hands.Hands(
        static_image_mode = True, 
        max_num_hands = 1,
        min_detection_confidence = 0.8,
        min_tracking_confidence = 0.8,
        model_complexity = 1
    )

    # Initialize pygame modules and get the clock to set fps
    pygame.init()
    reloj = pygame.time.Clock()

    # Set screen size and window title
    screen = pygame.display.set_mode(SCREENSIZE)
    pygame.display.set_caption("Camera Tests")

    # Prepare info text for the game
    fuente = pygame.font.Font(None, 25)
    infotexts = [fuente.render("To modify possible gestures to match use keybind z or click on the live feed", True, COLORINFOTEXT),\
                 fuente.render("Same keybind to return to the main app (can take some time or fail, so retry it)", True, COLORINFOTEXT),
                 fuente.render("While changing gestures, use keybinds 1-n (n) or click on the screen to select it", True, COLORINFOTEXT), \
                 fuente.render("Once selected, press s to save current live feed as gesture (see text changing)", True, COLORINFOTEXT), \
                 fuente.render("Quit the app with q or close the window", True, COLORINFOTEXT)]


    # Create live camera and static display panel
    camara = ScreenDisplay(CAMARAINITIALPOS,CAMARAINITIALDIM)
    camaraPanel = ScreenDisplayPanel(DISPLAYINITIALPOS,DISPLAYINITIALDIM,DISPLAYOFFSET,DISPLAYDISTRIB,
                                     DISPLAYSELCOLOR,text=(["Fail",DIRTEXT[0]],["Fail",DIRTEXT[1]],["Fail",DIRTEXT[2]],["Fail",DIRTEXT[3]]))

    # Update display with stored images
    for i in range(DISPLAYDISTRIB[0]*DISPLAYDISTRIB[1]):

        # Search the image as file (or used the gathered one) and load on a screen of the panel
        if not gathersamples:
            if os.path.exists(f'{displaypath}/{i}.{imgformat}'):
                image2 = cv2.imread(f'{displaypath}/{i}.{imgformat}')
            else:
                image2 = None
        elif len(images) > i:
            image2 = images[i]
        else:
            image2 = None
        camaraPanel.update(image2,SCREENSIZE,i)

        # Assign a gesture or finger state to the image
        if usefinger:
            camaraPanel.updatetext(getFingerstrfromImage(image2,hands),i)
        else:
            camaraPanel.updatetext(getgesture(image2,recognizer),i)

    # To make app working without live feed
    if debug:
        success = True
        image = None

    # Pygame flow
    while running:

        if not debug:
            success,image = video.read()

        # Event processing
        for event in pygame.event.get():

            # Quitting
            if event.type == pygame.QUIT or not success:
                running = False

            # All mouse events
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()

                # Camera on zoomed mode (Ready to update static display by selecting screens with mouse)
                # (Can return to normal mode by pressing live camera)
                if camara.dim[0] == CAMARAZOOMEDDIM[0] and camara.dim[1] == CAMARAZOOMEDDIM[1]:
                    if camara.pressed(pos):
                        camaraPanel.updatestates()
                        camara.adapt(CAMARAINITIALDIM,CAMARAINITIALPOS)
                    else:
                        camaraPanel.updatepressed(pos)
                # Camera on normal mode (Ready to pass to zoomed mode once live camera pressed)
                else:
                    if camara.pressed(pos):
                        camaraPanel.updatestates()
                        camara.adapt(CAMARAZOOMEDDIM,CAMARAZOOMEDPOS)

            # All keybinds events
            elif event.type == pygame.KEYDOWN:
                key = pygame.key.get_pressed()

                # Quit the game by a keypress
                if key[SHUTDOWNKEY]:
                    running = False

                elif camara.dim[0] == CAMARAZOOMEDDIM[0] and camara.dim[1] == CAMARAZOOMEDDIM[1]:

                    for i in range(len(DISPLAYSELECTKEYS)):
                        if key[DISPLAYSELECTKEYS[i]]:
                            camaraPanel.updatestates(i)
                            break
                
                    if key[DISPLAYUPDATEKEY]:
                        for i in range(len(camaraPanel.states)):
                            if camaraPanel.states[i] == 1:
                                camaraPanel.update(image,SCREENSIZE,i)

                                # Use finger detection or recognizer detection
                                if usefinger:
                                    camaraPanel.updatetext(getFingerstrfromImage(image,hands),i)
                                else:
                                    camaraPanel.updatetext(getgesture(image,recognizer),i)

                    
                    elif key[CAMARACHANGEKEY]:
                        camaraPanel.updatestates()
                        camara.adapt(CAMARAINITIALDIM,CAMARAINITIALPOS)

                elif key[CAMARACHANGEKEY]:
                    camaraPanel.updatestates()
                    camara.adapt(CAMARAZOOMEDDIM,CAMARAZOOMEDPOS)

        screen.fill((0,0,0))    # Clean the screen

        camara.update(image,SCREENSIZE) # Update live camera

        # Draw cameras on screen
        camara.draw(screen)
        camaraPanel.draw(screen)

        # Print info for the user to know keybinds and controls
        for i in range(len(infotexts)):
            screen.blit(infotexts[i], [5, SCREENSIZE[1] - 30*(5-i)])

        # Refresh screen and game
        pygame.display.flip()

        # Internal timer to avoid gesture/finger recognition each frame
        if timer > 0:
            timer -= 1

        # Enact the detection once timer runs out (and not on debug mode)
        elif camara.dim[0] == CAMARAINITIALDIM[0] and camara.dim[1] == CAMARAINITIALDIM[1] and not debug:

            if usefinger:
                curr_gesture = getFingerstrfromImage(image,hands)

            else:
                curr_gesture = getgesture(image,recognizer)

            for j in range(len(camaraPanel.displays)):
                if curr_gesture == camaraPanel.text[j][0]:
                    camaraPanel.updatestates(j,2)
                    # print(f'Gesture matched with action: {camaraPanel.text[j][1]}')
                    break

            timer = TIMER*GAMEFPS

        # Blink the selected gesture (After a fraction of the timer has passed)
        if timer < TIMER*GAMEFPS*DELAYFRACTION:
            for k in range(len(camaraPanel.displays)):
                if camaraPanel.states[k] == 2:
                    camaraPanel.states[k] += 1
                
        reloj.tick(GAMEFPS)

    # Uninitialize pygame modules
    if not debug:
        video.release()
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Program to take frames from a camera in real-time.',formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument('--gathersamples','-gs', action = 'store_true', help = 'Flag to start gathering samples from camera')
    parser.add_argument('--outputpath', '-out', type=str, default = 'Imagenes', help = 'Path to the folder to store frames')
    parser.add_argument('--saveframes', '-sfr', action = 'store_true', help = 'Save the frame on the outputpath')
    parser.add_argument('--usefinger', '-ug', action = 'store_false', help = 'Use finger detection instead of gesture')
    parser.add_argument('--cameraport', '-cp', type=int, default = 0, help = 'Camera port used')
    parser.add_argument('--saveformat', '-sf', type=str, default = 'jpg', help = 'Saved frames format')
    parser.add_argument('--maxsamples', '-maxs', type=int, default = 8, help = 'Max samples to gather (<= 0 will not put a limit)')

    parser.add_argument('--verbose','-v', action = 'store_true', help = 'Show information on terminal')
    parser.add_argument('--debug','-d', action = 'store_true', help = 'Debug mode (deactivate live feed)')


    args = parser.parse_args()

    # Initialize the recognizer for the gestures
    base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
    options = vision.GestureRecognizerOptions(base_options=base_options)
    recognizer = vision.GestureRecognizer.create_from_options(options)


    images = None
    # if args.gathersamples:
    #     images = gs.takesamples(args.outputpath,args.saveframes,args.saveformat,args.cameraport,args.maxsamples,args.verbose+1,recognizer)
    
    testgame(args.outputpath,args.saveformat,args.cameraport,args.gathersamples,images,args.usefinger,args.debug,recognizer)
