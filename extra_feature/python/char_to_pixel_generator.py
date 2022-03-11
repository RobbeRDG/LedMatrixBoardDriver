from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import numpy as np

def char_to_pixels(char, fontsize , height):
    #Load the font
    font = ImageFont.truetype('arial.ttf', fontsize)

    #calc the size of text in pixels
    w,h = font.getsize(char)

    #The hight should be constant
    h = height

    #create a blank b/w image
    image = Image.new('1', (w,height), 0)

    #Draw the image
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), char, 1, font) 

    #print(str(w) + ", " + str(h))
    #image.show()

    #Generate a matrix from the image
    arr = np.asarray(image)

    #Save the image matrix in a text file
    imageName = char + "_" + str(height) + '.txt'
    np.savetxt(imageName, arr, '%i', "", ",")
    

def main():
    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ\'/0123456789!. "

    for letter in alphabet:
        char_to_pixels(letter, 16, 16)


if __name__ == "__main__":
    main()
