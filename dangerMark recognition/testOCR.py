import cv2
from pytesseract import image_to_string
img = cv2.imread('test2.png',0)

method = [cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV]
blurVelocity = [(4,2),(5,2)]
rawStr = ''

for m in range(2):
    maskThresh = cv2.threshold(img.copy(),120,255,method[m])[1]
    for n in range(2):
        maskBlur = cv2.blur(maskThresh.copy(), blurVelocity[n])
        rawStr += str(image_to_string(maskBlur))

forbiddenSymbols = ['\\',' ',',','!','@','#','$','%','^','&','*','(',')','-', \
    '_','_','+','=',';','/','?',"'",'~','`','"',':','№','8','\n','\r']
for m in range(len(forbiddenSymbols)):
    rawStr = rawStr.replace(forbiddenSymbols[m],'')
finalStr = rawStr.lower()

print(finalStr)
cv2.waitKey(1)
