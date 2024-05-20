import cv2

image = cv2.imread("Binary_Mask.png")
cv2.imwrite("Image.png",image)
image1 = cv2.imread("Image.png")
shape = image.shape[:2]
l = eval(input("Point: "))
x = int(shape[0] - int(l[0])/10*shape[0])
y = int(shape[1] - int(l[1])/10*shape[1])
cv2.circle(image1,(x,y), 2, (0,255,0), -1)
cv2.imshow("Nodes", image1)
cv2.waitKey(1000)
l = eval(input("Point: "))