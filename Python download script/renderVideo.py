import os
import cv2
from tqdm import tqdm

def checkImageComplete(path):
    with open(path, 'rb') as f:
        check_chars = f.read()[-2:]
    return check_chars == b'\xff\xd9'
    # try:
    #     _ = io.imread(path)
    # except Exception as e:
    #     print(e)
    #     return False
    # return True

imagePath = "images"

images = os.listdir(imagePath)
sampleFrame = cv2.imread(os.path.join(imagePath,images[0]))
height, width, layers = sampleFrame.shape

##Sort images?

video = cv2.VideoWriter("WIPTimelapse.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 18, (width,height))
print("Rendering video...")
for img in tqdm(images):
    try:
        path = os.path.join(imagePath, img)

        frame = cv2.imread(path)
        # add text to frame
        text = img[:-4]
        textPosition = (width - len(text)*25, height - int(height * 0.03))
        frame = cv2.putText(frame, text, textPosition, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 5, cv2.LINE_AA)
        frame = cv2.putText(frame, text, textPosition, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        #imS = cv2.resize(frame, (960, 540))  # Resize image
        # cv2.imshow("output", imS)
        # cv2.waitKey(0)
        video.write(frame)
        
    except Exception as e:
        pass

video.release()