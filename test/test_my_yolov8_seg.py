import cv2
from PIL import Image

from ultralytics import YOLO

model = YOLO("/home/xiaoyang/Downloads/yolov8m-seg.pt")
# import pbd;pdb.set_trace()
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
# results = model.predict(source="0")
# results = model.predict(source="folder", show=True)  # Display preds. Accepts all YOLO predict arguments

# put model and image to gpu
# model.to("cuda")


# # from PIL
# im1 = Image.open("./bus.jpg")
# results = model.predict(source=im1, save=True)  # save plotted images

# from ndarray
im2 = cv2.imread("./bus.jpg")
print("before predict")
results = model.predict(source=im2, save=True, save_txt=True, stream=True)  # save predictions as labels
print("after predict")
# from list of PIL/ndarray
# results = model.predict(source=[im1, im2])


# Visualize the results
for i, r in enumerate(results):
    import pdb;pdb.set_trace()
    # Plot results image
    im_bgr = r.plot()  # BGR-order numpy array
    im_rgb = Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

    # Show results to screen (in supported environments)
    r.show()

    # Save results to disk
    r.save(filename=f"results{i}.jpg")
