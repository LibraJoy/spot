from bosdyn.client.image import build_image_request, ImageClient, image_pb2
import bosdyn.client.spot_cam as spot_cam
import bosdyn.client.util
from spot_ros2.CameraService import CameraService
import cv2
import numpy as np
import time

bosdyn.client.util.setup_logging(False)
sdk = bosdyn.client.create_standard_sdk('CerlabSpotSDK')

spot_cam.register_all_service_clients(sdk)

robot = sdk.create_robot('10.0.0.3')
robot.authenticate('user', 'scgau6g5w987')

ptzCam = CameraService(robot)
print(robot.list_services())
cam_image_client = robot.ensure_client("spot-cam-image")
# list client > readme

sources = cam_image_client.list_image_sources()
print("Available sources:", [s.name for s in sources])


# ------------------------SAVE IMAGES-----------------------------

# # Optimized requests: lower quality & resize for speed
# # Only use 4 cameras (skip c4 as requested)
# request = [
#     build_image_request(
#         f'{s.name}',
#         quality_percent=100,    # Lower quality = faster
#         resize_ratio=1.0,       # Half resolution = 4x less data
#         image_format=1         # JPEG format for faster decoding

#     ) for s in sources # c0, c1, c2, c3
# ]

# print("\nOptimized settings: 100% quality, 100% resolution, 4 cameras + pano_full, JPEG format")
# # print("Starting video capture (Press 'q' to quit)...\n")

# image_resposes = cam_image_client.get_image(request)
# for i, response in enumerate(image_resposes):
#         # start_time = time.time()
#         img = response.shot.image
#         # Convert to cv2 image
#         image_data = np.frombuffer(img.data, dtype=np.uint8)
#         cv_image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

#         cv2.imwrite(f"{sources[i].name}.jpg", cv_image)

# ----------------------------------------------SAVE IMAGES-----------------------------

# ----------------------------STREAM IMAGES-----------------------------

def show_multi(quality_percent=100, resize_ratio=0.30, image_format=2):
    """Show 4 cameras (c0-c3) in parallel"""
    request = [
        build_image_request(
            f'c{i}',
            quality_percent=quality_percent,
            resize_ratio=resize_ratio,
            image_format=image_format
        ) for i in range(4)  # c0, c1, c2, c3
    ]

    # Create windows for 4 cameras
    for i in range(4):
        cv2.namedWindow(f'Camera c{i}', cv2.WINDOW_NORMAL)
        cv2.resizeWindow(f'Camera c{i}', 640, 480)

    frame_count = 0
    start_time = time.time()

    # First frame info
    print("\nCapturing first frame to check format...")
    first_response = cam_image_client.get_image(request)
    first_img = first_response[0].shot.image
    print(f"Image format: {first_img.format} (1=JPEG, 2=RAW)")
    print(f"Pixel format: {first_img.pixel_format} (1=GREY, 3=RGB, 4=RGBA)")
    print(f"Resolution: {first_img.cols}x{first_img.rows}")
    print(f"Data size: {len(first_img.data):,} bytes\n")
    print("Press 'q' to quit\n")

    while True:
        image_resposes = cam_image_client.get_image(request)

        # Process all 4 cameras
        for i, response in enumerate(image_resposes):
            img = response.shot.image

            # Convert to cv2 (pixel format is always 3 = RGB)
            if img.format == 2:  # RAW
                cv_image = cv2.cvtColor(
                    np.frombuffer(img.data, dtype=np.uint8).reshape((img.rows, img.cols, 3)),
                    cv2.COLOR_RGB2BGR
                )
            else:  # JPEG
                cv_image = cv2.imdecode(np.frombuffer(img.data, dtype=np.uint8), cv2.IMREAD_COLOR)

            # Show each camera in its own window
            cv2.imshow(f'Camera c{i}', cv_image)

        frame_count += 1

        # Print FPS every 20 frames
        if frame_count % 20 == 0:
            print(f"Frame {frame_count}: {frame_count/(time.time()-start_time):.2f} FPS")

        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

    # Print stats
    print(f"\nTotal frames: {frame_count}")
    print(f"Total time: {time.time()-start_time:.2f}s")
    print(f"Average FPS: {frame_count/(time.time()-start_time):.2f} Hz")

    return image_resposes


def show_single(source='pano', quality_percent=100, resize_ratio=0.30, image_format=2, pixel_format=2):
    """Show single camera source (e.g., 'pano', 'c0', etc.)"""
    request = [
        build_image_request(
            source,
            quality_percent=quality_percent,
            resize_ratio=resize_ratio,
            image_format=image_format,
            pixel_format=pixel_format
        )
    ]

    # Create window
    cv2.namedWindow(source, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(source, 2400, 400)

    frame_count = 0
    start_time = time.time()

    # First frame info
    print(f"\nCapturing {source}...")
    first_response = cam_image_client.get_image(request)
    first_img = first_response[0].shot.image
    print(f"Image format: {first_img.format} (1=JPEG, 2=RAW)")
    print(f"Pixel format: {first_img.pixel_format}")
    print(f"Resolution: {first_img.cols}x{first_img.rows}")
    print(f"Data size: {len(first_img.data):,} bytes\n")
    print("Press 'q' to quit\n")

    while True:
        image_resposes = cam_image_client.get_image(request)
        img = image_resposes[0].shot.image

        # Convert to cv2 (pixel format is always 3 = RGB)
        if img.format == 2:  # RAW
            cv_image = cv2.cvtColor(
                np.frombuffer(img.data, dtype=np.uint8).reshape((img.rows, img.cols, 3)),
                cv2.COLOR_RGB2BGR
            )
        else:  # JPEG
            cv_image = cv2.imdecode(np.frombuffer(img.data, dtype=np.uint8), cv2.IMREAD_COLOR)

        # Show image
        cv2.imshow(source, cv_image)

        frame_count += 1

        # Print FPS every 20 frames
        if frame_count % 30 == 0:
            print(f"Frame {frame_count}: {frame_count/(time.time()-start_time):.2f} FPS")
        if frame_count == 300:
            break

        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

    # Print stats
    print(f"\nTotal frames: {frame_count}")
    print(f"Total time: {time.time()-start_time:.2f}s")
    print(f"Average FPS: {frame_count/(time.time()-start_time):.2f} Hz")

    return image_resposes


# Main execution
if __name__ == '__main__':
    # Uncomment the function you want to run:

    # Show 4 cameras in parallel
    # lowering quality percent will increase FPS only fo JPEG format, not RAW
    # pixel format: image_pb2.Image.PixelFormat https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html#image-pixelformat
    # pixel format 1 = GREY is not supported by the spot cam, it will return an error. 
    # image_resposes = show_single(quality_percent=50, resize_ratio=0.30, image_format=2, pixel_format=3)

    # Show single pano camera
    image_resposes = show_single('c1', quality_percent=60, resize_ratio=0.25, image_format=1, pixel_format=3)

    # Print last response metadata (excluding image data)
    if 'image_resposes' in locals() and len(image_resposes) > 0:
        print("\n" + "=" * 70)
        print("LAST IMAGE RESPONSE METADATA:")
        print("=" * 70)

        last_response = image_resposes[0]
        print(last_response.source.pinhole_brown_conrady.ListFields())  # Print all fields for debugging
        
        for items in last_response.ListFields():
            field_name = items[0].name
            field_value = items[1]

            print(f"\n{field_name}:")

            # Check if this is a message object (has ListFields) or primitive type
            if hasattr(field_value, 'ListFields'):
                # It's a message - iterate through its fields
                for sub_items in field_value.ListFields():
                    sub_name = sub_items[0].name
                    sub_value = sub_items[1]

                    # Skip image data (too large)
                    if sub_name == "image":
                        for img_field in sub_value.ListFields():
                            img_field_name = img_field[0].name
                            img_field_value = img_field[1]
                            if img_field_name != "data":
                                print(f"  {img_field_name}: {img_field_value}")
                            else:
                                print(f"  {img_field_name}: <{len(img_field_value)} bytes>")
                    else:
                        print(f"  {sub_name}: {sub_value}")
            else:
                # It's a primitive type (int, string, etc.)
                print(f"  {field_value}")


# ----------------------------------------------STREAM IMAGES-----------------------------