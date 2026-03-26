from bosdyn.client.image import build_image_request, ImageClient, image_pb2
import bosdyn.client.spot_cam as spot_cam
import bosdyn.client.util
import cv2
import numpy as np
import time
import os

bosdyn.client.util.setup_logging(False)
sdk = bosdyn.client.create_standard_sdk('CerlabSpotSDK')

spot_cam.register_all_service_clients(sdk)

robot = sdk.create_robot('10.0.0.3')
robot.authenticate()

# print(robot.list_services())
cam_image_client = robot.ensure_client("spot-cam-image")

sources = cam_image_client.list_image_sources()
print("Available sources:", [s.name for s in sources])


# ----------------------------STREAM IMAGES-----------------------------

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

    frame_count = 0
    start_time = time.time()

    # First frame info
    print(f"\nCapturing {source}...")
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
        
        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

    # Print stats
    print(f"\nTotal frames: {frame_count}")
    print(f"Total time: {time.time()-start_time:.2f}s")
    print(f"Average FPS: {frame_count/(time.time()-start_time):.2f} Hz")

    return image_resposes

if __name__ == '__main__':

    image_resposes = show_single('pano', quality_percent=60, resize_ratio=0.3, image_format=1, pixel_format=3)

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