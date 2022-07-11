import numpy
import os
import torch
from torchvision import transforms

from PIL import Image as PilImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

def predict(image: Image):
    model = torch.hub.load('huawei-noah/ghostnet', 'ghostnet_1x', pretrained=True)
    model.eval()
    # model = ghostnetForImageClassification.from_pretrained(ALGO_VERSION)
    # Enter line here

    preprocess = transforms.Compose([
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    input_tensor = preprocess(image)
    input_batch = input_tensor.unsqueeze(0) # create a mini-batch as expected by the model

    # move the input and model to GPU for speed if available
    if torch.cuda.is_available():
        input_batch = input_batch.to('cuda')
        model.to('cuda')

    with torch.no_grad():
        output = model(input_batch)

    # model predicts one of the 1000 ImageNet classes
    probabilities = torch.nn.functional.softmax(output[0], dim=0)
    with open("imagenet_classes.txt", "r") as f:
        categories = [s.strip() for s in f.readlines()]

    predicted_label = probabilities.argmax(-1).item()

    return predicted_label, categories[predicted_label]


class RosIO(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            '/ghostnet/image_raw',
            self.listener_callback,
            10
        )

        self.result_publisher = self.create_publisher(
            String,
            '/ghostnet/result',
            1
        )

    def listener_callback(self, msg: Image):
        # self.get_logger().info(msg.data)
        bridge = CvBridge()
        cv_image: numpy.ndarray = bridge.imgmsg_to_cv2(msg)
        # print(cv_image)
        # cv2.imshow('image', cv_image)
        # cv2.waitKey(0)
        converted_image = PilImage.fromarray(numpy.uint8(cv_image), 'RGB')
        # converted_image.show('image')
        result = str(predict(converted_image))
        print(f'Result: {result}')

        msg = String()
        msg.data = result
        self.result_publisher.publish(msg)


def main(args=None):
    print('GhostNet Started')

    rclpy.init(args=args)

    minimal_subscriber = RosIO()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
