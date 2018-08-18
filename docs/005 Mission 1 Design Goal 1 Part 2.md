# Mission 1, Design Goal 1 continued
## Access images from the Raspberry Pi Camera
As stated previously we can make use of work already carried out by the ROS community to make our life easier. The Raspberry Pi Ubuntu image I have installed includes a ROS package [raspicam_node](https://github.com/ubiquityRobotics/raspicam_node), we will make use of this package to access the camera. If you are using a different OS image on your Raspberry Pi you can still install the node from the GitHub site.
To add the node to our system we just need to include one of the supplied ROS launch files in our launch file. I'm going to use an image resolution of 1280 x 960 pixels, so adding the following to our launch file will start the raspicam_node in the required resolution.
``` XML
<include file="$(find raspicam_node)/launch/camerav2_1280x960.launch" />
```
ROS uses its own image format to pass images between nodes. In the case of raspicam_node this image is in a compressed format. Now when we come to use the image to detect and recognise a face we will be using OpenCV to process the image. We therefore need a method to convert ROS images to OpenCV images and back again. Not a problem, those nice people in the ROS community have produced a package called cv_bridge which will do the work for us. There is however a slight fly in the ointment. There are two versions of cv_bridge, one for C++ and one for Python. The C++ version works with both compressed and non-compressed ROS images but the Python version only works with non-compressed images. Now I would like to write the face recognition node in Python since 1) the pan_tilt node was written in C++ and writing the next node in Python will give us examples in both languages and 2) I already have a Python face recognition library I would like to make use of.

So that we can use Python for the face recognition node we can use another available node called republish. We will use this node to convert from a compressed image to a raw image. To start this node and attach it to the compressed image from the camera will include the following line in our launch file.
``` XML
<node name="republish" type="republish" pkg="image_transport" output="screen" args="compressed in:=/raspicam_node/image/ raw out:=/camera/image/raw" />
```
This will start the node republish, which is part of the [image_transport package](http://wiki.ros.org/image_transport), and subscribe it to the */raspicam_node/image/* topic. It will also name the published topic */camera/image/raw*, which we will subscribe to in our face recognition node.

Now I'm using the Raspberry Pi camera but you can use a different camera. You may find a ROS node is already available for your camera or you may need to wrap the camera library in a ROS node of your own. Although the topic name our face recognition node subscribes to is called */camera/image/raw*, you can always remap your topic name to match this name. We did this in the launch file used in part 1 to remap pan_tilt_node/index0_position to pan_tilt_node/head_position.
``` XML
<remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" />
```
## Detect and recognise faces
Before the system can attempt to recognise a face we need to train it with the subjects we wish to recognise. We will write two non ROS Python scripts to train our system. The first, *data_set_generator.py*, will use the camera to capture facial images of each of our subjects. The second, *training.py*, uses the images collected by the first script to do the actual training. The output of this second script is a yaml file containing the training data, the ROS node will load this training data during its initialisation. If you wish to add new people to your list of subjects you will need to rerun each script.

Our ROS package for the node is called *face_recognition* and is available in the [face_recognition repository](https://github.com/phopley/face_recognition). The sub folder *scripts* contains our two training scripts.

Each of the scripts makes use of face detection and face recognition built in to OpenCV. If you wish to fully understand how this works then may I suggest you read some of the many articles available on the internet. One of these can be found [here](https://github.com/informramiz/opencv-face-recognition-python). Not one article I came across had the code for exactly what I wanted so I have borrowed code from a number of articles. Here I'll give a high level description of each of the scripts starting with *data_set_generator.py*.

After the required imports we load the classifier using the OpenCV library, declare a helper function which we use to ensure that folders we require exist. The folder *dataset* will hold all the captured images and *trainer* will hold both the yaml file with the training data and a file containing names and ids of our subjects.
``` Python
import cv2
import os
import io
import numpy
import yaml
import picamera

# Detect object in video stream using Haarcascade Frontal Face
face_detector = cv2.CascadeClassifier('../classifiers/haarcascade_frontalface_default.xml')

def assure_path_exists(path):
  dir = os.path.dirname(path)
  if not os.path.exists(dir):
    os.makedirs(dir)

assure_path_exists("dataset/")
assure_path_exists("../trainer/")
```
Next we set the camera resolution, set up some variables including the file name which holds our list of subjects and open the file.

We then create a window to display the image read from the camera. This will enable the subject to position themselves within the camera field of view.

The script will then prompt the user for the subjects unique ID, the subjects name and whether it is low light conditions or not. The unique IDs should start at 1 and be incremented by 1 each time you add a new subject and the name is the name that you wish the robot to use for this subject. You should ideally run this script twice per subject, once in bright light and a second time in low light conditions. This will improve the chances of the recognition algorithm having success. Each run of the script will take 100 images of the subject, the file name of each image is constructed from the subject id and an image number. Image numbers are numbered 0 to 99 for those taken in bright light and 100 to 199 for those taken in low light.

The next step is to add the subject to the names file if they don't already exist.
``` Python
with picamera.PiCamera() as camera:
  camera.resolution = (1280, 960)

  looping = True
  count = 0
  end = 99
  names_dict = {}
  name_file = '../trainer/names.yml'

  # Open the file of IDs and names to append the new one to
  if os.path.exists(name_file):
    with open(name_file, 'r') as stream:
      names_dict = yaml.load(stream)

  cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

  face_id = raw_input("What is this persons ID number? ")
  name = raw_input("What is this persons name? ")
  low_light = raw_input("Low light Y/N?" )

  if low_light == 'Y' or low_light == 'y':
    count = 100
    end = 199

  # If not already in the dictionary add details
  if not face_id in names_dict:
    names_dict[int(face_id)]=name

  with open(name_file, 'w') as outfile:
    yaml.dump(names_dict, outfile, default_flow_style=False)
```
A loop is then entered to capture the images. Each pass of the loop captures an image from the camera and converts it to a numpy array for the OpenCV calls.

Using OpenCV we then convert the image to a grey scale image and attempt to detect a face in the image. If a face is detected the image is cropped around the face, the number of image samples is incremented and the cropped grey scale image is stored in the dataset folder. The original image from the camera along with a superimposed frame around the face is displayed to the user.

A check is then made to see if the user has pressed the 'q' key on the keyboard which is used to exit the program early. Otherwise a check is made to see if we have captured the one hundred images of the face, if so this main loop is exited.
``` Python
while(looping):
  # Create a memory stream so image doesn't need to be saved to a file
  stream = io.BytesIO()

  camera.capture(stream, format='jpeg')

  #Convert picture to numpy array
  buff = numpy.fromstring(stream.getvalue(), dtype=numpy.uint8)

  # Now create an OpenCV image
  image_frame = cv2.imdecode(buff, 1)

  # Convert frame to grayscale
  gray = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)

  # Detect frames of different sizes, list of faces rectangles
  faces = face_detector.detectMultiScale(gray, 1.3, 5)

  # Although faces could contain more than one face we only expect one
  # person to be in the data set image otherwise it would confuse
  # the whole thing
  if (len(faces) != 0):
    # Expecting one face only on the data set image
    (x, y, w, h) = faces[0]

    # Crop the image frame into rectangle
    cv2.rectangle(image_frame, (x,y), (x+w,y+h), (255,0,0), 4)

    # Increment sample face image
    count += 1

    # Save the captured image into the datasets folder
    cv2.imwrite("dataset/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])

    # Display the video frame, with bounded rectangle on the person's face
    cv2.imshow('frame', image_frame)

  # To stop taking video, press 'q' for at least 100ms
  if cv2.waitKey(100) & 0xFF == ord('q'):
    looping = False

  # If image taken reach 100, stop taking video
  elif count>end:
    looping = False
```
The last two lines close the window displaying the image and prints a message indicating the process is complete.
``` Python
# Close all started windows
cv2.destroyAllWindows()

print("Data prepared")
```
Once you have run the script for each subject, or if you have rerun the script for a new subject, you then run the *training.py* script.

The *training.py* script starts with the imports and the __assure_path_exists__ function definition, it then creates an instance of the OpenCV classes __LBPHFaceRecognizer_create__ and __CascadeClassifier__ using the same classifier file.
``` Python
import cv2
import os
import numpy as np

def assure_path_exists(path):
  dir = os.path.dirname(path)
  if not os.path.exists(dir):
    os.makedirs(dir)

# Create Local Binary Patterns Histograms for face recognization
recognizer = cv2.face.LBPHFaceRecognizer_create()

# Using prebuilt frontal face training model, for face detection
detector = cv2.CascadeClassifier("../classifiers/haarcascade_frontalface_default.xml");
```
The __get_images_and_labels__ function reads in each stored image, detects the face and obtains the id from the file name.
``` Python
# Create method to get the images and label data
def get_images_and_labels(path):

  # Get all file path
  image_paths = [os.path.join(path,f) for f in os.listdir(path)] 

  # Initialize empty face sample
  face_samples=[]

  # Initialize empty id
  ids = []

  # Loop all the file path
  for image_path in image_paths:

    # The stored image is grayscale so read in in gray scale
    gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Get the image id
    id = int(os.path.split(image_path)[-1].split(".")[1])

    # Get the face from the training images
    # Don't need any scaling as these images already full face
    faces = detector.detectMultiScale(gray);

    # During testing not always detected face on image, which
    # is odd as it should be just an image that was saved
    if (len(faces) == 0):
      print "No face on " + image_path

    else:
      # We know each image is only of one face
      (x, y, w, h) = faces[0]

      # Add the image to face samples
      face_samples.append(gray[y:y+h,x:x+w])

      # Add the ID to IDs
      ids.append(id)

  # Pass the face array and IDs array
  return face_samples,ids
```
Once all the faces and ids are obtained they are passed to the OpenCV face recognizer and the data from the recognizer is saved to disk. The face recognition library that will be used by our node will later load this data to train the recognizer.
``` Python
# Get the faces and IDs
faces,ids = get_images_and_labels('dataset')

# Train the model using the faces and IDs
recognizer.train(faces, np.array(ids))

# Save the model into trainer.yml
assure_path_exists('../trainer/')
recognizer.save('../trainer/trainer.yml')

print("Done")
```
The code for the ROS node itself is in the sub folder src in the file *face_recognition_node.py*. The code makes use of a library file, *face_recognition_lib.py*, which contains the class __FaceRecognition__. This file is in the sub folder *src/face_recognition_lib*.

Before describing the code for the node I'll discus the __FaceRecognition__ class. After the required imports and the declaration of the class it defines a number of functions.

The class constructor creates the OpenCV face recognizer and then reads the training file created by the training script. It then opens the file containing the list of names and the ids, and creates the classifier. It finally stores a confidence value passed to it. This value will be used to determine if the suggested ID for the face is accepted.
``` Python
def __init__(self, path, confidence):
  # Create Local Binary Patterns Histograms for face recognization
  self.__face_recognizer = cv2.face.LBPHFaceRecognizer_create()

  # Load the trained mode
  self.__face_recognizer.read(path + '/trainer/trainer.yml')

  # Load the names file
  with open(path + '/trainer/names.yml', 'r') as stream:
    self.__names_dict = yaml.load(stream)

  # Detect object in image using Haarcascade Frontal Face
  self.__face_detector = cv2.CascadeClassifier(path + '/classifiers/haarcascade_frontalface_default.xml')

  # Confidence level, the confidence of the system in recognising a face must be greater than
  # this level to be accepted by the system as a recognised face.
  self.__confidence_level = confidence
```
Two functions are declared which will be used to modify the captured image if a face is detected. The first will draw a rectangle on the image and the second will draw the supplied text on the image.
``` Python
# Function to draw rectangle on image according to given (x, y) coordinates
# and the given width and height
def draw_rectangle(self, img, rect, bgr):
  (x, y, w, h) = rect
  cv2.rectangle(img, (x, y), (x+w, y+h), bgr, 4)

# Function to draw text on give image starting at the passed (x, y) coordinates.
def draw_text(self, img, text, x, y, bgr):
  cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 3.0, bgr, 4)
```
The next function __detect_faces__, is used to detect any faces in the supplied image. It converts the image to grey scale so that OpenCV can be used to detect any faces. If faces are detected then the data for each face and a location of the face in the image is returned. Note that this part of the code is written to allow more than one face to be seen in the image.
``` Python
# This function detects any faces using OpenCV from the supplied image
def detect_faces(self, img):
  face_data = []

  #convert the test image to gray image as opencv face detector expects gray images
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  #let's detect multiscale (some images may be closer to camera than others) images
  #result is a list of faces
  faces_detected = self.__face_detector.detectMultiScale(gray, 1.3, 5);

  #if no faces are detected then return None
  if (len(faces_detected) == 0):
    return None, None

  #return only the face part of the image
  for face in faces_detected:
    (x, y, w, h) = face
    face_data.append(gray[y:y+w, x:x+h])

  # faces_detected is a list of rectangles where faces have been detected.
  # face_data is a list of the data for the faces detected

  return face_data, faces_detected
```
The final function in the class, __scan_for_faces__, is the one which will be called by the node to do the face detecting and recognition on the supplied image. This function  calls the __detect_faces__ function and if any faces are detected it loops through each face calling the OpenCV face predictor which returns the id of the recognised face and a value indicating the confidence in the prediction. This value is converted to a percentage of confidence. From the supplied id a name is obtained. A rectangle is drawn around the face along with the name of the subject and the confidence level. If the confidence level of the prediction exceeds the stored threshold value the box and text drawn on the image will be in the colour green, otherwise in the colour red. Also if the confidence threshold is exceeded a dictionary entry of the id and name will be entered. Once all the faces detected have been analysed this dictionary is returned to the caller.
``` Python
# This class function will be called from outside to scan the supplied img.
# First it attempts to detect faces in the image and then if any are found
# it attempts for recognise them against know subjects. It will adjust the
# supplied image.
def scan_for_faces(self, img):
  # First do the face detection, returned faces is a list of the faces detected
  # and rects is a list of rectangles of where the faces are in the image
  faces, rects = self.detect_faces(img)

  # Create a dictionary of IDs and Names of those detected in the image
  detected_dict = {}

  # If we detected faces then process each one
  if(faces != None):
    for index in range(len(faces)):
      # Predict the image using our face recognizer
      label, confidence = self.__face_recognizer.predict(faces[index])

      our_confidence = round(100 - confidence, 2)

      # Get name of respective label returned by face recognizer
      name_text = self.__names_dict[label]
      name_text_confidence = name_text + " {0:.2f}%".format(our_confidence)

      if(our_confidence > self.__confidence_level):
        colour = (0, 255, 0)
      else:
        colour = (0, 0, 255)

      #draw a rectangle around face(s) detected
      self.draw_rectangle(img, rects[index], colour)
      #draw name of predicted person(s) and the confidence value
      self.draw_text(img, name_text_confidence, rects[index,0], rects[index,1]-5, colour)

      if(our_confidence > self.__confidence_level):
        # Add details to the dictionary to be returned
        detected_dict[label]=name_text

  return detected_dict
```
Next I'll describe the ROS node itself.

The __main__ routine initialises ROS for the node and creates an instance of __FaceRecognitionNode__ and then calls spin to process the topics subscribed to and published.
``` Python
def main(args):
    rospy.init_node('face_recognition_node', anonymous=False)
    frn = FaceRecognitionNode()
    rospy.loginfo("Face recognition node started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
```
The rest of the file contains the __FaceRecognitionNode__ class.

The constructor for __FaceRecognitionNode__ registers that it will publish the topic *face_recognition_node/adjusted_image* which will be used to send the image with a box drawn around any faces recognised with the subject name and confidence level also printed on the image. This topic is used for testing the node.

An instance of CVBridge is created, as previously stated, this will be used to convert the ROS image to an OpenCV image.

The node subscribes to the topic *camera/image/raw* which will contain the latest image from the camera, the function __callback__ is registered to be called when a new image is received.

Next the node subscribes and publishes two more topics. We could have used an action or even a service here but I opted to use two topics. We subscribe to *face_recogntion_node/start* which is an empty message type and will be used as a trigger to run the face recognition operation on the next image received from the camera. When the message arrives on the topic the function __StartCallback__ will be called. The second topic, *face_recognition_node/result*, will contain the result of the attempt at face recognition. This topic contains a user defined message which is described later in this section.

The confidence threshold is read from the parameter server and is set to the default value of 20% if a value is not held in the parameter server.

Finally the constructor creates an instance of the __FaceRecognition__ class described above.
``` Python
class FaceRecognitionNode:

    def __init__(self):
        self.__image_pub = rospy.Publisher("face_recognition_node/adjusted_image", Image, queue_size=1)

        self.__bridge = CvBridge()
        self.__image_sub = rospy.Subscriber("camera/image/raw",Image, self.callback)

        # The calling node is probably on an action. Therefore we are going to use messages, one to
        # start the process and another to pass on the result.
        self.__command_sub = rospy.Subscriber("face_recognition_node/start",Empty, self.StartCallback)
        self.__response_pub = rospy.Publisher("face_recognition_node/result",face_recognition, queue_size=1)

        # Flag to indicate that we have been requested to use the next image
        self.__scan_next = False
        
        confidence_level = rospy.get_param('/face_rec_python/confidence_level', 20)
        rospy.loginfo("FaceRecognitionNode: Confidence level %s", str(confidence_level))
    
        # Create the face_recognition_lib class instance
        self.__frc = face_recognition_lib.FaceRecognition(roslib.packages.get_pkg_dir("face_recognition", required=True), confidence_level)
```
The function __StartCallback__ is called when a message is received on the *face_recogntion_node/start* topic. The function simply sets a flag indicating that a request to start a face recognition operation has been requested.
``` Python
# Callback for start command message
def StartCallback(self, data):
    # Indicate to use the next image for the scan
    self.__scan_next = True
```
The function __callback__ is called each time a message is received on the *camera/image/raw* topic.

If the flag is set indicating that a request for a face recognition operation was made then the received image is converted from a ROS image to an OpenCV image. A call to the __scan_for_faces__ is made to check the image for known faces. The adjusted image, which may now contain superimposed boxes and names is converted back to a ROS image and published on the *face_recognition_node/adjusted_image* topic.

The names and ids returned from the call to __scan_for_faces__ is then used to create the message for the topic *face_recognition_node/result*, which is then published. If no faces were recognised the data in the message will be empty.
``` Python
# Callback for new image received
def callback(self, data):
    if self.__scan_next == True:
        self.__scan_next = False
        # The image may show more than one face. Note that the supplied image
        # will be modified if faces are detected. The returned dictionary
        # will contain the unique IDs and Names of any subjects recognised.
        # If no detection/recognition dictionary will be empty
        image = self.__bridge.imgmsg_to_cv2(data, "bgr8")

        detected_dict = self.__frc.scan_for_faces(image)

        try:
            self.__image_pub.publish(self.__bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # Now post a message with the list of IDs and names
        ids = []
        names = []
        for k, v in detected_dict.items():
            ids.append(k)
            names.append(v)
        # Set the message to publish our custom message
        result = face_recognition()
        result.ids_detected = ids
        result.names_detected = names
        self.__response_pub.publish(result)
```
The node package also contains a *config.yaml* file which can be used to set the confidence level without having to recompile the code. The package also contains a launch file, *test.launch*, which can be used to test the node. As well as launching this node, it will launch the camera node and the node to republish the camera image in raw format.
## Face recognition message and action
As stated above the *face_recognition* package uses a user defined message to return the results of an operation looking for known faces. The package *face_recognition_msgs* contains a message, *face_recognition.msg*, and our first ROS action *scan_for_faces.action*.

The files for the package are available in the *face_recognition_msgs* folder.

We covered the creation of user messages in part 1 of this article. As usual the message file is stored in the sub folder *msg*.

The message contains an array of ids and an array of names for any recognised faces.
```
uint16[] ids_detected
string[] names_detected
```
The file for the action is stored in the *action* sub folder. An action is used when a node is required to perform a non-blocking task which may take some time. A request is sent to perform the task and a reply is received once the task is complete. Feedback during the task can also be received. The task request can also be cancelled. Here I'll just describe the action specification for the task held in this package, the actual task client/server is described later in the article. That said it is worth noting that this action will be responsible for positioning the camera and requesting the face recognition operation on the current image.

More information on ROS actions can be [found here](http://wiki.ros.org/actionlib).

An action specification contains a goal, result and feedback section. It looks similar to a message definition file except each of these parts is divided by the three dashes (---).
```
# This action scans by moving the head/camera for faces that are recognised
# There are no parameters to start the action
---
# Results of the final image/position scanned
face_recognition detected
---
# Percentage complete and result of the latest image/position scanned
float32 progress
face_recognition detected
```
Above the first three dashes is the goal. In our case we don't have any parameters for the goal, just the receipt of the goal will start our action.

The result parameters are below the first three dashes and in our case it is of type face_recognition which is defined in our  *face_recognition.msg* file. When the result of the action is returned all the faces detected will be contained in this parameter.

Below the second three dashes is the feedback parameters. In our case we will return a percentage complete of the number of face recognition operations we intend to run for the robots head movement range and the faces detected on the last individual scan.

We will run all the code together towards the end of the article but we can test our face detection node first.

Create a catkin workspace with the following terminal commands.
```
$ mkdir -p ~/rodney_ws/src
$ cd ~/rodney_ws/
$ catkin_make
```
Copy the two package folders *face_recognition* and *face_recognition_msgs* into the *~/rodney_ws/src* folder and then build the code. As a little tip I don't copy the code into the src folder but create a symbolic link in the src folder to the code location. That way I can have a number of workspaces using the same code files.
```
$ cd ~/rodney_ws/ 
$ catkin_make
```
Check that the build completes without any errors and then run the code.
```
$ cd ~/rodney_ws/
$ source devel/setup.bash
$ roslaunch face_recognition test.launch
```
With the nodes running on the Raspberry Pi I'm going to use a Linux workstation on the same network to run some test. Note: as we will use our user defined topics the code also needs to be built on this workstation. You can if you wish run the tests on the same Raspberry Pi running the system nodes.

At the workstation run the following to check that the nodes are running and connected to the correct topics. You can see the name of master in the output from running roslaunch. As I'm using the Ubiquity ROS Ubuntu image and have not changed the name my master is *ubiquityrobot*.
```
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
$ rqt_graph
```
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/part2_face_recognition_node.png" title="face recognition nodes graph">
If any topics have been misspelt in one part of the code then it will be obvious from the graph as the nodes will not be joined by the topics.

In another terminal enter the following in order to be able to view the images.
```
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
$ rqt_image_view
```
In the Image View GUI you can select the topic */camera/image/raw* to view the current camera image. For the test I'm going to select the topic */face_recognition_node/adjusted_image*, the image will currently be blank but when we request a face recognition operation we will be able to view the result.

In yet another terminal we will monitor the result topic with the following:
```
$ cd ~/rodney_ws/
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
$ source devel/setup.bash
$ rostopic echo /face_recognition_node/result
```
In the final terminal we will send a message on the */face_recognition_node/start* topic to kick off a face recognition operation.
```
$ cd ~/rodney_ws/
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
$ source devel/setup.bash
$ rostopic pub -1 /face_recognition_node/start std_msgs/Empty
```
When I published the topic without anyone in view of the camera the image viewer displayed an image of the room and the rostopic echo terminal reported empty messages with:
```
ids_detected: []
names_detected: []
---
```
When published with myself in view of the camera the rostopic echo terminal and the image viewer displayed the following:
```
ids_detected: [1]
names_detected: [Phil]
---
```
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/part2_image_view1.png" title="face recognition image 1">
When testing with two people in the image, it's trained for both these subjects, I got the following results.

```
ids_detected: [1, 2]
names_detected: [Phil, Dave]
---
```
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/part2_image_view2.png" title="face recognition image 2">
